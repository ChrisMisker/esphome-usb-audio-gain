# ESPHome `usb_audio` with hardware capture gain

A drop-in replacement for ESPHome's built-in [`usb_audio`](https://esphome.io/components/microphone/usb_audio/)
component. It adds control of the **hardware capture gain** of a USB Audio Class (UAC 1.0) input device, set on the
device's Feature Unit.

Many USB audio inputs power up with a high default input gain. USB turntables and cheap USB audio interfaces
often start at +10 dB or more, which clips loud material inside the device's ADC, before ESPHome receives a single
sample. Software gain cannot undo that. This component sets the device's own gain, so the signal is not clipped in
the first place.

- `hardware_gain` on the `usb_audio` microphone sets a fixed gain in dB, applied when the device connects and before
  every stream starts.
- A `usb_audio` **number** platform makes the gain adjustable from Home Assistant. It always shows the value read back
  from the device, and takes its range and step from the device.
- Without either option, the component behaves exactly like the built-in one and sends no extra USB requests.

Everything else in `usb_audio` (speaker, microphone, formats, buffering) is unchanged from upstream.

## Compatibility

| | |
|---|---|
| ESPHome | **2026.11.0** (based on the `usb_audio` component of that release) |
| Framework | ESP-IDF |
| Chips | ESP32 variants with USB OTG host (ESP32-S2, ESP32-S3, ESP32-P4) |
| USB devices | UAC 1.0 devices with a Feature Unit volume control on the capture path |
| Tested with | Lenco LS-50WD V2 USB turntable (gain −10.5 to +12 dB in 1.5 dB steps) on an ESP32-S3 |

> **Note:** an external component with the same name replaces the built-in `usb_audio` completely. Fixes made to
> `usb_audio` in later ESPHome releases will not reach you while you use this component. Check this repository for a
> release that matches your ESPHome version.

## Installation

```yaml
external_components:
  - source: github://ChrisMisker/esphome-usb-audio-gain@v0.1.0
    components: [usb_audio]
```

See [`example.yaml`](example.yaml) for a complete configuration.

## Configuration

### Microphone: `hardware_gain`

```yaml
usb_audio:

microphone:
  - platform: usb_audio
    id: usb_mic
    sample_rate: 48000
    bits_per_sample: 16bit
    num_channels: 2
    hardware_gain: 0dB
```

- **hardware_gain** (*Optional*, dB): the capture gain to set on every channel of the device's capture Feature Unit.
  The value is rounded to the nearest step the device supports and clamped to its range; a warning is logged when that
  changes the value. If omitted, the device's gain is left alone, unless a gain number below is configured and has a
  value.

All other microphone options are those of the built-in component.

### Number: runtime gain

```yaml
number:
  - platform: usb_audio
    id: usb_input_gain
    name: "USB Input Gain"
    restore_value: true
```

- **usb_audio_id** (*Optional*, ID): the `usb_audio` component, when there is more than one.
- **restore_value** (*Optional*, boolean, default `false`): save the gain the device last reported and set it again
  after a reboot. A restored value takes precedence over the microphone's `hardware_gain`.
- **min_value** / **max_value** / **step** (*Optional*, defaults −30 / 30 / 0.5): the range shown until the device
  connects. After that, the number uses the minimum, maximum and resolution the device reports.
- All other [number](https://esphome.io/components/number/) options, such as `name`, `icon`, `mode` and
  `entity_category` (default `config`).

Only one gain number per `usb_audio` component is supported. The number:

- shows the gain **read back from the device**, not the requested value. If the device rejects or rounds a value, you
  see what it actually uses;
- sends a change right away, also when no audio is streaming. When the device is disconnected, the change is kept and
  applied when it connects;
- works with the standard `number.set` action, so no custom action is needed:

  ```yaml
  - number.set:
      id: usb_input_gain
      value: 0
  ```

Home Assistant reads the range of an entity when it connects to the device. At boot, the USB device is normally found
before Home Assistant connects, so the device's range shows up right away. If you plug the device in later, reload the
ESPHome integration to see its range.

## Logs

With `hardware_gain` or a gain number configured, the component logs what it finds when the device connects:

```text
[I][usb_audio]: USB Audio Capture Device connected
[I][usb_audio]:   VID: 0x2034, PID: 0x0105
[I][usb_audio]:   UAC version: 1.00
[I][usb_audio]:   Capture path: Input Terminal <id> (type <type>) -> Feature Unit <id> -> streaming terminal <id>
[I][usb_audio]:   Channel 1 gain: +10.50 dB
[I][usb_audio]:   Channel 1 minimum gain: -10.50 dB, maximum gain: +12.00 dB, gain resolution: +1.50 dB
[I][usb_audio]: USB Audio capture gain before (Channel 1): +10.50 dB
[I][usb_audio]: Setting Channel 1 capture gain to +0.00 dB
[I][usb_audio]: Channel 1 gain readback: +0.00 dB
[I][usb_audio]: Hardware capture gain successfully configured
```

(Values from the tested turntable, with one channel shown; IDs and terminal types depend on the device.) "successfully configured" is logged only when every channel
reads back within half a step of the requested value. Otherwise an error is logged and audio keeps streaming at
whatever gain the device has. There is no software fallback.

A device without a volume control on its capture path logs one error, `the device has no volume control on its capture
path`, and otherwise works normally.

## How it works

- The capture Feature Unit is found from the device's own descriptors. The code walks back from the USB streaming
  terminal of the microphone interface, like Espressif's `usb_host_uac` driver does. No unit or interface IDs are
  hard-coded.
- Gain requests are UAC 1.0 class requests on the default control pipe: `SET_CUR`, plus `GET_CUR`/`GET_MIN`/`GET_MAX`/
  `GET_RES` for Volume Control, values in 1/256 dB. They are sent from a short-lived USB host client of the component's
  own, because `usb_host_uac` 1.5.0 does not expose the Feature Unit, the volume range or per-channel readback, and only
  accepts volume requests while a stream is started. The driver itself is not modified.
- All USB control transfers run on the component's existing worker task, which already keeps microphone reads out of the
  driver while it works, so they never race with streaming, stopping or a disconnect.
- On each stream start the stream is opened suspended (`FLAG_STREAM_SUSPEND_AFTER_START`), the gain is set and read
  back, and only then is the stream resumed. The first sample is therefore already captured at the new gain.

## Limitations

- Every channel with a volume control gets the same value, including a master channel if the device has one. On a
  device with both a master and per-channel controls, the gain therefore applies twice.
- If the capture path has a mixer or selector unit, its first input is followed, like `usb_host_uac` does.
- Only UAC 1.0, the only class version `usb_host_uac` supports.
- No hardware mute control yet.

## Background

This addresses the feature request in
[esphome/discussions#3973](https://github.com/orgs/esphome/discussions/3973). The plan is to propose it for inclusion in
ESPHome itself; [`docs/upstream-2026.11.0.patch`](docs/upstream-2026.11.0.patch) is the full diff against the
`usb_audio` component of ESPHome 2026.11.0.

## License

Derived from ESPHome. As in ESPHome, the C++ code is licensed under the GPLv3 and the Python code under the MIT
license; see [LICENSE](LICENSE).
