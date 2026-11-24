# Changelog

## v0.1.0

First release, based on the `usb_audio` component of ESPHome **2026.11.0**.

### Added

- `hardware_gain` option on the `usb_audio` microphone: sets the hardware capture gain (in dB) on the USB device's
  Feature Unit when it connects and before every stream starts. The value is rounded and clamped to what the device
  supports, then read back to confirm.
- `usb_audio` number platform: the capture gain as a Home Assistant entity. It shows the value read back from the
  device, takes its range and step from the device, can be changed while streaming, and can optionally be restored
  after a reboot. Works with the standard `number.set` action.
- Diagnostic logging of the capture path when the device connects: VID/PID, audio formats, UAC version, the terminal
  and Feature Unit chain, and per channel the current, minimum, maximum and resolution of the gain, plus mute state.

### Notes

- Replaces the built-in `usb_audio` component. Without `hardware_gain` or a gain number it behaves exactly like the
  built-in one and sends no extra USB requests.
- UAC 1.0 devices only. Every channel with a volume control gets the same gain.
- Tested with a Lenco LS-50WD V2 USB turntable on an ESP32-S3. Its default +10.5 dB input gain clipped loud
  records; at 0 dB the clipping was gone.
