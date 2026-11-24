#include "uac_control.h"

#ifdef USE_USB_AUDIO

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::usb_audio {

ESPHOME_LOG_TAG(TAG, "usb_audio.control");

// Descriptor types and codes from the USB 2.0 and USB Audio Class 1.0 specifications.
static constexpr uint8_t DESC_INTERFACE = 0x04;
static constexpr uint8_t DESC_CS_INTERFACE = 0x24;
static constexpr uint8_t CLASS_AUDIO = 0x01;
static constexpr uint8_t SUBCLASS_AUDIOCONTROL = 0x01;
static constexpr uint8_t SUBCLASS_AUDIOSTREAMING = 0x02;
// Audio Control interface descriptor subtypes.
static constexpr uint8_t AC_HEADER = 0x01;
static constexpr uint8_t AC_INPUT_TERMINAL = 0x02;
static constexpr uint8_t AC_OUTPUT_TERMINAL = 0x03;
static constexpr uint8_t AC_MIXER_UNIT = 0x04;
static constexpr uint8_t AC_SELECTOR_UNIT = 0x05;
static constexpr uint8_t AC_FEATURE_UNIT = 0x06;
static constexpr uint8_t AC_PROCESSING_UNIT = 0x07;
static constexpr uint8_t AC_EXTENSION_UNIT = 0x08;
// Audio Streaming interface descriptor subtype.
static constexpr uint8_t AS_GENERAL = 0x01;
// Feature Unit control selectors.
static constexpr uint8_t FU_MUTE_CONTROL = 0x01;
static constexpr uint8_t FU_VOLUME_CONTROL = 0x02;
// bmaControls bits.
static constexpr uint8_t FU_CONTROL_MUTE = 1 << 0;
static constexpr uint8_t FU_CONTROL_VOLUME = 1 << 1;
// Class-specific request to an interface, device to host and host to device.
static constexpr uint8_t REQUEST_TYPE_CLASS_INTERFACE_IN = 0xA1;
static constexpr uint8_t REQUEST_TYPE_CLASS_INTERFACE_OUT = 0x21;
static constexpr uint8_t SET_CUR = 0x01;

static constexpr size_t SETUP_SIZE = sizeof(usb_setup_packet_t);
static constexpr size_t MAX_DATA_SIZE = 8;
// Feature Unit requests take a few milliseconds; the driver itself allows 5 s.
static constexpr uint32_t CONTROL_TIMEOUT_MS = 1000;
static constexpr uint32_t FLUSH_TIMEOUT_MS = 200;
// A capture path longer than this is malformed or circular.
static constexpr uint8_t MAX_PATH_LENGTH = 16;
// Channel masks are 16 bits wide.
static constexpr uint8_t MAX_FEATURE_CHANNELS = 16;

static uint16_t read_u16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

const char *format_db(int16_t volume, char *buffer) {
  const int32_t magnitude = std::abs(static_cast<int32_t>(volume));
  // Hundredths of a dB, rounded.
  const int32_t hundredths = (magnitude * 100 + 128) / 256;
  snprintf(buffer, DB_STR_SIZE, "%c%" PRId32 ".%02" PRId32 " dB", volume < 0 ? '-' : '+', hundredths / 100,
           hundredths % 100);
  return buffer;
}

esp_err_t UACControlSession::open(uint8_t addr) {
  usb_host_client_config_t config{
      .is_synchronous = false,
      .max_num_event_msg = 5,
      .async = {.client_event_callback = &UACControlSession::client_event_cb, .callback_arg = this}};
  esp_err_t err = usb_host_client_register(&config, &this->client_);
  if (err != ESP_OK) {
    this->client_ = nullptr;
    return err;
  }
  err = usb_host_device_open(this->client_, addr, &this->device_);
  if (err != ESP_OK) {
    this->device_ = nullptr;
    this->close();
    return err;
  }
  err = usb_host_get_active_config_descriptor(this->device_, &this->config_desc_);
  if (err == ESP_OK) {
    err = usb_host_transfer_alloc(SETUP_SIZE + MAX_DATA_SIZE, 0, &this->transfer_);
  }
  if (err != ESP_OK) {
    this->transfer_ = nullptr;
    this->close();
  }
  return err;
}

void UACControlSession::close() {
  if (this->transfer_stuck_) {
    // The host stack may still complete the transfer into this memory, and a client with a transfer in flight cannot
    // close the device or deregister.
    ESP_LOGE(TAG, "Leaving a stuck control transfer and its USB host client behind");
    this->client_ = nullptr;
    this->device_ = nullptr;
    this->transfer_ = nullptr;
    return;
  }
  if (this->transfer_ != nullptr) {
    usb_host_transfer_free(this->transfer_);
    this->transfer_ = nullptr;
  }
  if (this->device_ != nullptr) {
    usb_host_device_close(this->client_, this->device_);
    this->device_ = nullptr;
  }
  this->config_desc_ = nullptr;
  if (this->client_ != nullptr) {
    // Deliver what the host library queued for this client (a device gone, a new device) before leaving. One call
    // handles every pending event; with a zero timeout it always returns ESP_OK, so it must not be looped on.
    usb_host_client_handle_events(this->client_, 0);
    esp_err_t err = usb_host_client_deregister(this->client_);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "usb_host_client_deregister failed: %s", esp_err_to_name(err));
    }
    this->client_ = nullptr;
  }
}

const uint8_t *UACControlSession::find_entity_(const uint8_t *begin, const uint8_t *end, uint8_t id) const {
  for (const uint8_t *p = begin; p + 4 <= end && p[0] >= 2 && p + p[0] <= end; p += p[0]) {
    if (p[1] != DESC_CS_INTERFACE || p[2] < AC_INPUT_TERMINAL || p[2] > AC_EXTENSION_UNIT) {
      continue;
    }
    // Terminals and units all carry their ID at offset 3.
    if (p[3] == id) {
      return p;
    }
  }
  return nullptr;
}

bool UACControlSession::find_capture_feature_unit(uint8_t stream_iface, UACFeatureUnit &unit) const {
  unit = UACFeatureUnit{};
  unit.stream_iface = stream_iface;
  if (this->config_desc_ == nullptr) {
    return false;
  }
  const auto *const config = reinterpret_cast<const uint8_t *>(this->config_desc_);
  const uint8_t *const end = config + this->config_desc_->wTotalLength;

  // The class-specific descriptors that follow the first Audio Control interface, and the terminal link of the
  // streaming interface.
  const uint8_t *ac_begin = nullptr;
  const uint8_t *ac_end = nullptr;
  bool ac_found = false;
  bool in_ac = false;
  bool in_stream = false;
  for (const uint8_t *p = config; p + 2 <= end && p[0] >= 2 && p + p[0] <= end; p += p[0]) {
    if (p[1] == DESC_INTERFACE && p[0] >= 9) {
      if (in_ac) {
        ac_end = p;
        in_ac = false;
      }
      const bool audio = p[5] == CLASS_AUDIO;
      if (audio && p[6] == SUBCLASS_AUDIOCONTROL && !ac_found) {
        ac_found = true;
        in_ac = true;
        unit.control_iface = p[2];
        ac_begin = p + p[0];
      }
      in_stream = audio && p[6] == SUBCLASS_AUDIOSTREAMING && p[2] == stream_iface;
      continue;
    }
    if (p[1] != DESC_CS_INTERFACE || p[0] < 3) {
      continue;
    }
    if (in_ac && p[2] == AC_HEADER && p[0] >= 5) {
      unit.bcd_adc = read_u16(p + 3);
    } else if (in_stream && p[2] == AS_GENERAL && p[0] >= 4 && unit.stream_terminal_id == 0) {
      unit.stream_terminal_id = p[3];
    }
  }
  if (in_ac) {
    ac_end = end;
  }
  if (!ac_found || unit.bcd_adc != 0x0100 || unit.stream_terminal_id == 0) {
    // usb_host_uac only opens UAC 1.0 devices, whose controls this follows.
    return false;
  }

  // Walk the capture path back from the streaming terminal; like usb_host_uac, take a mixer's or selector's first input
  // and the Feature Unit nearest the stream.
  const uint8_t *entity = this->find_entity_(ac_begin, ac_end, unit.stream_terminal_id);
  for (uint8_t hops = 0; entity != nullptr && hops < MAX_PATH_LENGTH; hops++) {
    const uint8_t length = entity[0];
    uint8_t source = 0;
    switch (entity[2]) {
      case AC_INPUT_TERMINAL:
        if (length >= 6) {
          unit.input_terminal_id = entity[3];
          unit.input_terminal_type = read_u16(entity + 4);
        }
        return true;
      case AC_OUTPUT_TERMINAL:
        source = length >= 8 ? entity[7] : 0;
        break;
      case AC_FEATURE_UNIT: {
        const uint8_t control_size = length >= 6 ? entity[5] : 0;
        if (unit.unit_id == 0 && control_size != 0 && length >= 7) {
          unit.unit_id = entity[3];
          const uint8_t count = std::min<uint8_t>((length - 7) / control_size, MAX_FEATURE_CHANNELS);
          unit.channels = count > 0 ? count - 1 : 0;
          for (uint8_t ch = 0; ch < count; ch++) {
            const uint8_t controls = entity[6 + ch * control_size];
            if (controls & FU_CONTROL_VOLUME) {
              unit.volume_channels |= 1 << ch;
            }
            if (controls & FU_CONTROL_MUTE) {
              unit.mute_channels |= 1 << ch;
            }
          }
        }
        source = length >= 5 ? entity[4] : 0;
        break;
      }
      case AC_MIXER_UNIT:
      case AC_SELECTOR_UNIT:
        source = (length >= 6 && entity[4] > 0) ? entity[5] : 0;
        break;
      case AC_PROCESSING_UNIT:
      case AC_EXTENSION_UNIT:
        source = (length >= 8 && entity[6] > 0) ? entity[7] : 0;
        break;
      default:
        break;
    }
    entity = source != 0 ? this->find_entity_(ac_begin, ac_end, source) : nullptr;
  }
  return true;
}

esp_err_t UACControlSession::get_volume(const UACFeatureUnit &unit, uint8_t request, uint8_t channel, int16_t &volume) {
  uint8_t data[2]{};
  esp_err_t err = this->get_(unit, request, FU_VOLUME_CONTROL, channel, data, sizeof(data));
  if (err == ESP_OK) {
    volume = static_cast<int16_t>(read_u16(data));
  }
  return err;
}

esp_err_t UACControlSession::get_volume_range(const UACFeatureUnit &unit, uint8_t channel, UACVolumeRange &range) {
  esp_err_t err = this->get_volume(unit, GET_MIN, channel, range.min);
  if (err == ESP_OK) {
    err = this->get_volume(unit, GET_MAX, channel, range.max);
  }
  if (err == ESP_OK) {
    err = this->get_volume(unit, GET_RES, channel, range.res);
  }
  if (err == ESP_OK && (range.min > range.max || range.res <= 0)) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  return err;
}

esp_err_t UACControlSession::get_mute(const UACFeatureUnit &unit, uint8_t channel, bool &muted) {
  uint8_t data = 0;
  esp_err_t err = this->get_(unit, GET_CUR, FU_MUTE_CONTROL, channel, &data, 1);
  if (err == ESP_OK) {
    muted = data != 0;
  }
  return err;
}

esp_err_t UACControlSession::set_volume(const UACFeatureUnit &unit, uint8_t channel, int16_t volume) {
  uint8_t data[2] = {static_cast<uint8_t>(volume & 0xFF), static_cast<uint8_t>((volume >> 8) & 0xFF)};
  return this->request_(false, unit, SET_CUR, FU_VOLUME_CONTROL, channel, data, sizeof(data));
}

esp_err_t UACControlSession::request_(bool in, const UACFeatureUnit &unit, uint8_t request, uint8_t selector,
                                      uint8_t channel, uint8_t *data, uint16_t length) {
  if (this->transfer_ == nullptr || this->transfer_stuck_ || length > MAX_DATA_SIZE) {
    return ESP_ERR_INVALID_STATE;
  }
  auto *setup = reinterpret_cast<usb_setup_packet_t *>(this->transfer_->data_buffer);
  setup->bmRequestType = in ? REQUEST_TYPE_CLASS_INTERFACE_IN : REQUEST_TYPE_CLASS_INTERFACE_OUT;
  setup->bRequest = request;
  setup->wValue = static_cast<uint16_t>((selector << 8) | channel);
  setup->wIndex = static_cast<uint16_t>((unit.unit_id << 8) | unit.control_iface);
  setup->wLength = length;
  this->transfer_->num_bytes = static_cast<int>(SETUP_SIZE + length);
  if (!in) {
    memcpy(this->transfer_->data_buffer + SETUP_SIZE, data, length);
  }

  esp_err_t err = this->submit_and_wait_();
  if (err != ESP_OK) {
    return err;
  }
  if (this->transfer_->actual_num_bytes != static_cast<int>(SETUP_SIZE + length)) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (in) {
    memcpy(data, this->transfer_->data_buffer + SETUP_SIZE, length);
  }
  return ESP_OK;
}

void UACControlSession::transfer_cb(usb_transfer_t *transfer) {
  // Runs inside usb_host_client_handle_events() on the calling task.
  static_cast<UACControlSession *>(transfer->context)->transfer_done_ = true;
}

bool UACControlSession::wait_(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (!this->transfer_done_) {
    if (millis() - start >= timeout_ms) {
      return false;
    }
    usb_host_client_handle_events(this->client_, pdMS_TO_TICKS(5));
  }
  return true;
}

esp_err_t UACControlSession::submit_and_wait_() {
  this->transfer_done_ = false;
  this->transfer_->device_handle = this->device_;
  this->transfer_->bEndpointAddress = 0;
  this->transfer_->callback = &UACControlSession::transfer_cb;
  this->transfer_->context = this;
  this->transfer_->timeout_ms = CONTROL_TIMEOUT_MS;
  esp_err_t err = usb_host_transfer_submit_control(this->client_, this->transfer_);
  if (err != ESP_OK) {
    return err;
  }
  if (!this->wait_(CONTROL_TIMEOUT_MS)) {
    // Like usb_host_uac on a control timeout: halt, flush and clear the default pipe, so the transfer comes back.
    usb_host_endpoint_halt(this->device_, 0);
    usb_host_endpoint_flush(this->device_, 0);
    usb_host_endpoint_clear(this->device_, 0);
    if (!this->wait_(FLUSH_TIMEOUT_MS)) {
      this->transfer_stuck_ = true;
    }
    return ESP_ERR_TIMEOUT;
  }
  switch (this->transfer_->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
      return ESP_OK;
    case USB_TRANSFER_STATUS_STALL:
      // The device refuses the request: it does not support this control or request.
      return ESP_ERR_NOT_SUPPORTED;
    case USB_TRANSFER_STATUS_NO_DEVICE:
      return ESP_ERR_INVALID_STATE;
    default:
      return ESP_FAIL;
  }
}

}  // namespace esphome::usb_audio

#endif
