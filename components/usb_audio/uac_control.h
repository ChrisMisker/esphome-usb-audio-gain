#pragma once

#include "esphome/core/defines.h"

#ifdef USE_USB_AUDIO

#include <cstddef>
#include <cstdint>

extern "C" {
#include "esp_err.h"
#include "usb/usb_host.h"
}

namespace esphome::usb_audio {

/// The Feature Unit on the capture path of one audio streaming interface, as the device's UAC 1.0 descriptors
/// describe it. Channel 0 is the master channel, 1..channels the logical channels.
struct UACFeatureUnit {
  uint16_t bcd_adc{0};
  uint8_t control_iface{0};
  uint8_t stream_iface{0};
  // The terminal the streaming interface links to (bTerminalLink), the USB streaming Output Terminal for capture.
  uint8_t stream_terminal_id{0};
  // The Input Terminal the capture path starts at, 0 when the path does not lead to one.
  uint8_t input_terminal_id{0};
  uint16_t input_terminal_type{0};
  // 0 when there is no Feature Unit on the path.
  uint8_t unit_id{0};
  uint8_t channels{0};
  uint16_t volume_channels{0};
  uint16_t mute_channels{0};

  bool has_volume(uint8_t channel) const { return (this->volume_channels >> channel) & 1; }
  bool has_mute(uint8_t channel) const { return (this->mute_channels >> channel) & 1; }
};

/// Volume range of one Feature Unit channel, in 1/256 dB like every UAC 1.0 volume value.
struct UACVolumeRange {
  int16_t min{0};
  int16_t max{0};
  int16_t res{0};
};

/// A short-lived USB host client of its own that reads a UAC 1.0 device's Audio Control descriptors and sends Feature
/// Unit requests on the device's default pipe, for what usb_host_uac does not expose: the Feature Unit and its
/// channels, the volume range and per-channel readback.
///
/// Every call blocks until the device answers, so it runs on the hub's worker only, while the worker holds the driver.
class UACControlSession {
 public:
  UACControlSession() = default;
  ~UACControlSession() { this->close(); }
  UACControlSession(const UACControlSession &) = delete;
  UACControlSession &operator=(const UACControlSession &) = delete;

  esp_err_t open(uint8_t addr);
  void close();

  /// Fills `unit` for the capture path of streaming interface `stream_iface`; false when the descriptors have no UAC
  /// 1.0 Audio Control interface or no terminal link for it. `unit.unit_id` stays 0 when the path has no Feature Unit.
  bool find_capture_feature_unit(uint8_t stream_iface, UACFeatureUnit &unit) const;

  /// `request` is one of the GET_* request codes below.
  esp_err_t get_volume(const UACFeatureUnit &unit, uint8_t request, uint8_t channel, int16_t &volume);
  esp_err_t get_volume_range(const UACFeatureUnit &unit, uint8_t channel, UACVolumeRange &range);
  esp_err_t get_mute(const UACFeatureUnit &unit, uint8_t channel, bool &muted);
  /// SET_CUR of one channel's volume, in 1/256 dB.
  esp_err_t set_volume(const UACFeatureUnit &unit, uint8_t channel, int16_t volume);

  static constexpr uint8_t GET_CUR = 0x81;
  static constexpr uint8_t GET_MIN = 0x82;
  static constexpr uint8_t GET_MAX = 0x83;
  static constexpr uint8_t GET_RES = 0x84;

 protected:
  esp_err_t get_(const UACFeatureUnit &unit, uint8_t request, uint8_t selector, uint8_t channel, uint8_t *data,
                 uint16_t length) {
    return this->request_(true, unit, request, selector, channel, data, length);
  }
  esp_err_t request_(bool in, const UACFeatureUnit &unit, uint8_t request, uint8_t selector, uint8_t channel,
                     uint8_t *data, uint16_t length);
  esp_err_t submit_and_wait_();
  bool wait_(uint32_t timeout_ms);
  const uint8_t *find_entity_(const uint8_t *begin, const uint8_t *end, uint8_t id) const;

  static void transfer_cb(usb_transfer_t *transfer);
  static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {}

  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t device_{nullptr};
  usb_transfer_t *transfer_{nullptr};
  const usb_config_desc_t *config_desc_{nullptr};
  bool transfer_done_{false};
  // A transfer the host stack never returned; its memory and the client are left alone.
  bool transfer_stuck_{false};
};

/// Formats a 1/256 dB value as "+10.50 dB" into `buffer`, which holds at least DB_STR_SIZE bytes, without using float
/// formatting on the worker's small stack.
static constexpr size_t DB_STR_SIZE = 16;
const char *format_db(int16_t volume, char *buffer);

}  // namespace esphome::usb_audio

#endif
