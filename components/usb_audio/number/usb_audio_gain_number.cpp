#include "usb_audio_gain_number.h"

#ifdef USE_USB_AUDIO

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome::usb_audio {

ESPHOME_LOG_TAG(TAG, "usb_audio.number");

// UAC 1.0 volume is a signed 16-bit value in 1/256 dB.
static constexpr float GAIN_LIMIT_DB = 32767.0f / 256.0f;

void USBAudioGainNumber::setup() {
  this->parent_->add_on_microphone_gain_callback(
      [this](const CaptureGainState &state) { this->on_gain_state_(state); });
  if (!this->restore_value_) {
    return;
  }
  this->pref_ = this->make_entity_preference<float>();
  float value;
  if (this->pref_.load(&value) && std::isfinite(value)) {
    // Overrides the microphone's hardware_gain.
    this->saved_ = value;
    this->parent_->request_microphone_gain(static_cast<int16_t>(std::lround(value * 256.0f)));
    this->publish_state(value);
  }
}

void USBAudioGainNumber::dump_config() {
  LOG_NUMBER("", "USB Audio Gain", this);
  ESP_LOGCONFIG(TAG, "  Restore value: %s", YESNO(this->restore_value_));
}

void USBAudioGainNumber::control(float value) {
  value = std::clamp(value, -GAIN_LIMIT_DB, GAIN_LIMIT_DB);
  this->parent_->request_microphone_gain(static_cast<int16_t>(std::lround(value * 256.0f)));
  if (!this->parent_->is_microphone_connected()) {
    // Nothing reports back until the device connects; show and keep the request meanwhile.
    this->publish_state(value);
    this->save_(value);
  }
  // Otherwise the worker sets it and the readback publishes the value the device actually took.
}

void USBAudioGainNumber::on_gain_state_(const CaptureGainState &state) {
  // Home Assistant picks up a changed range when it next connects; at boot the device usually connects first.
  this->traits.set_min_value(state.min / 256.0f);
  this->traits.set_max_value(state.max / 256.0f);
  this->traits.set_step(state.res / 256.0f);
  const float value = state.current / 256.0f;
  this->publish_state(value);
  this->save_(value);
}

void USBAudioGainNumber::save_(float value) {
  if (this->restore_value_ && value != this->saved_) {
    this->saved_ = value;
    this->pref_.save(&value);
  }
}

}  // namespace esphome::usb_audio

#endif
