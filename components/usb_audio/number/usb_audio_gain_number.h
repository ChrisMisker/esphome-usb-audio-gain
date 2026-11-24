#pragma once

#include "esphome/core/defines.h"

#ifdef USE_USB_AUDIO

#include "esphome/components/number/number.h"
#include "esphome/components/usb_audio/usb_audio.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome::usb_audio {

/// The microphone's hardware capture gain in dB, set on every channel. It always shows what the device reports back,
/// and takes its range and step from the device once it connects.
class USBAudioGainNumber : public number::Number, public Component, public Parented<USBAudioComponent> {
 public:
  void setup() override;
  void dump_config() override;
  // After the hub, which it registers with.
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }

 protected:
  void control(float value) override;
  void on_gain_state_(const CaptureGainState &state);
  void save_(float value);

  bool restore_value_{false};
  ESPPreferenceObject pref_;
  float saved_{NAN};
};

}  // namespace esphome::usb_audio

#endif
