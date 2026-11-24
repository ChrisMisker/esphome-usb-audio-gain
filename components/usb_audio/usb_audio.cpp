#include "usb_audio.h"

#ifdef USE_USB_AUDIO

#include <algorithm>
#include <cinttypes>
#include <cmath>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "uac_control.h"

extern "C" {
#include "esp_err.h"
}

namespace esphome::usb_audio {

ESPHOME_LOG_TAG(TAG, "usb_audio");

// The driver task completes and resubmits the speaker's transfers, with only CONFIG_UAC_NUM_ISOC_URBS of them
// queued on the bus, so it runs above the network stack like the I2S speaker task; each wakeup is short.
static constexpr UBaseType_t UAC_TASK_PRIORITY = 19;
static constexpr uint32_t UAC_TASK_STACK_SIZE = 4096;
// Below the driver's task, so its transfers are not held up by control requests.
static constexpr UBaseType_t WORKER_TASK_PRIORITY = 5;
static constexpr uint32_t WORKER_TASK_STACK_SIZE = 4096;
// The worker retries volume and mute up to this many times, this far apart.
static constexpr uint8_t MAX_CONTROL_ATTEMPTS = 5;
static constexpr uint32_t CONTROL_RETRY_MS = 100;
// A failed close is retried up to this many times, this far apart, before the handle is given up.
static constexpr uint8_t MAX_CLOSE_ATTEMPTS = 5;
static constexpr uint32_t CLOSE_RETRY_MS = 500;

static const LogString *endpoint_name(Endpoint endpoint) {
  return endpoint == Endpoint::SPEAKER ? LOG_STR("Speaker") : LOG_STR("Microphone");
}

static uint8_t discrete_rate_count(const uac_host_dev_alt_param_t &alt) {
  return std::min<uint8_t>(alt.sample_freq_type, UAC_FREQ_NUM_MAX);
}

static bool alt_supports_rate(const uac_host_dev_alt_param_t &alt, uint32_t sample_rate) {
  if (alt.sample_freq_type == 0) {
    return sample_rate >= alt.sample_freq_lower && sample_rate <= alt.sample_freq_upper;
  }
  const uint8_t count = discrete_rate_count(alt);
  return std::find(alt.sample_freq, alt.sample_freq + count, sample_rate) != alt.sample_freq + count;
}

static void log_alt_settings(uac_host_device_handle_t handle, uint8_t alt_num) {
  for (uint8_t alt = 1; alt <= alt_num; alt++) {
    uac_host_dev_alt_param_t alt_param{};
    if (uac_host_get_device_alt_param(handle, alt, &alt_param) != ESP_OK) {
      continue;
    }
    if (alt_param.sample_freq_type == 0) {
      ESP_LOGI(TAG, "  alt %u: channels=%u bits=%u subframe=%u bytes sample rate range=%" PRIu32 "-%" PRIu32 " Hz", alt,
               alt_param.channels, alt_param.bit_resolution, alt_param.subframe_size, alt_param.sample_freq_lower,
               alt_param.sample_freq_upper);
      continue;
    }
    ESP_LOGI(TAG, "  alt %u: channels=%u bits=%u subframe=%u bytes sample rates:", alt, alt_param.channels,
             alt_param.bit_resolution, alt_param.subframe_size);
    const uint8_t count = discrete_rate_count(alt_param);
    for (uint8_t i = 0; i < count; i++) {
      // Some devices list a rate more than once.
      if (std::find(alt_param.sample_freq, alt_param.sample_freq + i, alt_param.sample_freq[i]) ==
          alt_param.sample_freq + i) {
        ESP_LOGI(TAG, "    %" PRIu32 " Hz", alt_param.sample_freq[i]);
      }
    }
  }
}

void USBAudioComponent::setup() {
  this->event_queue_ = xQueueCreateStatic(EVENT_QUEUE_LENGTH, sizeof(USBAudioEvent), this->event_queue_storage_.data(),
                                          &this->event_queue_struct_);
  this->op_queue_ =
      xQueueCreateStatic(OP_QUEUE_LENGTH, sizeof(Op), this->op_queue_storage_.data(), &this->op_queue_struct_);

  uac_host_driver_config_t uac_cfg = {
      .create_background_task = true,
      .task_priority = UAC_TASK_PRIORITY,
      .stack_size = UAC_TASK_STACK_SIZE,
      .core_id = 0,
      .callback = &USBAudioComponent::driver_event_stub,
      .callback_arg = this,
  };

  esp_err_t err = uac_host_install(&uac_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uac_host_install failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  this->uac_installed_ = true;

  // Opening, starting, suspending and closing streams and setting volume are control transfers that can block for
  // seconds, so they run here instead of on the main loop.
  if (!this->worker_.create(&USBAudioComponent::worker_task, "usb_audio", WORKER_TASK_STACK_SIZE, this,
                            WORKER_TASK_PRIORITY, false)) {
    this->mark_failed(LOG_STR("Failed to create worker task"));
    return;
  }
  // The USB callbacks and the worker wake the loop when there is work.
  this->disable_loop();
}

void USBAudioComponent::loop() {
  this->process_events_(App.get_loop_component_start_time());

  if (const uint32_t errors = this->transfer_errors_.exchange(0); errors != 0) {
    ESP_LOGW(TAG, "%" PRIu32 " USB transfer errors", errors);
  }

  // Stay awake while a lost device still waits to be closed.
  if (std::any_of(this->endpoints_.begin(), this->endpoints_.end(),
                  [](const EndpointState &ep) { return ep.closing != nullptr; })) {
    return;
  }
  this->disable_loop();
}

void USBAudioComponent::process_events_(uint32_t now) {
  this->finish_closes_(now);

  USBAudioEvent event;
  // Only the events already queued: one posted again while this runs waits for the next loop().
  for (UBaseType_t queued = uxQueueMessagesWaiting(this->event_queue_);
       queued > 0 && xQueueReceive(this->event_queue_, &event, 0) == pdTRUE; queued--) {
    this->process_event_(event, now);
  }
}

void USBAudioComponent::dump_config() {
  const auto &speaker = this->endpoint_(Endpoint::SPEAKER);
  const auto &microphone = this->endpoint_(Endpoint::MICROPHONE);
  ESP_LOGCONFIG(TAG,
                "USB Audio:\n"
                "  Device connected: %s\n"
                "  Speaker registered: %s\n"
                "  Microphone registered: %s",
                YESNO(speaker.handle.load() != nullptr || microphone.handle.load() != nullptr),
                YESNO(speaker.platform != nullptr), YESNO(microphone.platform != nullptr));
}

bool USBAudioComponent::teardown() {
  if (this->event_queue_ == nullptr) {
    return true;
  }
  // Stop the platforms first, then let the worker close each device once its platform is idle.
  this->begin_close_(Endpoint::SPEAKER);
  this->begin_close_(Endpoint::MICROPHONE);
  // The loop time does not advance while components tear down.
  const uint32_t now = millis();
  this->process_events_(now);
  if (!this->finish_closes_(now)) {
    return false;
  }
  // Wait for the worker to finish whatever it was still running.
  if (uxQueueMessagesWaiting(this->op_queue_) != 0 || this->driver_busy_) {
    return false;
  }
  xQueueReset(this->event_queue_);

  if (this->uac_installed_) {
    esp_err_t err = uac_host_uninstall();
    if (err == ESP_ERR_INVALID_STATE) {
      return false;
    }
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "uac_host_uninstall returned %s", esp_err_to_name(err));
    }
    this->uac_installed_ = false;
  }
  return true;
}

void USBAudioComponent::resume(Endpoint endpoint) {
  auto &ep = this->endpoint_(endpoint);
  ep.active = true;
  ep.request_seq++;
  const uac_host_device_handle_t handle = ep.handle;
  if (handle == nullptr) {
    if (ep.open_failed && !ep.open_pending) {
      // The device is still plugged in but its last open failed; try again for this start.
      ep.open_failed = false;
      this->open_stream_(endpoint, ep.failed_addr, ep.failed_iface);
      return;
    }
    ESP_LOGD(TAG, "%s interface not connected yet", LOG_STR_ARG(endpoint_name(endpoint)));
    return;
  }
  if (ep.start_pending) {
    // The pending start reports back to the platform, and checks `active` first.
    return;
  }
  if (!ep.stream_started) {
    if (ep.start_rejected) {
      // Already probed and logged for this device; a retrying caller must not flood the log.
      this->stream_failed_(endpoint);
      return;
    }
    this->start_stream_(endpoint);
    return;
  }
  // Queued even while an earlier resume or suspend is pending: the worker runs them in order, so the driver ends in
  // the latest requested state, and only this request's RESUMED reports back.
  Op op;
  op.type = Op::Type::RESUME;
  op.endpoint = endpoint;
  op.handle = handle;
  op.seq = ep.request_seq;
  if (!this->queue_op_(op)) {
    this->stream_failed_(endpoint);
  }
}

void USBAudioComponent::suspend(Endpoint endpoint) {
  auto &ep = this->endpoint_(endpoint);
  ep.active = false;
  ep.request_seq++;
  const uac_host_device_handle_t handle = ep.handle;
  // A start still running on the worker is suspended once its STARTED arrives.
  if (!ep.stream_started || handle == nullptr) {
    return;
  }
  Op op;
  op.type = Op::Type::SUSPEND;
  op.endpoint = endpoint;
  op.handle = handle;
  this->queue_op_(op);
}

bool USBAudioComponent::enter_driver_(uint32_t &timeout_ms) {
  const uint32_t start = millis();
  while (true) {
    // Counted before the busy check, so the worker waits for this call to leave the driver before it takes it.
    this->driver_users_.fetch_add(1);
    if (!this->driver_busy_) {
      const uint32_t waited = millis() - start;
      timeout_ms = waited < timeout_ms ? timeout_ms - waited : 0;
      return true;
    }
    this->driver_users_.fetch_sub(1);
    // A control request takes a few milliseconds, so go on as soon as the worker is done with it.
    if (millis() - start >= timeout_ms) {
      return false;
    }
    vTaskDelay(1);
  }
}

esp_err_t USBAudioComponent::read_microphone(uint8_t *buffer, size_t size, size_t *bytes_read, uint32_t timeout_ms) {
  *bytes_read = 0;
  if (!this->enter_driver_(timeout_ms)) {
    return ESP_ERR_TIMEOUT;
  }
  uac_host_device_handle_t handle = this->endpoint_(Endpoint::MICROPHONE).handle;
  if (handle == nullptr) {
    this->driver_users_.fetch_sub(1);
    return ESP_ERR_INVALID_STATE;
  }
  // The driver collects up to `size` bytes for the whole timeout and returns what it got, or ESP_ERR_TIMEOUT when
  // nothing arrived.
  uint32_t actual = 0;
  esp_err_t err = uac_host_device_read(handle, buffer, static_cast<uint32_t>(size), &actual, pdMS_TO_TICKS(timeout_ms));
  this->driver_users_.fetch_sub(1);
  *bytes_read = actual;
  return err;
}

esp_err_t USBAudioComponent::write_speaker(const uint8_t *data, size_t length, uint32_t timeout_ms) {
  if (!this->enter_driver_(timeout_ms)) {
    // A non-blocking caller retries later.
    return ESP_ERR_TIMEOUT;
  }
  uac_host_device_handle_t handle = this->endpoint_(Endpoint::SPEAKER).handle;
  if (handle == nullptr) {
    this->driver_users_.fetch_sub(1);
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = uac_host_device_write(handle, const_cast<uint8_t *>(data), static_cast<uint32_t>(length),
                                        pdMS_TO_TICKS(timeout_ms));
  this->driver_users_.fetch_sub(1);
  if (err == ESP_FAIL || err == ESP_ERR_INVALID_STATE) {
    // Ring buffer full or stream inactive; report as timeout so callers back off and retry.
    err = ESP_ERR_TIMEOUT;
  }
  return err;
}

void USBAudioComponent::process_event_(const USBAudioEvent &event, uint32_t now) {
  switch (event.type) {
    case USBAudioEvent::Type::CONNECTED: {
      auto &ep = this->endpoint_(event.endpoint);
      if (ep.closing != nullptr) {
        // A device that came back waits until its old handle is closed; the queue keeps draining meanwhile, so the
        // worker's CLOSED can arrive.
        ep.connect_deferred = true;
        ep.deferred_addr = event.addr;
        ep.deferred_iface = event.iface_num;
        break;
      }
      this->open_stream_(event.endpoint, event.addr, event.iface_num);
      break;
    }
    case USBAudioEvent::Type::DISCONNECTED:
      this->handle_disconnect_(event.handle);
      break;
    case USBAudioEvent::Type::OPENED:
      this->on_opened_(event);
      break;
    case USBAudioEvent::Type::STARTED:
      this->on_started_(event);
      break;
    case USBAudioEvent::Type::RESUMED:
      this->on_resumed_(event);
      break;
    case USBAudioEvent::Type::CLOSED:
      this->on_closed_(event, now);
      break;
    case USBAudioEvent::Type::GAIN: {
      CaptureGainState state;
      {
        LockGuard lock(this->gain_lock_);
        state = this->gain_state_;
      }
      this->gain_callbacks_.call(state);
      break;
    }
  }
}

void USBAudioComponent::open_stream_(Endpoint endpoint, uint8_t addr, uint8_t iface_num) {
  auto &ep = this->endpoint_(endpoint);
  const LogString *name = endpoint_name(endpoint);
  if (ep.platform == nullptr) {
    ESP_LOGD(TAG, "%s interface connected but not configured", LOG_STR_ARG(name));
    return;
  }
  if (ep.open_pending) {
    // Either a duplicate, or a replacement for a device that went away during the pending open; on_opened_() tells
    // them apart and replays a replacement.
    ep.connect_deferred = true;
    ep.deferred_addr = addr;
    ep.deferred_iface = iface_num;
    return;
  }
  if (ep.handle.load() != nullptr) {
    ESP_LOGW(TAG, "%s interface already open, ignoring duplicate event", LOG_STR_ARG(name));
    return;
  }
  Op op;
  op.type = Op::Type::OPEN;
  op.endpoint = endpoint;
  op.addr = addr;
  op.iface_num = iface_num;
  if (!this->queue_op_(op)) {
    // The event is already off the queue, so post it again for a later loop() instead of losing the device.
    USBAudioEvent event;
    event.type = USBAudioEvent::Type::CONNECTED;
    event.endpoint = endpoint;
    event.addr = addr;
    event.iface_num = iface_num;
    this->post_event_(event);
    return;
  }
  ep.open_pending = true;
}

void USBAudioComponent::on_opened_(const USBAudioEvent &event) {
  auto &ep = this->endpoint_(event.endpoint);
  ep.open_pending = false;
  bool lost_meanwhile = false;
  for (auto &lost : this->lost_while_opening_) {
    if (event.ok && lost == event.handle) {
      lost = nullptr;
      lost_meanwhile = true;
    }
  }
  if (std::none_of(this->endpoints_.begin(), this->endpoints_.end(),
                   [](const EndpointState &other) { return other.open_pending; })) {
    // Only a pending open can still claim an entry; a failed one reports no handle, so its entry would stay behind
    // and could match a later device that reuses the freed handle.
    this->lost_while_opening_.fill(nullptr);
  }
  if (!event.ok) {
    if (ep.connect_deferred) {
      // A newer connect arrived during the failed open; try that one.
      ep.connect_deferred = false;
      this->open_stream_(event.endpoint, ep.deferred_addr, ep.deferred_iface);
      return;
    }
    // Keep where the device is, so the next start() retries the open without a replug.
    ep.open_failed = true;
    ep.failed_addr = event.addr;
    ep.failed_iface = event.iface_num;
    if (ep.active) {
      this->stream_failed_(event.endpoint);
    }
    return;
  }
  ep.open_failed = false;
  ep.handle = event.handle;
  ep.iface = event.iface_num;
  ESP_LOGI(TAG, "%s interface %u connected (addr=%u)", LOG_STR_ARG(endpoint_name(event.endpoint)), ep.iface,
           event.addr);
  if (lost_meanwhile) {
    // It went away while the worker was opening it; a connect deferred meanwhile is opened once it is closed.
    this->handle_disconnect_(event.handle);
    return;
  }
  if (event.endpoint == Endpoint::MICROPHONE && this->gain_pending_) {
    // A gain request arrived after the open had read the requests, while the handle was not published yet.
    this->queue_gain_op_();
  }
  // The device that was being opened is still here, so a connect deferred meanwhile was a duplicate.
  ep.connect_deferred = false;
  if (ep.active) {
    this->start_stream_(event.endpoint);
  }
}

void USBAudioComponent::start_stream_(Endpoint endpoint) {
  auto &ep = this->endpoint_(endpoint);
  Op op;
  op.type = Op::Type::START;
  op.endpoint = endpoint;
  op.handle = ep.handle;
  if (!this->queue_op_(op)) {
    this->stream_failed_(endpoint);
    return;
  }
  ep.start_pending = true;
}

void USBAudioComponent::on_started_(const USBAudioEvent &event) {
  auto &ep = this->endpoint_(event.endpoint);
  ep.start_pending = false;
  if (event.handle != ep.handle.load()) {
    // The device went away meanwhile; the platform was already told.
    return;
  }
  if (!event.ok) {
    ep.start_rejected = event.rejected;
    if (ep.active) {
      this->stream_failed_(event.endpoint);
    }
    return;
  }
  ep.stream_started = true;
  if (event.endpoint == Endpoint::SPEAKER) {
    // Send the current volume and mute to the newly started stream.
    this->volume_.pending = true;
    this->mute_.pending = true;
    this->queue_controls_();
  }
  if (!ep.active) {
    // The platform stopped while the stream was starting.
    this->suspend(event.endpoint);
    return;
  }
  ep.platform->on_stream_state(StreamEvent::RUNNING);
}

void USBAudioComponent::on_resumed_(const USBAudioEvent &event) {
  auto &ep = this->endpoint_(event.endpoint);
  if (event.handle != ep.handle.load() || event.seq != ep.request_seq || !ep.active) {
    // The device went away, or a later suspend or resume is queued behind this one and decides the state.
    return;
  }
  if (!event.ok) {
    this->stream_failed_(event.endpoint);
    return;
  }
  ep.platform->on_stream_state(StreamEvent::RUNNING);
}

void USBAudioComponent::stream_failed_(Endpoint endpoint) {
  auto &ep = this->endpoint_(endpoint);
  // The platform goes back to stopped, so a reconnect must not restart the stream; its next start() retries.
  ep.active = false;
  ep.platform->on_stream_state(StreamEvent::FAILED);
}

void USBAudioComponent::set_speaker_volume_level(float volume) {
  this->speaker_volume_ = static_cast<uint8_t>(std::lround(std::clamp(volume, 0.0f, 1.0f) * 100.0f));
  this->volume_.pending = true;
  this->queue_controls_();
}

void USBAudioComponent::set_speaker_mute_state(bool mute_state) {
  this->speaker_muted_ = mute_state;
  this->mute_.pending = true;
  this->queue_controls_();
}

void USBAudioComponent::queue_controls_() {
  if (this->op_queue_ == nullptr) {
    return;
  }
  if (this->controls_queued_.exchange(true)) {
    // The queued op sends the latest values, so lifecycle ops keep their queue slots.
    return;
  }
  Op op;
  op.type = Op::Type::CONTROLS;
  if (xQueueSend(this->op_queue_, &op, 0) != pdTRUE) {
    // The controls stay pending and the next start or change queues them again.
    this->controls_queued_ = false;
  }
}

void USBAudioComponent::handle_disconnect_(uac_host_device_handle_t handle) {
  for (size_t i = 0; i < this->endpoints_.size(); i++) {
    auto &ep = this->endpoints_[i];
    if (handle != nullptr && ep.handle.load() == handle) {
      const auto endpoint = static_cast<Endpoint>(i);
      ESP_LOGI(TAG, "%s interface %u disconnected", LOG_STR_ARG(endpoint_name(endpoint)), ep.iface);
      this->begin_close_(endpoint);
      return;
    }
  }
  if (std::any_of(this->endpoints_.begin(), this->endpoints_.end(),
                  [](const EndpointState &ep) { return ep.open_pending; })) {
    // Its open is still running on the worker; close it once it reports back.
    for (auto &lost : this->lost_while_opening_) {
      if (lost == nullptr || lost == handle) {
        lost = handle;
        return;
      }
    }
    // Both endpoints are already waiting on a lost open; the oldest one is the least likely to still report back.
    this->lost_while_opening_[0] = handle;
    return;
  }
  ESP_LOGD(TAG, "Disconnect event for an interface with no active stream");
}

void USBAudioComponent::begin_close_(Endpoint endpoint) {
  auto &ep = this->endpoint_(endpoint);
  // Platform reads and writes see nullptr from here on and return at once.
  const uac_host_device_handle_t handle = ep.handle.exchange(nullptr);
  if (handle == nullptr) {
    return;
  }
  ep.closing = handle;
  ep.close_queued = false;
  // `active` is kept, so the stream restarts when the device returns.
  ep.platform->on_stream_state(StreamEvent::LOST);
}

bool USBAudioComponent::finish_closes_(uint32_t now) {
  bool done = true;
  for (size_t i = 0; i < this->endpoints_.size(); i++) {
    auto &ep = this->endpoints_[i];
    if (ep.closing == nullptr) {
      continue;
    }
    done = false;
    // Closing frees the driver's interface, so wait until no platform task can still be using the handle.
    if (ep.close_queued || !ep.platform->stream_idle()) {
      continue;
    }
    if (ep.close_attempts > 0 && static_cast<int32_t>(now - ep.close_retry_ms) < 0) {
      // A failed close waits a moment before it is retried.
      continue;
    }
    Op op;
    op.type = Op::Type::CLOSE;
    op.endpoint = static_cast<Endpoint>(i);
    op.handle = ep.closing;
    op.stop = ep.stream_started;
    ep.close_queued = this->queue_op_(op);
  }
  return done;
}

void USBAudioComponent::on_closed_(const USBAudioEvent &event, uint32_t now) {
  auto &ep = this->endpoint_(event.endpoint);
  ep.close_queued = false;
  if (!event.ok && ++ep.close_attempts < MAX_CLOSE_ATTEMPTS) {
    // Keep the handle, so finish_closes_() retries the close.
    ep.close_retry_ms = now + CLOSE_RETRY_MS;
    return;
  }
  if (!event.ok) {
    ESP_LOGE(TAG, "Giving up closing the %s interface after %u attempts", LOG_STR_ARG(endpoint_name(event.endpoint)),
             ep.close_attempts);
  }
  ep.close_attempts = 0;
  ep.closing = nullptr;
  ep.stream_started = false;
  ep.start_rejected = false;
  ep.start_pending = false;
  if (ep.connect_deferred) {
    // The device came back while its old handle was closing.
    ep.connect_deferred = false;
    this->open_stream_(event.endpoint, ep.deferred_addr, ep.deferred_iface);
  }
}

bool USBAudioComponent::queue_op_(const Op &op) {
  if (xQueueSend(this->op_queue_, &op, 0) != pdTRUE) {
    ESP_LOGW(TAG, "USB audio work queue full, dropping %s request", LOG_STR_ARG(endpoint_name(op.endpoint)));
    return false;
  }
  return true;
}

void USBAudioComponent::post_event_(const USBAudioEvent &event) {
  if (xQueueSend(this->event_queue_, &event, 0) != pdTRUE) {
    ESP_LOGW(TAG, "USB audio event queue full, dropping event");
  }
  this->enable_loop_soon_any_context();
}

void USBAudioComponent::worker_task(void *param) {
  auto *self = static_cast<USBAudioComponent *>(param);
  Op op;
  while (true) {
    if (xQueueReceive(self->op_queue_, &op, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    self->lock_driver_();
    self->run_op_(op);
    self->unlock_driver_();
  }
}

void USBAudioComponent::lock_driver_() {
  this->driver_busy_ = true;
  // Reads and writes give up within their timeout; wait for those already inside the driver.
  while (this->driver_users_ != 0) {
    vTaskDelay(1);
  }
}

void USBAudioComponent::unlock_driver_() { this->driver_busy_ = false; }

void USBAudioComponent::run_op_(const Op &op) {
  const LogString *name = endpoint_name(op.endpoint);
  USBAudioEvent result;
  result.endpoint = op.endpoint;
  result.handle = op.handle;

  switch (op.type) {
    case Op::Type::OPEN: {
      uint32_t buffer_size = this->endpoint_(op.endpoint).platform->get_buffer_size();
      uint32_t threshold = std::max<uint32_t>(buffer_size / 4, 1U);
      if (op.endpoint == Endpoint::SPEAKER) {
        // The driver raises TX_DONE after taking a transfer only while the ring buffer holds at most the threshold,
        // and it needs the threshold below the size. The speaker never fills past its buffer_size, so it hears about
        // every transfer even when a waiting write lands right after one.
        threshold = buffer_size;
        buffer_size += SPEAKER_RING_SLACK;
      }
      uac_host_device_config_t dev_cfg = {
          .addr = op.addr,
          .iface_num = op.iface_num,
          .buffer_size = buffer_size,
          .buffer_threshold = threshold,
          .callback = &USBAudioComponent::device_event_stub,
          .callback_arg = this,
      };
      uac_host_device_handle_t handle{nullptr};
      esp_err_t err = uac_host_device_open(&dev_cfg, &handle);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open %s interface: %s", LOG_STR_ARG(name), esp_err_to_name(err));
      } else if (op.endpoint == Endpoint::MICROPHONE && this->gain_control_) {
        this->log_capture_controls_(handle);
        // Set it as soon as the device is here, so the gain number shows the real value before any stream.
        this->sync_microphone_gain_(handle, true);
      }
      result.type = USBAudioEvent::Type::OPENED;
      result.ok = err == ESP_OK;
      result.handle = handle;
      result.addr = op.addr;
      result.iface_num = op.iface_num;
      this->post_event_(result);
      return;
    }
    case Op::Type::START: {
      result.type = USBAudioEvent::Type::STARTED;
      uac_host_stream_config_t stm_cfg{};
      if (this->populate_stream_config_(op.endpoint, op.handle, stm_cfg, result.rejected)) {
        // The driver only takes volume requests once the stream is started, so a configured capture gain starts it
        // suspended, sets the gain and then lets the audio flow.
        const bool set_gain = op.endpoint == Endpoint::MICROPHONE && this->gain_control_;
        if (set_gain) {
          stm_cfg.flags |= FLAG_STREAM_SUSPEND_AFTER_START;
        }
        esp_err_t err = uac_host_device_start(op.handle, &stm_cfg);
        if (set_gain && err == ESP_OK) {
          this->sync_microphone_gain_(op.handle, false);
          ESP_LOGD(TAG, "Starting USB Audio capture");
          // Already active is ESP_OK.
          err = uac_host_device_resume(op.handle);
        }
        // Already started or active is ESP_OK; ESP_ERR_INVALID_STATE means the interface is not ready.
        result.ok = err == ESP_OK;
        // The host cannot allocate the endpoint's pipe, for example when its max packet size exceeds the USB FIFO.
        result.rejected = err == ESP_ERR_NOT_SUPPORTED;
        if (!result.ok) {
          ESP_LOGW(TAG, "Failed to start %s stream: %s", LOG_STR_ARG(name), esp_err_to_name(err));
        }
      }
      this->post_event_(result);
      return;
    }
    case Op::Type::RESUME: {
      esp_err_t err = uac_host_device_resume(op.handle);
      result.type = USBAudioEvent::Type::RESUMED;
      result.seq = op.seq;
      // Already started or active is ESP_OK; ESP_ERR_INVALID_STATE means the interface is not ready.
      result.ok = err == ESP_OK;
      if (!result.ok) {
        ESP_LOGW(TAG, "Failed to resume %s stream: %s", LOG_STR_ARG(name), esp_err_to_name(err));
      }
      this->post_event_(result);
      return;
    }
    case Op::Type::SUSPEND: {
      esp_err_t err = uac_host_device_suspend(op.handle);
      if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to suspend %s stream: %s", LOG_STR_ARG(name), esp_err_to_name(err));
      }
      return;
    }
    case Op::Type::CLOSE: {
      if (op.stop) {
        esp_err_t err = uac_host_device_stop(op.handle);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
          ESP_LOGW(TAG, "Failed to stop %s stream: %s", LOG_STR_ARG(name), esp_err_to_name(err));
        }
      }
      esp_err_t err = uac_host_device_close(op.handle);
      result.type = USBAudioEvent::Type::CLOSED;
      result.ok = err == ESP_OK;
      if (!result.ok) {
        // The interface may still be held, so the main loop keeps the handle and retries.
        ESP_LOGW(TAG, "uac_host_device_close failed: %s", esp_err_to_name(err));
        this->post_event_(result);
        return;
      }
      if (op.endpoint == Endpoint::SPEAKER) {
        // A reconnected device may support different controls.
        for (ControlState *control : {&this->volume_, &this->mute_}) {
          control->supported = true;
          control->attempts = 0;
        }
      }
      this->post_event_(result);
      return;
    }
    case Op::Type::GAIN: {
      // Cleared first, so a request made during the transfers queues another op.
      this->gain_queued_ = false;
      const uac_host_device_handle_t handle = this->endpoint_(Endpoint::MICROPHONE).handle;
      if (handle != nullptr) {
        this->sync_microphone_gain_(handle, false);
      }
      // Otherwise gain_pending_ stays set and the next open sends it.
      return;
    }
    case Op::Type::CONTROLS: {
      // Cleared first, so a change made during the transfer queues another op.
      this->controls_queued_ = false;
      if (this->apply_pending_speaker_controls_()) {
        // Give the device a moment before retrying; the queue keeps any other work in order.
        this->unlock_driver_();
        vTaskDelay(pdMS_TO_TICKS(CONTROL_RETRY_MS));
        this->lock_driver_();
        this->queue_controls_();
      }
      return;
    }
  }
}

bool USBAudioComponent::populate_stream_config_(Endpoint endpoint, uac_host_device_handle_t handle,
                                                uac_host_stream_config_t &stream_config, bool &rejected) {
  const LogString *name = endpoint_name(endpoint);
  const audio::AudioStreamInfo info = this->endpoint_(endpoint).platform->get_usb_stream_info();
  const uint8_t channels = info.get_channels();
  const uint8_t bits = info.get_bits_per_sample();
  const uint32_t sample_rate = info.get_sample_rate();

  uac_host_dev_info_t dev_info{};
  esp_err_t err = uac_host_get_device_info(handle, &dev_info);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to query %s interface info: %s", LOG_STR_ARG(name), esp_err_to_name(err));
    return false;
  }

  // uac_host_device_start() picks the alternate setting itself from channels, bit depth and rate: the last match with
  // discrete rates, or the first match with a rate range. Find the one it will pick, so its subframe size is checked.
  uint8_t selected_alt = 0;
  uint8_t selected_subframe = 0;
  for (uint8_t alt = 1; alt <= dev_info.iface_alt_num; alt++) {
    uac_host_dev_alt_param_t alt_param{};
    if (uac_host_get_device_alt_param(handle, alt, &alt_param) != ESP_OK || alt_param.channels != channels ||
        alt_param.bit_resolution != bits || !alt_supports_rate(alt_param, sample_rate)) {
      continue;
    }
    selected_alt = alt;
    selected_subframe = alt_param.subframe_size;
    if (alt_param.sample_freq_type == 0) {
      break;
    }
  }

  // ESPHome's samples are packed, so a device that pads them (e.g. 24-bit samples in 4-byte subframes) cannot be
  // streamed unchanged.
  if (selected_alt != 0 && selected_subframe * 8 == bits) {
    stream_config.channels = channels;
    stream_config.bit_resolution = bits;
    stream_config.sample_freq = sample_rate;
    stream_config.flags = 0;
    return true;
  }

  rejected = true;
  if (selected_alt != 0) {
    ESP_LOGW(TAG, "%s interface %u would stream alt %u with %u-bit samples padded to %u bytes, which is not supported",
             LOG_STR_ARG(name), dev_info.iface_num, selected_alt, bits, selected_subframe);
  }
  ESP_LOGW(TAG, "%s interface %u does not support requested format (channels=%u bits=%u sample_rate=%" PRIu32 " Hz)",
           LOG_STR_ARG(name), dev_info.iface_num, channels, bits, sample_rate);
  log_alt_settings(handle, dev_info.iface_alt_num);
  return false;
}

static const LogString *terminal_type_name(uint16_t type) {
  switch (type) {
    case 0x0201:
      return LOG_STR("microphone");
    case 0x0600:
    case 0x0601:
    case 0x0603:
      return LOG_STR("analog connector");
    case 0x0602:
      return LOG_STR("digital audio interface");
    case 0x0605:
      return LOG_STR("S/PDIF");
    default:
      return LOG_STR("other");
  }
}

// The worker logs this once per microphone open, so a device's capture controls are visible before anything is set.
void USBAudioComponent::log_capture_controls_(uac_host_device_handle_t handle) {
  uac_host_dev_info_t info{};
  if (uac_host_get_device_info(handle, &info) != ESP_OK) {
    return;
  }
  // One line per message: the worker's logs pass through the logger's small task buffer, which drops long ones.
  ESP_LOGI(TAG, "USB Audio Capture Device connected");
  ESP_LOGI(TAG, "  VID: 0x%04X, PID: 0x%04X", info.VID, info.PID);
  ESP_LOGI(TAG, "  Address: %u, streaming interface: %u", info.addr, info.iface_num);
  log_alt_settings(handle, info.iface_alt_num);

  UACControlSession session;
  esp_err_t err = session.open(info.addr);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Cannot read the capture controls: %s", esp_err_to_name(err));
    return;
  }
  UACFeatureUnit unit;
  if (!session.find_capture_feature_unit(info.iface_num, unit)) {
    ESP_LOGW(TAG, "  No UAC 1.0 capture path found in the descriptors");
    return;
  }
  ESP_LOGI(TAG, "  UAC version: %X.%02X", unit.bcd_adc >> 8, unit.bcd_adc & 0xFF);
  ESP_LOGI(TAG, "  Audio Control interface: %u", unit.control_iface);
  ESP_LOGI(TAG, "  Capture path: Input Terminal %u (type 0x%04X, %s) -> Feature Unit %u -> streaming terminal %u",
           unit.input_terminal_id, unit.input_terminal_type, LOG_STR_ARG(terminal_type_name(unit.input_terminal_type)),
           unit.unit_id, unit.stream_terminal_id);
  if (unit.unit_id == 0) {
    ESP_LOGI(TAG, "  Capture Feature Unit: none (no hardware gain or mute)");
    return;
  }
  ESP_LOGI(TAG, "  Capture Feature Unit: %u, logical channels: %u", unit.unit_id, unit.channels);
  ESP_LOGI(TAG, "  Volume controls: 0x%04X, mute controls: 0x%04X (bit 0 = master)", unit.volume_channels,
           unit.mute_channels);
  for (uint8_t ch = 0; ch <= unit.channels; ch++) {
    char cur_str[DB_STR_SIZE];
    char min_str[DB_STR_SIZE];
    char max_str[DB_STR_SIZE];
    char res_str[DB_STR_SIZE];
    char label[12];
    if (ch == 0) {
      snprintf(label, sizeof(label), "Master");
    } else {
      snprintf(label, sizeof(label), "Channel %u", ch);
    }
    if (unit.has_volume(ch)) {
      int16_t cur = 0;
      UACVolumeRange range;
      esp_err_t cur_err = session.get_volume(unit, UACControlSession::GET_CUR, ch, cur);
      esp_err_t range_err = session.get_volume_range(unit, ch, range);
      if (cur_err == ESP_OK) {
        ESP_LOGI(TAG, "  %s gain: %s", label, format_db(cur, cur_str));
      } else {
        ESP_LOGW(TAG, "  %s gain: GET_CUR failed: %s", label, esp_err_to_name(cur_err));
      }
      if (range_err == ESP_OK) {
        ESP_LOGI(TAG, "  %s minimum gain: %s, maximum gain: %s, gain resolution: %s", label,
                 format_db(range.min, min_str), format_db(range.max, max_str), format_db(range.res, res_str));
      } else {
        ESP_LOGW(TAG, "  %s gain range: GET_MIN/MAX/RES failed: %s", label, esp_err_to_name(range_err));
      }
    }
    if (unit.has_mute(ch)) {
      bool muted = false;
      esp_err_t mute_err = session.get_mute(unit, ch, muted);
      if (mute_err == ESP_OK) {
        ESP_LOGI(TAG, "  %s mute: %s", label, ONOFF(muted));
      } else {
        ESP_LOGW(TAG, "  %s mute: GET_CUR failed: %s", label, esp_err_to_name(mute_err));
      }
    }
  }
}

// Snap a gain onto the device's grid: min + n * res, within [min, max].
static int16_t snap_gain(int32_t gain, const UACVolumeRange &range) {
  int32_t value = std::clamp<int32_t>(gain, range.min, range.max);
  const int32_t steps = (value - range.min + range.res / 2) / range.res;
  value = range.min + steps * range.res;
  if (value > range.max) {
    value -= range.res;
  }
  return static_cast<int16_t>(value);
}

void USBAudioComponent::sync_microphone_gain_(uac_host_device_handle_t handle, bool verbose) {
  char db_str[DB_STR_SIZE];
  char db_str2[DB_STR_SIZE];
  // Cleared before the transfers: a request made meanwhile sets it again and queues another sync.
  this->gain_pending_ = false;
  const int32_t want = this->gain_request_;
  const char *failure = want != NO_GAIN ? "Hardware capture gain not set" : "Hardware capture gain not read";

  uac_host_dev_info_t info{};
  esp_err_t err = uac_host_get_device_info(handle, &info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s: device info unavailable: %s", failure, esp_err_to_name(err));
    return;
  }
  UACControlSession session;
  err = session.open(info.addr);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s: cannot reach the device's controls: %s", failure, esp_err_to_name(err));
    return;
  }
  UACFeatureUnit unit;
  if (!session.find_capture_feature_unit(info.iface_num, unit) || unit.unit_id == 0 || unit.volume_channels == 0) {
    ESP_LOGE(TAG, "%s: the device has no volume control on its capture path", failure);
    return;
  }

  CaptureGainState state;
  // The channel `state` describes: the first logical channel read, or the master channel.
  bool state_from_channel = false;
  bool state_set = false;
  bool sent = false;
  bool confirmed = true;
  for (uint8_t ch = 0; ch <= unit.channels; ch++) {
    if (!unit.has_volume(ch)) {
      continue;
    }
    const char *label = ch == 0 ? "Master" : "Channel";
    UACVolumeRange range;
    err = session.get_volume_range(unit, ch, range);
    int16_t current = 0;
    if (err == ESP_OK) {
      err = session.get_volume(unit, UACControlSession::GET_CUR, ch, current);
    }
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "%s: %s %u gain cannot be read: %s", failure, label, ch, esp_err_to_name(err));
      confirmed = false;
      continue;
    }
    if (verbose) {
      ESP_LOGI(TAG, "USB Audio capture gain before (%s %u): %s", label, ch, format_db(current, db_str));
    }

    // Like usb_host_uac, every channel with a volume control (the master channel too) takes the same value.
    if (want != NO_GAIN) {
      const int16_t target = snap_gain(want, range);
      if (target != want) {
        char min_str[DB_STR_SIZE];
        char max_str[DB_STR_SIZE];
        ESP_LOGW(TAG, "%s %u: %s is not supported by the device (%s to %s, steps of %s), using %s", label, ch,
                 format_db(static_cast<int16_t>(want), db_str), format_db(range.min, min_str),
                 format_db(range.max, max_str), format_db(range.res, db_str2), format_db(target, db_str));
      }
      ESP_LOGI(TAG, "Setting %s %u capture gain to %s", label, ch, format_db(target, db_str));
      sent = true;
      err = session.set_volume(unit, ch, target);
      if (err == ESP_OK) {
        err = session.get_volume(unit, UACControlSession::GET_CUR, ch, current);
      }
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: %s %u SET_CUR or readback failed: %s", failure, label, ch, esp_err_to_name(err));
        confirmed = false;
        continue;
      }
      ESP_LOGI(TAG, "%s %u gain readback: %s", label, ch, format_db(current, db_str));
      // Within half a step of the target.
      if (std::abs(static_cast<int32_t>(current) - target) * 2 > range.res) {
        ESP_LOGE(TAG, "%s %u reports %s instead of %s", label, ch, format_db(current, db_str),
                 format_db(target, db_str2));
        confirmed = false;
      }
    }
    if (!state_set || (ch > 0 && !state_from_channel)) {
      state = CaptureGainState{current, range.min, range.max, range.res};
      state_set = true;
      state_from_channel = ch > 0;
    }
  }
  if (sent) {
    if (confirmed) {
      ESP_LOGI(TAG, "Hardware capture gain successfully configured");
    } else {
      ESP_LOGE(TAG, "Hardware capture gain could not be confirmed by readback");
    }
  }

  if (!state_set) {
    return;
  }
  {
    LockGuard lock(this->gain_lock_);
    this->gain_state_ = state;
  }
  USBAudioEvent event;
  event.type = USBAudioEvent::Type::GAIN;
  event.endpoint = Endpoint::MICROPHONE;
  this->post_event_(event);
}

void USBAudioComponent::request_microphone_gain(int16_t gain) {
  this->gain_request_ = gain;
  this->gain_pending_ = true;
  this->queue_gain_op_();
}

void USBAudioComponent::queue_gain_op_() {
  if (this->op_queue_ == nullptr || this->gain_queued_.exchange(true)) {
    // Not set up yet, or the queued op will send the latest request.
    return;
  }
  Op op;
  op.type = Op::Type::GAIN;
  op.endpoint = Endpoint::MICROPHONE;
  if (xQueueSend(this->op_queue_, &op, 0) != pdTRUE) {
    // gain_pending_ stays set, so the next connect or request queues it again.
    this->gain_queued_ = false;
  }
}

bool USBAudioComponent::apply_pending_speaker_controls_() {
  const uac_host_device_handle_t handle = this->endpoint_(Endpoint::SPEAKER).handle;
  if (handle == nullptr) {
    // Sent once the stream starts.
    return false;
  }
  bool retry = false;
  if (this->volume_.pending.exchange(false) && this->volume_.supported) {
    retry |= this->apply_control_(this->volume_, uac_host_device_set_volume(handle, this->speaker_volume_),
                                  LOG_STR("volume"));
  }
  if (this->mute_.pending.exchange(false) && this->mute_.supported) {
    retry |= this->apply_control_(this->mute_, uac_host_device_set_mute(handle, this->speaker_muted_), LOG_STR("mute"));
  }
  return retry;
}

bool USBAudioComponent::apply_control_(ControlState &control, esp_err_t err, const LogString *name) {
  if (err == ESP_OK) {
    control.attempts = 0;
    return false;
  }
  if (err == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "Speaker %s control not supported by device", LOG_STR_ARG(name));
    control.supported = false;
    return false;
  }
  if (err == ESP_ERR_INVALID_STATE) {
    // The stream is not started yet; starting it queues the controls again.
    control.pending = true;
    return false;
  }
  if (++control.attempts >= MAX_CONTROL_ATTEMPTS) {
    ESP_LOGW(TAG, "Failed to set speaker %s: %s", LOG_STR_ARG(name), esp_err_to_name(err));
    control.attempts = 0;
    return false;
  }
  control.pending = true;
  return true;
}

void USBAudioComponent::driver_event_stub(uint8_t addr, uint8_t iface_num, const uac_host_driver_event_t event,
                                          void *user_data) {
  auto *self = static_cast<USBAudioComponent *>(user_data);
  USBAudioEvent evt;
  switch (event) {
    case UAC_HOST_DRIVER_EVENT_TX_CONNECTED:
      evt.endpoint = Endpoint::SPEAKER;
      break;
    case UAC_HOST_DRIVER_EVENT_RX_CONNECTED:
      evt.endpoint = Endpoint::MICROPHONE;
      break;
    default:
      return;
  }
  evt.type = USBAudioEvent::Type::CONNECTED;
  evt.addr = addr;
  evt.iface_num = iface_num;
  self->post_event_(evt);
}

void USBAudioComponent::device_event_stub(uac_host_device_handle_t handle, const uac_host_device_event_t event,
                                          void *user_data) {
  auto *self = static_cast<USBAudioComponent *>(user_data);
  if (event == UAC_HOST_DEVICE_EVENT_TX_DONE) {
    auto &speaker = self->endpoint_(Endpoint::SPEAKER);
    // Only the speaker's own interface; the handle is cleared first when the device goes away.
    if (speaker.platform != nullptr && handle == speaker.handle.load()) {
      speaker.platform->on_transfer_taken();
    }
  } else if (event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
    self->transfer_errors_.fetch_add(1, std::memory_order_relaxed);
    self->enable_loop_soon_any_context();
  } else if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
    USBAudioEvent evt;
    evt.type = USBAudioEvent::Type::DISCONNECTED;
    evt.handle = handle;
    self->post_event_(evt);
  }
}

}  // namespace esphome::usb_audio

#endif
