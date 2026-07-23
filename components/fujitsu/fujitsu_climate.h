/*
  esphome-fujitsu - ESPHome climate component for Fujitsu air conditioners using
  the UTY-TFSXW1 serial protocol (9600 8N1, inverted). Wraps the platform-agnostic
  protocol core (fujitsu_protocol.h) and maps its registers onto an ESPHome
  climate entity plus an optional outdoor-temperature sensor.
*/

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

#include "fujitsu_protocol.h"

namespace esphome {
namespace fujitsu {

class FujitsuClimate : public climate::Climate, public uart::UARTDevice, public Component {
 public:
  FujitsuClimate() = default;

  // Component
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Climate
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // Configuration (set from Python codegen)
  void set_horizontal_swing_supported(bool value) { horizontal_swing_supported_ = value; }
  void set_presets_enabled(bool value) { presets_enabled_ = value; }
  void set_log_raw_frames(bool value) { log_raw_frames_ = value; }
  void set_outdoor_temperature_sensor(sensor::Sensor *sensor) { outdoor_sensor_ = sensor; }

  // True once the handshake with the AC is complete and it is actively polling.
  // Handy for driving a status LED (see example.yaml).
  bool is_ac_connected() const { return protocol_.is_running(); }

 protected:
  // Adapts the ESPHome UART to the protocol core's ByteIO interface.
  class UartIO : public faircon::ByteIO {
   public:
    explicit UartIO(uart::UARTDevice *device) : device_(device) {}
    int available() override { return this->device_->available(); }
    int read() override {
      uint8_t b;
      return this->device_->read_byte(&b) ? static_cast<int>(b) : -1;
    }
    void write(const uint8_t *data, size_t len) override { this->device_->write_array(data, len); }

   private:
    uart::UARTDevice *device_;
  };

  void on_register_change(uint16_t address, uint16_t old_value, uint16_t new_value);
  void on_status(const char *status);
  void on_log(const char *level, const char *message);
  void refresh_state_from_registers();

  UartIO io_{this};
  faircon::FujitsuProtocol protocol_{io_, []() { return millis(); }};

  sensor::Sensor *outdoor_sensor_{nullptr};
  bool horizontal_swing_supported_{false};
  bool presets_enabled_{true};
  bool log_raw_frames_{false};

  bool climate_state_dirty_{false};
};

}  // namespace fujitsu
}  // namespace esphome
