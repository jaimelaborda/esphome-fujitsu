#include "fujitsu_climate.h"

#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace fujitsu {

static const char *const TAG = "fujitsu";

// ---------------------------------------------------------------------------
// Some ESPHome versions expose ClimateTraits::set_supports_current_temperature()
// and some (2026+) inferred it and removed the setter. Call it only if present.
// ---------------------------------------------------------------------------
template<typename T>
static auto enable_current_temperature(T &traits, int)
    -> decltype(traits.set_supports_current_temperature(true), void()) {
  traits.set_supports_current_temperature(true);
}
template<typename T> static void enable_current_temperature(T &, long) {}

// ---------------------------------------------------------------------------
// Enum mapping helpers.
// ---------------------------------------------------------------------------
static faircon::Mode to_fujitsu_mode(climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_COOL: return faircon::Mode::Cool;
    case climate::CLIMATE_MODE_DRY: return faircon::Mode::Dry;
    case climate::CLIMATE_MODE_FAN_ONLY: return faircon::Mode::Fan;
    case climate::CLIMATE_MODE_HEAT: return faircon::Mode::Heat;
    case climate::CLIMATE_MODE_HEAT_COOL:
    case climate::CLIMATE_MODE_AUTO:
    default:
      return faircon::Mode::Auto;
  }
}

static climate::ClimateMode from_fujitsu_mode(uint16_t value) {
  switch (static_cast<faircon::Mode>(value)) {
    case faircon::Mode::Cool: return climate::CLIMATE_MODE_COOL;
    case faircon::Mode::Dry: return climate::CLIMATE_MODE_DRY;
    case faircon::Mode::Fan: return climate::CLIMATE_MODE_FAN_ONLY;
    case faircon::Mode::Heat: return climate::CLIMATE_MODE_HEAT;
    case faircon::Mode::Auto:
    default:
      return climate::CLIMATE_MODE_HEAT_COOL;
  }
}

static faircon::FanSpeed to_fujitsu_fan(climate::ClimateFanMode fan) {
  switch (fan) {
    case climate::CLIMATE_FAN_QUIET: return faircon::FanSpeed::Quiet;
    case climate::CLIMATE_FAN_LOW: return faircon::FanSpeed::Low;
    case climate::CLIMATE_FAN_MEDIUM: return faircon::FanSpeed::Medium;
    case climate::CLIMATE_FAN_HIGH: return faircon::FanSpeed::High;
    case climate::CLIMATE_FAN_AUTO:
    default:
      return faircon::FanSpeed::Auto;
  }
}

static climate::ClimateFanMode from_fujitsu_fan(uint16_t value) {
  switch (static_cast<faircon::FanSpeed>(value)) {
    case faircon::FanSpeed::Quiet: return climate::CLIMATE_FAN_QUIET;
    case faircon::FanSpeed::Low: return climate::CLIMATE_FAN_LOW;
    case faircon::FanSpeed::Medium: return climate::CLIMATE_FAN_MEDIUM;
    case faircon::FanSpeed::High: return climate::CLIMATE_FAN_HIGH;
    case faircon::FanSpeed::Auto:
    default:
      return climate::CLIMATE_FAN_AUTO;
  }
}

// ---------------------------------------------------------------------------
// Component lifecycle.
// ---------------------------------------------------------------------------
void FujitsuClimate::setup() {
  this->protocol_.set_log_raw_frames(this->log_raw_frames_);
  this->protocol_.set_log_cb([this](const char *level, const char *msg) { this->on_log(level, msg); });
  this->protocol_.set_status_cb([this](const char *status) { this->on_status(status); });
  this->protocol_.set_register_change_cb(
      [this](uint16_t a, uint16_t o, uint16_t n) { this->on_register_change(a, o, n); });
  this->protocol_.setup();

  // Sensible defaults until the first poll populates real values.
  this->mode = climate::CLIMATE_MODE_OFF;
  this->publish_state();
}

void FujitsuClimate::loop() {
  this->protocol_.loop();

  if (this->climate_state_dirty_) {
    this->climate_state_dirty_ = false;
    this->refresh_state_from_registers();
  }
}

void FujitsuClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Fujitsu Climate (UTY-TFSXW1):");
  ESP_LOGCONFIG(TAG, "  Horizontal swing exposed: %s", YESNO(this->horizontal_swing_supported_));
  ESP_LOGCONFIG(TAG, "  Presets (eco/boost) enabled: %s", YESNO(this->presets_enabled_));
  ESP_LOGCONFIG(TAG, "  Raw frame logging: %s", YESNO(this->log_raw_frames_));
  if (this->outdoor_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Outdoor temperature", this->outdoor_sensor_);
  }
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_NONE, 8);
}

// ---------------------------------------------------------------------------
// Traits.
// ---------------------------------------------------------------------------
climate::ClimateTraits FujitsuClimate::traits() {
  climate::ClimateTraits traits;
  enable_current_temperature(traits, 0);

  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);  // Fujitsu "Auto"
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_DRY);
  traits.add_supported_mode(climate::CLIMATE_MODE_FAN_ONLY);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);

  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_QUIET);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);

  traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_VERTICAL);
  if (this->horizontal_swing_supported_) {
    traits.add_supported_swing_mode(climate::CLIMATE_SWING_HORIZONTAL);
    traits.add_supported_swing_mode(climate::CLIMATE_SWING_BOTH);
  }

  if (this->presets_enabled_) {
    traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
    traits.add_supported_preset(climate::CLIMATE_PRESET_ECO);
    traits.add_supported_preset(climate::CLIMATE_PRESET_BOOST);
  }

  // Heat allows down to 16 C, cool/auto down to 18 C; the firmware clamps per mode.
  traits.set_visual_min_temperature(16.0f);
  traits.set_visual_max_temperature(30.0f);
  traits.set_visual_target_temperature_step(0.5f);
  traits.set_visual_current_temperature_step(0.1f);

  return traits;
}

// ---------------------------------------------------------------------------
// Control (Home Assistant -> AC).
// ---------------------------------------------------------------------------
void FujitsuClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    climate::ClimateMode mode = *call.get_mode();
    if (mode == climate::CLIMATE_MODE_OFF) {
      this->protocol_.set_power(faircon::Power::Off);
    } else {
      // Ensure the unit is on, then apply the mode.
      if (!this->protocol_.is_powered_on()) {
        this->protocol_.set_power(faircon::Power::On);
      }
      this->protocol_.set_mode(to_fujitsu_mode(mode));
    }
  }

  if (call.get_target_temperature().has_value()) {
    this->protocol_.set_setpoint_celsius(*call.get_target_temperature());
  }

  if (call.get_fan_mode().has_value()) {
    this->protocol_.set_fan_speed(to_fujitsu_fan(*call.get_fan_mode()));
  }

  if (call.get_swing_mode().has_value()) {
    switch (*call.get_swing_mode()) {
      case climate::CLIMATE_SWING_BOTH:
        this->protocol_.set_vertical_swing(true);
        this->protocol_.set_horizontal_swing(true);
        break;
      case climate::CLIMATE_SWING_VERTICAL:
        this->protocol_.set_vertical_swing(true);
        this->protocol_.set_horizontal_swing(false);
        break;
      case climate::CLIMATE_SWING_HORIZONTAL:
        this->protocol_.set_vertical_swing(false);
        this->protocol_.set_horizontal_swing(true);
        break;
      case climate::CLIMATE_SWING_OFF:
      default:
        this->protocol_.set_vertical_swing(false);
        this->protocol_.set_horizontal_swing(false);
        break;
    }
  }

  if (this->presets_enabled_ && call.get_preset().has_value()) {
    switch (*call.get_preset()) {
      case climate::CLIMATE_PRESET_BOOST:
        this->protocol_.set_powerful(true);
        this->protocol_.set_economy(false);
        break;
      case climate::CLIMATE_PRESET_ECO:
        this->protocol_.set_economy(true);
        this->protocol_.set_powerful(false);
        break;
      case climate::CLIMATE_PRESET_NONE:
      default:
        this->protocol_.set_powerful(false);
        this->protocol_.set_economy(false);
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// State (AC -> Home Assistant).
// ---------------------------------------------------------------------------
void FujitsuClimate::on_register_change(uint16_t address, uint16_t /*old_value*/, uint16_t new_value) {
  switch (address) {
    case faircon::reg::Power:
    case faircon::reg::Mode:
    case faircon::reg::SetpointTemp:
    case faircon::reg::FanSpeed:
    case faircon::reg::VerticalSwing:
    case faircon::reg::HorizontalSwing:
    case faircon::reg::ActualTemp:
    case faircon::reg::Powerful:
    case faircon::reg::EconomyMode:
      this->climate_state_dirty_ = true;
      break;
    case faircon::reg::OutdoorTemp:
      if (this->outdoor_sensor_ != nullptr) {
        this->outdoor_sensor_->publish_state(faircon::FujitsuProtocol::raw_to_celsius(new_value));
      }
      break;
    default:
      break;
  }
}

void FujitsuClimate::refresh_state_from_registers() {
  // Operating mode.
  if (!this->protocol_.is_powered_on()) {
    this->mode = climate::CLIMATE_MODE_OFF;
  } else {
    this->mode = from_fujitsu_mode(this->protocol_.get_register(faircon::reg::Mode));
  }

  // Target temperature (0xFFFF is reported in fan mode -> leave unchanged).
  uint16_t setpoint = this->protocol_.get_register(faircon::reg::SetpointTemp);
  if (setpoint != 0x0000 && setpoint != 0xFFFF) {
    this->target_temperature = faircon::FujitsuProtocol::setpoint_raw_to_celsius(setpoint);
  }

  // Current temperature.
  uint16_t actual = this->protocol_.get_register(faircon::reg::ActualTemp);
  if (actual != 0x0000 && actual != 0xFFFF) {
    this->current_temperature = faircon::FujitsuProtocol::raw_to_celsius(actual);
  } else {
    this->current_temperature = NAN;
  }

  // Fan speed.
  this->fan_mode = from_fujitsu_fan(this->protocol_.get_register(faircon::reg::FanSpeed));

  // Swing.
  bool v = this->protocol_.get_register(faircon::reg::VerticalSwing) == faircon::ON;
  bool h = this->protocol_.get_register(faircon::reg::HorizontalSwing) == faircon::ON;
  if (v && h) {
    this->swing_mode = climate::CLIMATE_SWING_BOTH;
  } else if (v) {
    this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
  } else if (h) {
    this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  } else {
    this->swing_mode = climate::CLIMATE_SWING_OFF;
  }

  // Presets.
  if (this->presets_enabled_) {
    if (this->protocol_.get_register(faircon::reg::Powerful) == faircon::ON) {
      this->preset = climate::CLIMATE_PRESET_BOOST;
    } else if (this->protocol_.get_register(faircon::reg::EconomyMode) == faircon::ON) {
      this->preset = climate::CLIMATE_PRESET_ECO;
    } else {
      this->preset = climate::CLIMATE_PRESET_NONE;
    }
  }

  this->publish_state();
}

void FujitsuClimate::on_status(const char *status) { ESP_LOGI(TAG, "Status: %s", status); }

void FujitsuClimate::on_log(const char *level, const char *message) {
  // Map the protocol core's level strings to ESPHome log levels.
  switch (level[0]) {
    case 'e': ESP_LOGE(TAG, "%s", message); break;  // error
    case 'w': ESP_LOGW(TAG, "%s", message); break;  // warn
    case 'i': ESP_LOGI(TAG, "%s", message); break;  // info
    default: ESP_LOGD(TAG, "%s", message); break;   // debug
  }
}

}  // namespace fujitsu
}  // namespace esphome
