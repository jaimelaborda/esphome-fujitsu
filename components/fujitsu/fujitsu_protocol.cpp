/*
  esphome-fujitsu - Fujitsu air-conditioner controller (UTY-TFSXW1 protocol).
  Platform-agnostic protocol core. See fujitsu_protocol.h for the design notes.
*/

#include "fujitsu_protocol.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace faircon {

// ---------------------------------------------------------------------------
// Static register groups. Concatenated they form the full 70-register table
// (every address is unique), which mirrors upstream's initRegistryTable().
// ---------------------------------------------------------------------------
static const uint16_t kInitial1[] = {reg::Initial0, reg::Initial1};

static const uint16_t kInitial2[] = {
    reg::Initial2, reg::Initial3, reg::Initial4, reg::Initial5, reg::Initial6,
    reg::Initial7, reg::Initial8, reg::Initial9, reg::Initial10, reg::Initial11,
    reg::VerticalAirflowDirectionCount, reg::VerticalSwingSupported,
    reg::HorizontalAirflowDirectionCount, reg::HorizontalSwingSupported};

static const uint16_t kInitial3[] = {
    reg::EconomyModeSupported, reg::MinimumHeatSupported, reg::HumanSensorSupported,
    reg::EnergySavingFanSupported, reg::Initial20, reg::Initial21, reg::Initial22,
    reg::PowerfulSupported, reg::OutdoorUnitLowNoiseSupported, reg::CoilDrySupported};

static const uint16_t kFrameA[] = {
    reg::Power, reg::Mode, reg::SetpointTemp, reg::FanSpeed,
    reg::VerticalAirflowSetterRegistry, reg::VerticalSwing, reg::VerticalAirflow,
    reg::HorizontalAirflowSetterRegistry, reg::HorizontalSwing, reg::HorizontalAirflow,
    reg::Register11, reg::ActualTemp, reg::Register13};

static const uint16_t kFrameB[] = {
    reg::EconomyMode, reg::MinimumHeat, reg::HumanSensor, reg::Register17,
    reg::Register18, reg::Register19, reg::Register20, reg::Register21,
    reg::EnergySavingFan, reg::Register23, reg::Powerful, reg::OutdoorUnitLowNoise,
    reg::CoilDry, reg::Register27, reg::Register28, reg::Register29, reg::Register30,
    reg::Register31, reg::Register32};

static const uint16_t kFrameC[] = {
    reg::Register33, reg::Register34, reg::Register35, reg::Register36,
    reg::Register37, reg::Register38, reg::Register39, reg::Register40,
    reg::Register41, reg::OutdoorTemp, reg::Register43, reg::Register44};

template<size_t N>
static constexpr size_t count_of(const uint16_t (&)[N]) {
  return N;
}

static std::string to_hex(const uint8_t *buffer, int size) {
  std::string out;
  out.reserve(static_cast<size_t>(size) * 3);
  char tmp[4];
  for (int i = 0; i < size; i++) {
    std::snprintf(tmp, sizeof(tmp), i ? " %02X" : "%02X", buffer[i]);
    out += tmp;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Framing helpers.
// ---------------------------------------------------------------------------
uint16_t frame_checksum(const uint8_t *buffer, int length_without_checksum) {
  uint16_t checksum = 0xFFFF;
  for (int i = 0; i < length_without_checksum; i++) {
    checksum -= buffer[i];
  }
  return checksum;
}

bool frame_is_valid(const uint8_t *buffer, int size) {
  if (size < 3) {
    return false;
  }
  uint16_t frame_cs = static_cast<uint16_t>((buffer[size - 2] << 8) | buffer[size - 1]);
  return frame_cs == frame_checksum(buffer, size - 2);
}

static void append_checksum(std::vector<uint8_t> &frame) {
  uint16_t cs = frame_checksum(frame.data(), static_cast<int>(frame.size()));
  frame.push_back(static_cast<uint8_t>((cs >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(cs & 0xFF));
}

std::vector<uint8_t> build_init1() {
  std::vector<uint8_t> f = {0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
  append_checksum(f);
  return f;
}

std::vector<uint8_t> build_init2() {
  std::vector<uint8_t> f = {0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x01};
  append_checksum(f);
  return f;
}

std::vector<uint8_t> build_read_request(const uint16_t *addresses, size_t count) {
  size_t payload = 2 * count;
  std::vector<uint8_t> f;
  f.reserve(payload + 7);
  f.push_back(0x03);
  f.push_back(0x00);
  f.push_back(0x00);
  f.push_back(0x00);
  f.push_back(static_cast<uint8_t>(payload));
  for (size_t i = 0; i < count; i++) {
    f.push_back(static_cast<uint8_t>((addresses[i] >> 8) & 0xFF));
    f.push_back(static_cast<uint8_t>(addresses[i] & 0xFF));
  }
  append_checksum(f);
  return f;
}

std::vector<uint8_t> build_write_request(const std::pair<uint16_t, uint16_t> *regs, size_t count) {
  size_t payload = 4 * count;
  std::vector<uint8_t> f;
  f.reserve(payload + 7);
  f.push_back(0x02);
  f.push_back(0x00);
  f.push_back(0x00);
  f.push_back(0x00);
  f.push_back(static_cast<uint8_t>(payload));
  for (size_t i = 0; i < count; i++) {
    f.push_back(static_cast<uint8_t>((regs[i].first >> 8) & 0xFF));
    f.push_back(static_cast<uint8_t>(regs[i].first & 0xFF));
    f.push_back(static_cast<uint8_t>((regs[i].second >> 8) & 0xFF));
    f.push_back(static_cast<uint8_t>(regs[i].second & 0xFF));
  }
  append_checksum(f);
  return f;
}

// ---------------------------------------------------------------------------
// FrameParser.
// ---------------------------------------------------------------------------
void FrameParser::feed_byte(uint8_t byte, uint32_t now_ms, const Callback &callback) {
  if ((now_ms - last_ms_) >= 20) {
    index_ = 0;  // inter-frame gap -> start a new frame
  }
  last_ms_ = now_ms;

  if (index_ >= kMaxFrame) {
    index_ = 0;  // overflow safety (upstream could overrun its 128-byte buffer)
  }
  buffer_[index_++] = byte;

  if (index_ > 4 && index_ == static_cast<int>(buffer_[4]) + 7) {
    int size = static_cast<int>(buffer_[4]) + 7;
    bool valid = frame_is_valid(buffer_, size);
    if (callback) {
      callback(buffer_, size, valid);
    }
    index_ = 0;  // ready for the next frame
  }
}

void FrameParser::poll(ByteIO &io, uint32_t now_ms, const Callback &callback) {
  while (io.available() > 0) {
    int b = io.read();
    if (b < 0) {
      break;
    }
    feed_byte(static_cast<uint8_t>(b), now_ms, callback);
  }
}

// ---------------------------------------------------------------------------
// FujitsuProtocol.
// ---------------------------------------------------------------------------
FujitsuProtocol::FujitsuProtocol(ByteIO &io, MillisFn millis) : io_(io), millis_(std::move(millis)) {}

void FujitsuProtocol::setup() {
  registers_.clear();
  auto seed = [this](const uint16_t *addrs, size_t n) {
    for (size_t i = 0; i < n; i++) {
      registers_[addrs[i]] = 0x0000;
    }
  };
  seed(kInitial1, count_of(kInitial1));
  seed(kInitial2, count_of(kInitial2));
  seed(kInitial3, count_of(kInitial3));
  seed(kFrameA, count_of(kFrameA));
  seed(kFrameB, count_of(kFrameB));
  seed(kFrameC, count_of(kFrameC));

  initialized_ = true;
  terminated_ = false;
  running_ = false;
  last_frame_sent_ = FrameType::None;
  last_response_received_ = true;
  no_response_notified_ = false;
  last_request_ms_ = millis_();
}

void FujitsuProtocol::loop() {
  if (!initialized_) {
    return;
  }

  uint32_t now = millis_();

  if (terminated_) {
    // Self-healing: retry the whole handshake a few seconds after a fatal error,
    // so a transient glitch doesn't require a physical reboot.
    if ((now - terminated_ms_) >= 10000) {
      log("info", "Re-initializing after termination");
      terminated_ = false;
      running_ = false;
      last_frame_sent_ = FrameType::None;
      last_response_received_ = true;
      no_response_notified_ = false;
      last_request_ms_ = now;
    } else {
      return;
    }
  }

  send_request();

  parser_.poll(io_, now, [this](const uint8_t *buffer, int size, bool valid) {
    this->on_frame(buffer, size, valid);
  });
}

void FujitsuProtocol::send_request() {
  if (terminated_) {
    return;
  }

  uint32_t now = millis_();

  if (!last_response_received_ && (now - last_request_ms_) >= 200) {
    if (last_frame_sent_ == FrameType::None || last_frame_sent_ == FrameType::Init1) {
      // Communication not established yet; the handshake is simply retried.
      last_frame_sent_ = FrameType::None;
      last_response_received_ = true;
    } else if (!no_response_notified_) {
      no_response_notified_ = true;
      log("error", "No response for 200 ms");
      set_status("No response for 200 ms");
    }
    return;
  }

  if (!(last_response_received_ && (now - last_request_ms_) >= 400)) {
    return;
  }

  last_request_ms_ = now;

  switch (last_frame_sent_) {
    case FrameType::None: {
      auto f = build_init1();
      io_.write(f.data(), f.size());
      last_frame_sent_ = FrameType::Init1;
      last_response_received_ = false;
      set_status("Init1 Send");
      if (log_raw_frames_) {
        log("debug", ("send " + to_hex(f.data(), static_cast<int>(f.size()))).c_str());
      }
      break;
    }

    case FrameType::Init1: {
      auto f = build_init2();
      io_.write(f.data(), f.size());
      last_frame_sent_ = FrameType::Init2;
      last_response_received_ = false;
      set_status("Init2 Send");
      if (log_raw_frames_) {
        log("debug", ("send " + to_hex(f.data(), static_cast<int>(f.size()))).c_str());
      }
      break;
    }

    case FrameType::Init2:
      request_registers(FrameType::InitialRegistries1, kInitial1, count_of(kInitial1));
      break;

    case FrameType::InitialRegistries1:
      request_registers(FrameType::InitialRegistries2, kInitial2, count_of(kInitial2));
      break;

    case FrameType::InitialRegistries2:
      request_registers(FrameType::InitialRegistries3, kInitial3, count_of(kInitial3));
      break;

    case FrameType::InitialRegistries3:
    case FrameType::FrameC:
      request_registers(FrameType::FrameA, kFrameA, count_of(kFrameA));
      break;

    case FrameType::FrameA:
      if (!write_queue_.empty()) {
        active_write_ = write_queue_.front();
        write_queue_.pop_front();
        send_write(active_write_);
      } else {
        request_registers(FrameType::FrameB, kFrameB, count_of(kFrameB));
      }
      break;

    case FrameType::SendRegistries: {
      // Re-read the registers we just wrote so the reported state reflects reality.
      std::vector<uint16_t> addrs;
      addrs.reserve(active_write_.regs.size());
      for (auto &p : active_write_.regs) {
        addrs.push_back(p.first);
      }
      request_registers(FrameType::CheckRegistries, addrs.data(), addrs.size());
      break;
    }

    case FrameType::CheckRegistries:
      request_registers(FrameType::FrameB, kFrameB, count_of(kFrameB));
      break;

    case FrameType::FrameB:
      request_registers(FrameType::FrameC, kFrameC, count_of(kFrameC));
      break;
  }
}

void FujitsuProtocol::request_registers(FrameType type, const uint16_t *addresses, size_t count) {
  last_frame_sent_ = type;
  last_response_received_ = false;
  auto f = build_read_request(addresses, count);
  if (log_raw_frames_) {
    log("debug", ("read " + to_hex(f.data(), static_cast<int>(f.size()))).c_str());
  }
  io_.write(f.data(), f.size());
}

void FujitsuProtocol::send_write(const WriteCommand &cmd) {
  last_frame_sent_ = FrameType::SendRegistries;
  last_response_received_ = false;
  auto f = build_write_request(cmd.regs.data(), cmd.regs.size());
  log("info", ("write " + to_hex(f.data(), static_cast<int>(f.size()))).c_str());
  io_.write(f.data(), f.size());
}

void FujitsuProtocol::on_frame(const uint8_t *buffer, int size, bool valid) {
  if (!initialized_) {
    return;
  }

  if (!valid) {
    log("error", ("invalid checksum: " + to_hex(buffer, size)).c_str());
    return;
  }

  if (terminated_) {
    log("error", ("frame after termination: " + to_hex(buffer, size)).c_str());
    return;
  }

  if (no_response_notified_) {
    no_response_notified_ = false;
    set_status("Running");
  }

  if (last_frame_sent_ == FrameType::Init1) {
    // Two 8-byte frames can appear right after the unit restarts; ignore them and
    // keep waiting for the real handshake acknowledgement.
    static const uint8_t restart_frames[][8] = {
        {0xFE, 0x00, 0x00, 0x00, 0x01, 0x02, 0xFE, 0xFE},
        {0xFC, 0x00, 0x00, 0x00, 0x01, 0x02, 0xFF, 0x00},
    };
    for (const auto &rf : restart_frames) {
      if (size == 8 && std::memcmp(buffer, rf, 8) == 0) {
        if (log_raw_frames_) {
          log("debug", ("restart frame: " + to_hex(buffer, size)).c_str());
        }
        return;
      }
    }

    // Expected ack: 00 00 00 00 01 01 FF FD. Accept on type + ok-status; middle
    // bytes are tolerated to be robust against small firmware variations.
    if (size == 8 && buffer[0] == 0x00 && buffer[5] == 0x01) {
      last_response_received_ = true;
    } else {
      log("error", ("unexpected Init1 response, terminating: " + to_hex(buffer, size)).c_str());
      set_status("Terminated Init1");
      terminated_ = true;
      terminated_ms_ = millis_();
    }
    return;
  }

  if (last_frame_sent_ == FrameType::Init2) {
    // Expected ack: 01 00 00 00 01 01 FF FC.
    if (size == 8 && buffer[0] == 0x01 && buffer[5] == 0x01) {
      last_response_received_ = true;
      running_ = true;
      set_status("Running");
    } else {
      log("error", ("unexpected Init2 response, terminating: " + to_hex(buffer, size)).c_str());
      set_status("Terminated Init2");
      terminated_ = true;
      terminated_ms_ = millis_();
    }
    return;
  }

  if (buffer[0] == 0x03) {  // response to a read request
    if (buffer[5] != 0x01) {
      log("error", ("read response bad status: " + to_hex(buffer, size)).c_str());
    }
    last_response_received_ = true;
    if (buffer[5] == 0x01) {
      update_registers(buffer, size);
    }
    return;
  }

  if (buffer[0] == 0x02) {  // response to a write request
    if (log_raw_frames_) {
      log("debug", ("write ack: " + to_hex(buffer, size)).c_str());
    }
    if (buffer[5] != 0x01) {
      log("error", ("write response bad status: " + to_hex(buffer, size)).c_str());
    }
    last_response_received_ = true;
    return;
  }
}

void FujitsuProtocol::update_registers(const uint8_t *buffer, int size) {
  int count = buffer[4] / 4;

  for (int i = 0; i < count; i++) {
    int index = 6 + i * 4;
    if (index + 4 > size - 2) {
      break;  // would read into/past the checksum; malformed
    }

    uint16_t address = static_cast<uint16_t>((buffer[index] << 8) | buffer[index + 1]);
    uint16_t value = static_cast<uint16_t>((buffer[index + 2] << 8) | buffer[index + 3]);

    auto it = registers_.find(address);
    if (it == registers_.end()) {
      // Unknown address (should not happen since we only request known ones).
      char msg[48];
      std::snprintf(msg, sizeof(msg), "unknown register %04X = %04X", address, value);
      log("warn", msg);
      continue;
    }

    if (it->second != value) {
      uint16_t old_value = it->second;
      it->second = value;

      if (log_raw_frames_) {
        char msg[40];
        std::snprintf(msg, sizeof(msg), "%04X | %04X -> %04X", address, old_value, value);
        log("debug", msg);
      }
      if (register_change_cb_) {
        register_change_cb_(address, old_value, value);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// State queries.
// ---------------------------------------------------------------------------
bool FujitsuProtocol::has_register(uint16_t address) const {
  return registers_.find(address) != registers_.end();
}

uint16_t FujitsuProtocol::get_register(uint16_t address) const {
  auto it = registers_.find(address);
  return it == registers_.end() ? 0x0000 : it->second;
}

bool FujitsuProtocol::is_powered_on() const { return get_register(reg::Power) == static_cast<uint16_t>(Power::On); }

bool FujitsuProtocol::is_feature_supported(uint16_t feature_address) const {
  uint16_t value = get_register(feature_address);
  if (feature_address == reg::VerticalAirflowDirectionCount ||
      feature_address == reg::HorizontalAirflowDirectionCount) {
    return value > 0x0000;
  }
  return value == 0x0001;
}

// ---------------------------------------------------------------------------
// Command helpers.
// ---------------------------------------------------------------------------
void FujitsuProtocol::enqueue(WriteCommand cmd) {
  if (cmd.regs.empty()) {
    return;
  }
  constexpr size_t kMaxQueue = 32;
  if (write_queue_.size() >= kMaxQueue) {
    write_queue_.pop_front();
    log("warn", "write queue full, dropping oldest command");
  }
  write_queue_.push_back(std::move(cmd));
}

bool FujitsuProtocol::guard_blocks_change() {
  if (get_register(reg::CoilDry) == ON) {
    log("info", "Coil dry is on, ignoring change");
    return true;
  }
  if (get_register(reg::MinimumHeat) == ON) {
    log("info", "Minimum heat is on, ignoring change");
    return true;
  }
  return false;
}

void FujitsuProtocol::set_power(Power power) {
  enqueue({{{reg::Power, static_cast<uint16_t>(power)}}});
}

void FujitsuProtocol::set_mode(Mode mode) {
  if (guard_blocks_change()) {
    return;
  }
  enqueue({{{reg::Mode, static_cast<uint16_t>(mode)}}});
}

void FujitsuProtocol::set_fan_speed(FanSpeed speed) {
  if (guard_blocks_change()) {
    return;
  }
  enqueue({{{reg::FanSpeed, static_cast<uint16_t>(speed)}}});
}

void FujitsuProtocol::set_setpoint_celsius(float celsius) {
  if (guard_blocks_change()) {
    return;
  }
  if (get_register(reg::Mode) == static_cast<uint16_t>(Mode::Fan)) {
    log("info", "Fan mode enabled, setpoint ignored");
    return;
  }
  bool heat_mode = get_register(reg::Mode) == static_cast<uint16_t>(Mode::Heat);
  int tenths = round_setpoint_tenths(celsius, heat_mode);
  enqueue({{{reg::SetpointTemp, static_cast<uint16_t>(tenths)}}});
}

void FujitsuProtocol::set_vertical_swing(bool on) {
  if (get_register(reg::CoilDry) == ON) {
    log("info", "Coil dry is on, ignoring change");
    return;
  }
  if (!is_feature_supported(reg::VerticalSwingSupported)) {
    log("warn", "Vertical swing not supported");
    return;
  }
  enqueue({{{reg::VerticalSwing, on ? ON : OFF}}});
}

void FujitsuProtocol::set_horizontal_swing(bool on) {
  if (get_register(reg::CoilDry) == ON) {
    log("info", "Coil dry is on, ignoring change");
    return;
  }
  if (!is_feature_supported(reg::HorizontalSwingSupported)) {
    log("warn", "Horizontal swing not supported");
    return;
  }
  enqueue({{{reg::HorizontalSwing, on ? ON : OFF}}});
}

void FujitsuProtocol::set_vertical_airflow(Airflow position) {
  if (get_register(reg::CoilDry) == ON) {
    log("info", "Coil dry is on, ignoring change");
    return;
  }
  if (!is_feature_supported(reg::VerticalAirflowDirectionCount)) {
    log("warn", "Vertical airflow not supported");
    return;
  }
  enqueue({{{reg::VerticalSwing, OFF},
           {reg::VerticalAirflowSetterRegistry, static_cast<uint16_t>(position)}}});
}

void FujitsuProtocol::set_horizontal_airflow(Airflow position) {
  if (get_register(reg::CoilDry) == ON) {
    log("info", "Coil dry is on, ignoring change");
    return;
  }
  if (!is_feature_supported(reg::HorizontalAirflowDirectionCount)) {
    log("warn", "Horizontal airflow not supported");
    return;
  }
  enqueue({{{reg::HorizontalSwing, OFF},
           {reg::HorizontalAirflowSetterRegistry, static_cast<uint16_t>(position)}}});
}

void FujitsuProtocol::set_powerful(bool on) {
  if (guard_blocks_change()) {
    return;
  }
  enqueue({{{reg::Powerful, on ? ON : OFF}}});
}

void FujitsuProtocol::set_economy(bool on) {
  if (guard_blocks_change()) {
    return;
  }
  enqueue({{{reg::EconomyMode, on ? ON : OFF}}});
}

// ---------------------------------------------------------------------------
// Pure conversion helpers.
// ---------------------------------------------------------------------------
int FujitsuProtocol::round_setpoint_tenths(double celsius, bool heat_mode) {
  int result = static_cast<int>(celsius * 10 + 0.5);
  result = (result + 2) / 5 * 5;  // round to nearest 0.5 C
  int min_temp = heat_mode ? SETPOINT_MIN_HEAT : SETPOINT_MIN_COOL;
  if (result < min_temp) {
    result = min_temp;
  } else if (result > SETPOINT_MAX) {
    result = SETPOINT_MAX;
  }
  return result;
}

// ---------------------------------------------------------------------------
// Logging / status plumbing.
// ---------------------------------------------------------------------------
void FujitsuProtocol::set_status(const char *status) {
  if (status_cb_) {
    status_cb_(status);
  }
}

void FujitsuProtocol::log(const char *level, const char *msg) {
  if (log_cb_) {
    log_cb_(level, msg);
  }
}

}  // namespace faircon
