/*
  esphome-fujitsu - Fujitsu air-conditioner controller (UTY-TFSXW1 protocol) for ESPHome.

  This "protocol core" is deliberately platform-agnostic: it depends only on the C++
  standard library (no Arduino.h, no ESPHome headers). That lets the exact same code
  run on the ESP8266/ESP32 under ESPHome *and* be compiled and unit-tested on a PC.

  Protocol reverse-engineered by Benas Ragauskas (https://github.com/Benas09/FujitsuAC).
  This is an independent ESPHome port of that protocol.
*/

#pragma once

#include <cstdint>
#include <cstddef>
#include <deque>
#include <functional>
#include <unordered_map>
#include <vector>

namespace faircon {

// ---------------------------------------------------------------------------
// Byte transport abstraction.
// On device this is backed by the ESPHome UART; in tests it is backed by an
// in-memory queue. read() returns -1 when no byte is available.
// ---------------------------------------------------------------------------
class ByteIO {
 public:
  virtual ~ByteIO() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual void write(const uint8_t *data, size_t len) = 0;
};

using MillisFn = std::function<uint32_t()>;
using LogCb = std::function<void(const char *level, const char *msg)>;
using RegisterChangeCb = std::function<void(uint16_t address, uint16_t old_value, uint16_t new_value)>;
using StatusCb = std::function<void(const char *status)>;

// ---------------------------------------------------------------------------
// Register addresses (UTY-TFSXW1). Named after their function where known;
// the rest keep the upstream "RegisterNN" names so log dumps line up with the
// original project's notes.
// ---------------------------------------------------------------------------
namespace reg {
constexpr uint16_t Initial0 = 0x0001;
constexpr uint16_t Initial1 = 0x0101;

constexpr uint16_t Initial2 = 0x0110;
constexpr uint16_t Initial3 = 0x0111;
constexpr uint16_t Initial4 = 0x0112;
constexpr uint16_t Initial5 = 0x0113;
constexpr uint16_t Initial6 = 0x0114;
constexpr uint16_t Initial7 = 0x0115;
constexpr uint16_t Initial8 = 0x0117;
constexpr uint16_t Initial9 = 0x011A;
constexpr uint16_t Initial10 = 0x011D;
constexpr uint16_t Initial11 = 0x0120;
constexpr uint16_t VerticalAirflowDirectionCount = 0x0130;
constexpr uint16_t VerticalSwingSupported = 0x0131;
constexpr uint16_t HorizontalAirflowDirectionCount = 0x0142;
constexpr uint16_t HorizontalSwingSupported = 0x0143;

constexpr uint16_t EconomyModeSupported = 0x0150;
constexpr uint16_t MinimumHeatSupported = 0x0151;
constexpr uint16_t HumanSensorSupported = 0x0152;
constexpr uint16_t EnergySavingFanSupported = 0x0153;
constexpr uint16_t Initial20 = 0x0154;
constexpr uint16_t Initial21 = 0x0155;
constexpr uint16_t Initial22 = 0x0156;
constexpr uint16_t PowerfulSupported = 0x0170;
constexpr uint16_t OutdoorUnitLowNoiseSupported = 0x0171;
constexpr uint16_t CoilDrySupported = 0x0193;

constexpr uint16_t Power = 0x1000;
constexpr uint16_t Mode = 0x1001;
constexpr uint16_t SetpointTemp = 0x1002;
constexpr uint16_t FanSpeed = 0x1003;
constexpr uint16_t VerticalAirflowSetterRegistry = 0x1010;
constexpr uint16_t VerticalSwing = 0x1011;
constexpr uint16_t VerticalAirflow = 0x10A0;
constexpr uint16_t HorizontalAirflowSetterRegistry = 0x1022;
constexpr uint16_t HorizontalSwing = 0x1023;
constexpr uint16_t HorizontalAirflow = 0x10A9;
constexpr uint16_t Register11 = 0x1031;
constexpr uint16_t ActualTemp = 0x1033;
constexpr uint16_t Register13 = 0x1034;

constexpr uint16_t EconomyMode = 0x1100;
constexpr uint16_t MinimumHeat = 0x1101;
constexpr uint16_t HumanSensor = 0x1102;
constexpr uint16_t Register17 = 0x1103;
constexpr uint16_t Register18 = 0x1104;
constexpr uint16_t Register19 = 0x1105;
constexpr uint16_t Register20 = 0x1106;
constexpr uint16_t Register21 = 0x1107;
constexpr uint16_t EnergySavingFan = 0x1108;
constexpr uint16_t Register23 = 0x1109;
constexpr uint16_t Powerful = 0x1120;
constexpr uint16_t OutdoorUnitLowNoise = 0x1121;
constexpr uint16_t CoilDry = 0x1144;
constexpr uint16_t Register27 = 0x1200;
constexpr uint16_t Register28 = 0x1201;
constexpr uint16_t Register29 = 0x1202;
constexpr uint16_t Register30 = 0x1203;
constexpr uint16_t Register31 = 0x1204;
constexpr uint16_t Register32 = 0x1141;

constexpr uint16_t Register33 = 0x1400;
constexpr uint16_t Register34 = 0x1401;
constexpr uint16_t Register35 = 0x1402;
constexpr uint16_t Register36 = 0x1403;
constexpr uint16_t Register37 = 0x1404;
constexpr uint16_t Register38 = 0x1405;
constexpr uint16_t Register39 = 0x1406;
constexpr uint16_t Register40 = 0x140E;
constexpr uint16_t Register41 = 0x2000;
constexpr uint16_t OutdoorTemp = 0x2020;
constexpr uint16_t Register43 = 0x2021;
constexpr uint16_t Register44 = 0xF001;
}  // namespace reg

// ---------------------------------------------------------------------------
// Register value enumerations.
// ---------------------------------------------------------------------------
enum class Power : uint16_t { Off = 0x0000, On = 0x0001 };

enum class Mode : uint16_t {
  Auto = 0x0000,
  Cool = 0x0001,
  Dry = 0x0002,
  Fan = 0x0003,
  Heat = 0x0004,
  None = 0x0005,  // internal / powered-off sentinel
};

enum class FanSpeed : uint16_t {
  Auto = 0x0000,
  Quiet = 0x0002,
  Low = 0x0005,
  Medium = 0x0008,
  High = 0x000B,
};

enum class Airflow : uint16_t {
  Position1 = 0x0001,
  Position2 = 0x0002,
  Position3 = 0x0003,
  Position4 = 0x0004,
  Position5 = 0x0005,
  Position6 = 0x0006,
  Swing = 0x0020,
};

// The many on/off feature registers all share this encoding.
constexpr uint16_t OFF = 0x0000;
constexpr uint16_t ON = 0x0001;

// Temperature encoding constants (see TFSXW1Bridge::valueToString upstream).
constexpr int TEMP_OFFSET = 5025;   // current/outdoor: celsius = (raw - 5025) / 100
constexpr int SETPOINT_MIN_COOL = 180;  // 18.0 C (tenths)
constexpr int SETPOINT_MIN_HEAT = 160;  // 16.0 C (tenths)
constexpr int SETPOINT_MAX = 300;       // 30.0 C (tenths)

// ---------------------------------------------------------------------------
// Frame checksum + framing helpers (pure functions, easy to unit test).
// ---------------------------------------------------------------------------

// Fujitsu checksum: 0xFFFF minus the (wrapping) sum of every byte except the
// trailing 2 checksum bytes.
uint16_t frame_checksum(const uint8_t *buffer, int length_without_checksum);

// True when the trailing 2 bytes match frame_checksum() of the rest.
bool frame_is_valid(const uint8_t *buffer, int size);

// Build the two fixed handshake frames. Returned vectors include the checksum.
std::vector<uint8_t> build_init1();
std::vector<uint8_t> build_init2();

// Build a "read these registers" request (frame type 0x03).
std::vector<uint8_t> build_read_request(const uint16_t *addresses, size_t count);

// Build a "write these registers" request (frame type 0x02).
std::vector<uint8_t> build_write_request(const std::pair<uint16_t, uint16_t> *regs, size_t count);

// ---------------------------------------------------------------------------
// Incoming byte reassembler. Mirrors the upstream Buffer: a >=20 ms gap resets
// the frame, and a frame is considered complete once (length byte + 7) bytes
// have arrived. Adds bounds safety and a reset after each emitted frame.
// ---------------------------------------------------------------------------
class FrameParser {
 public:
  using Callback = std::function<void(const uint8_t *buffer, int size, bool valid)>;

  // Drain everything currently available from `io`, emitting complete frames.
  void poll(ByteIO &io, uint32_t now_ms, const Callback &callback);

  // Feed a single byte (used by tests and by poll()).
  void feed_byte(uint8_t byte, uint32_t now_ms, const Callback &callback);

  void reset() { index_ = 0; }

 private:
  static constexpr int kMaxFrame = 128;
  uint32_t last_ms_ = 0;
  uint8_t buffer_[kMaxFrame] = {0};
  int index_ = 0;
};

// ---------------------------------------------------------------------------
// The controller state machine.
// ---------------------------------------------------------------------------
class FujitsuProtocol {
 public:
  FujitsuProtocol(ByteIO &io, MillisFn millis);

  void set_register_change_cb(RegisterChangeCb cb) { register_change_cb_ = std::move(cb); }
  void set_status_cb(StatusCb cb) { status_cb_ = std::move(cb); }
  void set_log_cb(LogCb cb) { log_cb_ = std::move(cb); }
  void set_log_raw_frames(bool enable) { log_raw_frames_ = enable; }

  void setup();
  void loop();

  // ---- High level commands (queued; drained one write-frame per poll cycle).
  void set_power(Power power);
  void set_mode(Mode mode);
  void set_fan_speed(FanSpeed speed);
  void set_setpoint_celsius(float celsius);
  void set_vertical_swing(bool on);
  void set_horizontal_swing(bool on);
  void set_vertical_airflow(Airflow position);
  void set_horizontal_airflow(Airflow position);
  void set_powerful(bool on);
  void set_economy(bool on);

  // ---- State queries.
  bool has_register(uint16_t address) const;
  uint16_t get_register(uint16_t address) const;  // 0x0000 if unknown
  bool is_powered_on() const;
  bool is_feature_supported(uint16_t feature_address) const;
  bool is_running() const { return running_; }
  bool is_terminated() const { return terminated_; }
  int vertical_airflow_direction_count() const { return get_register(reg::VerticalAirflowDirectionCount); }
  int horizontal_airflow_direction_count() const { return get_register(reg::HorizontalAirflowDirectionCount); }

  // Iterate every known register (used to publish initial state).
  const std::unordered_map<uint16_t, uint16_t> &registers() const { return registers_; }

  // ---- Static, pure conversion helpers (unit-tested directly).
  // Round a celsius setpoint to the nearest 0.5 C and clamp; returns tenths.
  static int round_setpoint_tenths(double celsius, bool heat_mode);
  // Current / outdoor temperature raw -> celsius.
  static float raw_to_celsius(uint16_t raw) { return (static_cast<int>(raw) - TEMP_OFFSET) / 100.0f; }
  // Setpoint raw -> celsius.
  static float setpoint_raw_to_celsius(uint16_t raw) { return raw / 10.0f; }

 private:
  enum class FrameType : int {
    None = -1,
    Init1 = 0,
    Init2 = 1,
    InitialRegistries1 = 2,
    InitialRegistries2 = 3,
    InitialRegistries3 = 4,
    FrameA = 5,
    FrameB = 6,
    FrameC = 7,
    SendRegistries = 8,
    CheckRegistries = 9,
  };

  struct WriteCommand {
    std::vector<std::pair<uint16_t, uint16_t>> regs;
  };

  ByteIO &io_;
  MillisFn millis_;

  RegisterChangeCb register_change_cb_;
  StatusCb status_cb_;
  LogCb log_cb_;
  bool log_raw_frames_ = false;

  std::unordered_map<uint16_t, uint16_t> registers_;

  FrameParser parser_;

  uint32_t last_request_ms_ = 0;
  bool last_response_received_ = true;
  bool no_response_notified_ = false;
  bool initialized_ = false;
  bool terminated_ = false;
  bool running_ = false;
  uint32_t terminated_ms_ = 0;

  FrameType last_frame_sent_ = FrameType::None;

  std::deque<WriteCommand> write_queue_;
  WriteCommand active_write_;  // command currently being written/verified

  // state machine steps
  void send_request();
  void request_registers(FrameType type, const uint16_t *addresses, size_t count);
  void send_write(const WriteCommand &cmd);
  void on_frame(const uint8_t *buffer, int size, bool valid);
  void update_registers(const uint8_t *buffer, int size);

  void enqueue(WriteCommand cmd);
  bool guard_blocks_change();  // coil-dry / minimum-heat lockouts

  void set_status(const char *status);
  void log(const char *level, const char *msg);
};

}  // namespace faircon
