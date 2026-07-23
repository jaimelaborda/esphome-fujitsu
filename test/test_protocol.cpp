/*
  Host unit tests for the Fujitsu (UTY-TFSXW1) protocol core.

  Build & run (Windows, from a Visual Studio "Developer Command Prompt", or via
  test/run_tests.ps1 which sets that environment up for you):

      cl /nologo /std:c++17 /EHsc /W3 ^
         test\test_protocol.cpp components\fujitsu\fujitsu_protocol.cpp ^
         /Fe:build\fujitsu_tests.exe
      build\fujitsu_tests.exe

  No external test framework is required.
*/

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "../components/fujitsu/fujitsu_protocol.h"
#include "fake_ac.h"

using namespace faircon;

// --------------------------------------------------------------------------
// Tiny test harness.
// --------------------------------------------------------------------------
static int g_checks = 0;
static int g_failures = 0;
static const char *g_current_test = "";

#define CHECK(cond)                                                            \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_failures++;                                                            \
      std::printf("  FAIL [%s] line %d: %s\n", g_current_test, __LINE__, #cond); \
    }                                                                          \
  } while (0)

#define CHECK_EQ_INT(actual, expected)                                              \
  do {                                                                              \
    g_checks++;                                                                     \
    long long a = (long long) (actual);                                            \
    long long e = (long long) (expected);                                          \
    if (a != e) {                                                                   \
      g_failures++;                                                                 \
      std::printf("  FAIL [%s] line %d: %s == %s (got %lld, want %lld)\n",          \
                  g_current_test, __LINE__, #actual, #expected, a, e);             \
    }                                                                              \
  } while (0)

static bool almost_equal(float a, float b) { return std::fabs(a - b) < 0.001f; }

#define CHECK_FLOAT(actual, expected)                                               \
  do {                                                                              \
    g_checks++;                                                                     \
    float a = (float) (actual);                                                     \
    float e = (float) (expected);                                                   \
    if (!almost_equal(a, e)) {                                                      \
      g_failures++;                                                                 \
      std::printf("  FAIL [%s] line %d: %s == %s (got %f, want %f)\n",              \
                  g_current_test, __LINE__, #actual, #expected, a, e);             \
    }                                                                              \
  } while (0)

static void check_bytes(const std::vector<uint8_t> &actual,
                        const std::vector<uint8_t> &expected, int line) {
  g_checks++;
  bool ok = actual.size() == expected.size();
  if (ok) {
    for (size_t i = 0; i < actual.size(); i++) {
      if (actual[i] != expected[i]) {
        ok = false;
        break;
      }
    }
  }
  if (!ok) {
    g_failures++;
    std::printf("  FAIL [%s] line %d: byte mismatch\n    got  ", g_current_test, line);
    for (uint8_t b : actual) std::printf("%02X ", b);
    std::printf("\n    want ");
    for (uint8_t b : expected) std::printf("%02X ", b);
    std::printf("\n");
  }
}
#define CHECK_BYTES(actual, expected) check_bytes((actual), (expected), __LINE__)

// --------------------------------------------------------------------------
// Fake clock shared by the integration tests.
// --------------------------------------------------------------------------
static uint32_t g_now = 0;
static uint32_t fake_millis() { return g_now; }

// A wired-up controller + simulated AC.
struct Rig {
  std::deque<uint8_t> to_ac;
  std::deque<uint8_t> to_ctrl;
  QueueIO ctrl_io{&to_ctrl, &to_ac};  // controller reads AC->ctrl, writes ctrl->AC
  FujitsuProtocol proto{ctrl_io, fake_millis};
  FakeAC ac{&to_ac, &to_ctrl, fake_millis};

  Rig() {
    g_now = 1000;  // start at a non-zero time
    proto.setup();
  }

  // Advance one 400 ms poll cycle: controller sends, AC answers, controller reads.
  void cycle() {
    g_now += 400;
    proto.loop();
    ac.step();
    proto.loop();
  }
  void run(int cycles) {
    for (int i = 0; i < cycles; i++) cycle();
  }
};

// --------------------------------------------------------------------------
// Tests.
// --------------------------------------------------------------------------
static void test_checksum() {
  g_current_test = "checksum";
  // Init1 frame is a known-good vector.
  std::vector<uint8_t> init1 = {0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFB};
  CHECK(frame_is_valid(init1.data(), (int) init1.size()));
  CHECK_EQ_INT(frame_checksum(init1.data(), 9), 0xFFFB);

  // Corrupt one byte -> invalid.
  init1[6] = 0x01;
  CHECK(!frame_is_valid(init1.data(), (int) init1.size()));

  // Init1 ack.
  std::vector<uint8_t> ack = {0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFD};
  CHECK(frame_is_valid(ack.data(), (int) ack.size()));
}

static void test_build_init_frames() {
  g_current_test = "build_init_frames";
  CHECK_BYTES(build_init1(), (std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFB}));
  CHECK_BYTES(build_init2(), (std::vector<uint8_t>{0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x01, 0xFF, 0xF5}));
}

static void test_build_read_request() {
  g_current_test = "build_read_request";
  // Read the two initial registers 0x0001, 0x0101.
  uint16_t addrs[] = {0x0001, 0x0101};
  auto f = build_read_request(addrs, 2);
  // type 03, len=4, payload=00 01 01 01, checksum = 0xFFFF-03-04-00-01-01-01 = 0xFFF5
  CHECK_BYTES(f, (std::vector<uint8_t>{0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x01, 0x01, 0x01, 0xFF, 0xF5}));
  CHECK(frame_is_valid(f.data(), (int) f.size()));
}

static void test_build_write_request() {
  g_current_test = "build_write_request";
  // Write Power(0x1000) = On(0x0001).
  std::pair<uint16_t, uint16_t> regs[] = {{0x1000, 0x0001}};
  auto f = build_write_request(regs, 1);
  // type 02, len=4, payload=10 00 00 01, checksum = 0xFFFF-02-04-10-00-00-01 = 0xFFE8
  CHECK_BYTES(f, (std::vector<uint8_t>{0x02, 0x00, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x01, 0xFF, 0xE8}));
  CHECK(frame_is_valid(f.data(), (int) f.size()));

  // Two registers in one frame.
  std::pair<uint16_t, uint16_t> regs2[] = {{0x1000, 0x0001}, {0x1001, 0x0004}};
  auto f2 = build_write_request(regs2, 2);
  CHECK_EQ_INT(f2[4], 8);  // payload length
  CHECK(frame_is_valid(f2.data(), (int) f2.size()));
}

static void test_frame_parser() {
  g_current_test = "frame_parser";
  // Feed a valid init ack byte-by-byte; expect exactly one valid frame.
  std::vector<uint8_t> ack = {0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFD};
  FrameParser parser;
  int frames = 0, valid_frames = 0, last_size = 0;
  auto cb = [&](const uint8_t *, int size, bool valid) {
    frames++;
    if (valid) valid_frames++;
    last_size = size;
  };
  uint32_t t = 100;
  for (uint8_t b : ack) parser.feed_byte(b, t, cb);
  CHECK_EQ_INT(frames, 1);
  CHECK_EQ_INT(valid_frames, 1);
  CHECK_EQ_INT(last_size, 8);

  // Two frames back-to-back at the same timestamp -> emit-reset handles both.
  frames = valid_frames = 0;
  for (uint8_t b : ack) parser.feed_byte(b, t, cb);
  for (uint8_t b : ack) parser.feed_byte(b, t, cb);
  CHECK_EQ_INT(frames, 2);
  CHECK_EQ_INT(valid_frames, 2);

  // A >=20 ms gap mid-frame resets, so a truncated frame is discarded.
  frames = valid_frames = 0;
  parser.reset();
  parser.feed_byte(0x00, 1000, cb);
  parser.feed_byte(0x00, 1000, cb);
  parser.feed_byte(0x00, 1000, cb);        // partial (3 bytes)
  for (uint8_t b : ack) parser.feed_byte(b, 1100, cb);  // gap -> fresh frame
  CHECK_EQ_INT(frames, 1);
  CHECK_EQ_INT(valid_frames, 1);

  // Invalid checksum still emits a frame, flagged invalid.
  frames = valid_frames = 0;
  parser.reset();
  std::vector<uint8_t> bad = {0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x12, 0x34};
  for (uint8_t b : bad) parser.feed_byte(b, 5000, cb);
  CHECK_EQ_INT(frames, 1);
  CHECK_EQ_INT(valid_frames, 0);
}

static void test_temp_conversions() {
  g_current_test = "temp_conversions";
  // Current/outdoor: celsius = (raw - 5025) / 100  (DummyUnit documented vectors)
  CHECK_FLOAT(FujitsuProtocol::raw_to_celsius(0x1D97), 25.5f);
  CHECK_FLOAT(FujitsuProtocol::raw_to_celsius(0x1C6B), 22.5f);
  CHECK_FLOAT(FujitsuProtocol::raw_to_celsius(0x1B71), 20.0f);
  CHECK_FLOAT(FujitsuProtocol::raw_to_celsius(0x1964), 14.75f);  // outdoor
  CHECK_FLOAT(FujitsuProtocol::raw_to_celsius(0x15E0), 5.75f);

  // Setpoint: celsius = raw / 10
  CHECK_FLOAT(FujitsuProtocol::setpoint_raw_to_celsius(0x00FA), 25.0f);
  CHECK_FLOAT(FujitsuProtocol::setpoint_raw_to_celsius(180), 18.0f);

  // Rounding to nearest 0.5 C, returned in tenths.
  CHECK_EQ_INT(FujitsuProtocol::round_setpoint_tenths(24.3, false), 245);  // 24.3 -> 24.5
  CHECK_EQ_INT(FujitsuProtocol::round_setpoint_tenths(24.2, false), 240);  // 24.2 -> 24.0
  CHECK_EQ_INT(FujitsuProtocol::round_setpoint_tenths(25.0, false), 250);
  // Clamps: cool minimum 18.0, heat minimum 16.0, max 30.0.
  CHECK_EQ_INT(FujitsuProtocol::round_setpoint_tenths(10.0, false), 180);
  CHECK_EQ_INT(FujitsuProtocol::round_setpoint_tenths(10.0, true), 160);
  CHECK_EQ_INT(FujitsuProtocol::round_setpoint_tenths(35.0, false), 300);
  CHECK_EQ_INT(FujitsuProtocol::round_setpoint_tenths(17.0, true), 170);  // allowed in heat
}

static void test_handshake_and_read() {
  g_current_test = "handshake_and_read";
  Rig rig;
  int status_running = 0;
  rig.proto.set_status_cb([&](const char *s) {
    if (std::string(s) == "Running") status_running++;
  });

  rig.run(20);

  CHECK(rig.proto.is_running());
  CHECK(!rig.proto.is_terminated());
  CHECK(status_running > 0);

  // Registers now mirror the AC.
  CHECK_EQ_INT(rig.proto.get_register(reg::Power), 0x0001);
  CHECK_EQ_INT(rig.proto.get_register(reg::Mode), 0x0001);           // cool
  CHECK_EQ_INT(rig.proto.get_register(reg::SetpointTemp), 0x00FA);   // 25.0
  CHECK_EQ_INT(rig.proto.get_register(reg::ActualTemp), 0x1B71);     // 20.0
  CHECK_EQ_INT(rig.proto.get_register(reg::OutdoorTemp), 0x1964);    // 14.75
  CHECK(rig.proto.is_powered_on());

  // Feature detection.
  CHECK(rig.proto.is_feature_supported(reg::VerticalSwingSupported));
  CHECK(rig.proto.is_feature_supported(reg::VerticalAirflowDirectionCount));  // 6 > 0
  CHECK(rig.proto.is_feature_supported(reg::PowerfulSupported));

  // Derived values.
  CHECK_FLOAT(FujitsuProtocol::setpoint_raw_to_celsius(rig.proto.get_register(reg::SetpointTemp)), 25.0f);
  CHECK_FLOAT(FujitsuProtocol::raw_to_celsius(rig.proto.get_register(reg::ActualTemp)), 20.0f);
}

static void test_write_power_off() {
  g_current_test = "write_power_off";
  Rig rig;
  rig.run(20);
  CHECK(rig.proto.is_powered_on());

  rig.proto.set_power(Power::Off);
  rig.run(20);

  CHECK_EQ_INT(rig.ac.reg(reg::Power), 0x0000);          // AC actually turned off
  CHECK_EQ_INT(rig.proto.get_register(reg::Power), 0x0000);  // and controller re-read it
  CHECK(!rig.proto.is_powered_on());
}

static void test_write_setpoint() {
  g_current_test = "write_setpoint";
  Rig rig;
  rig.run(20);

  rig.proto.set_setpoint_celsius(22.0f);
  rig.run(20);

  CHECK_EQ_INT(rig.ac.reg(reg::SetpointTemp), 220);
  CHECK_EQ_INT(rig.proto.get_register(reg::SetpointTemp), 220);
}

static void test_write_mode_and_queue() {
  g_current_test = "write_mode_and_queue";
  Rig rig;
  rig.run(20);

  // Queue several commands at once (as an HA climate call would).
  rig.proto.set_mode(Mode::Heat);
  rig.proto.set_fan_speed(FanSpeed::High);
  rig.run(30);

  CHECK_EQ_INT(rig.ac.reg(reg::Mode), (uint16_t) Mode::Heat);
  CHECK_EQ_INT(rig.ac.reg(reg::FanSpeed), (uint16_t) FanSpeed::High);
  CHECK_EQ_INT(rig.proto.get_register(reg::Mode), (uint16_t) Mode::Heat);
  CHECK_EQ_INT(rig.proto.get_register(reg::FanSpeed), (uint16_t) FanSpeed::High);

  // Now in heat mode, a very low setpoint clamps to 16.0 C.
  rig.proto.set_setpoint_celsius(12.0f);
  rig.run(20);
  CHECK_EQ_INT(rig.ac.reg(reg::SetpointTemp), 160);
}

static void test_fan_mode_blocks_setpoint() {
  g_current_test = "fan_mode_blocks_setpoint";
  Rig rig;
  rig.run(20);

  rig.proto.set_mode(Mode::Fan);
  rig.run(20);
  CHECK_EQ_INT(rig.ac.reg(reg::Mode), (uint16_t) Mode::Fan);

  uint16_t before = rig.ac.reg(reg::SetpointTemp);
  rig.proto.set_setpoint_celsius(22.0f);  // must be ignored in fan mode
  rig.run(20);
  CHECK_EQ_INT(rig.ac.reg(reg::SetpointTemp), before);
}

static void test_feature_gating() {
  g_current_test = "feature_gating";
  Rig rig;
  rig.ac.set_reg(reg::HorizontalSwingSupported, 0x0000);  // this unit lacks H-swing
  rig.run(20);

  CHECK(!rig.proto.is_feature_supported(reg::HorizontalSwingSupported));

  uint16_t before = rig.ac.reg(reg::HorizontalSwing);
  rig.proto.set_horizontal_swing(true);  // should be dropped
  rig.run(20);
  CHECK_EQ_INT(rig.ac.reg(reg::HorizontalSwing), before);

  // Vertical swing IS supported and should go through.
  rig.proto.set_vertical_swing(true);
  rig.run(20);
  CHECK_EQ_INT(rig.ac.reg(reg::VerticalSwing), 0x0001);
}

static void test_terminate_and_recover() {
  g_current_test = "terminate_and_recover";
  Rig rig;
  rig.ac.set_bad_init1(true);
  rig.run(6);
  CHECK(rig.proto.is_terminated());
  CHECK(!rig.proto.is_running());

  // The AC starts behaving; after the 10 s cooldown the controller re-inits.
  rig.ac.set_bad_init1(false);
  g_now += 11000;  // pass the recovery cooldown
  rig.run(20);
  CHECK(!rig.proto.is_terminated());
  CHECK(rig.proto.is_running());
  CHECK_EQ_INT(rig.proto.get_register(reg::SetpointTemp), 0x00FA);
}

static void test_register_change_callback() {
  g_current_test = "register_change_callback";
  Rig rig;
  int power_changes = 0;
  uint16_t last_power = 0xFFFF;
  rig.proto.set_register_change_cb([&](uint16_t addr, uint16_t, uint16_t nv) {
    if (addr == reg::Power) {
      power_changes++;
      last_power = nv;
    }
  });
  rig.run(20);           // initial read: 0 -> 1
  CHECK(power_changes >= 1);
  CHECK_EQ_INT(last_power, 0x0001);

  rig.proto.set_power(Power::Off);
  rig.run(20);           // 1 -> 0
  CHECK_EQ_INT(last_power, 0x0000);
}

int main() {
  std::printf("Running Fujitsu protocol tests...\n");

  test_checksum();
  test_build_init_frames();
  test_build_read_request();
  test_build_write_request();
  test_frame_parser();
  test_temp_conversions();
  test_handshake_and_read();
  test_write_power_off();
  test_write_setpoint();
  test_write_mode_and_queue();
  test_fan_mode_blocks_setpoint();
  test_feature_gating();
  test_terminate_and_recover();
  test_register_change_callback();

  std::printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("TESTS FAILED\n");
  return 1;
}
