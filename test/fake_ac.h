/*
  FakeAC - an in-memory Fujitsu indoor-unit simulator for host unit tests.

  It speaks the AC side of the UTY-TFSXW1 protocol: it answers the two init
  frames, serves register reads, and applies register writes - exactly like the
  (commented-out) DummyUnit in the upstream project. Wiring a FujitsuProtocol to
  a FakeAC through a byte pipe lets us test the full handshake + read/write loop
  entirely on a PC.
*/

#pragma once

#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

#include "../components/fujitsu/fujitsu_protocol.h"

// A ByteIO backed by two shared FIFOs (one per direction).
class QueueIO : public faircon::ByteIO {
 public:
  QueueIO(std::deque<uint8_t> *in, std::deque<uint8_t> *out) : in_(in), out_(out) {}
  int available() override { return static_cast<int>(in_->size()); }
  int read() override {
    if (in_->empty()) {
      return -1;
    }
    uint8_t b = in_->front();
    in_->pop_front();
    return b;
  }
  void write(const uint8_t *data, size_t len) override {
    for (size_t i = 0; i < len; i++) {
      out_->push_back(data[i]);
    }
  }

 private:
  std::deque<uint8_t> *in_;
  std::deque<uint8_t> *out_;
};

class FakeAC {
 public:
  // `to_ac` carries controller->AC bytes, `to_ctrl` carries AC->controller bytes.
  FakeAC(std::deque<uint8_t> *to_ac, std::deque<uint8_t> *to_ctrl,
         faircon::MillisFn millis)
      : io_(to_ac, to_ctrl), millis_(std::move(millis)) {
    seed_defaults();
  }

  // Make the unit reply to the first init frame with a wrong ack, to exercise
  // the controller's terminate/recover path.
  void set_bad_init1(bool bad) { bad_init1_ = bad; }

  void set_reg(uint16_t addr, uint16_t value) { regs_[addr] = value; }
  uint16_t reg(uint16_t addr) const {
    auto it = regs_.find(addr);
    return it == regs_.end() ? 0x0000 : it->second;
  }

  // Process every complete frame currently queued from the controller.
  void step() {
    parser_.poll(io_, millis_(), [this](const uint8_t *buf, int size, bool valid) {
      if (valid) {
        on_frame(buf, size);
      }
    });
  }

 private:
  QueueIO io_;
  faircon::MillisFn millis_;
  faircon::FrameParser parser_;
  std::unordered_map<uint16_t, uint16_t> regs_;
  bool bad_init1_ = false;

  void on_frame(const uint8_t *buf, int size) {
    switch (buf[0]) {
      case 0x00: {  // init1
        if (bad_init1_) {
          uint8_t bad[8] = {0xAA, 0x00, 0x00, 0x00, 0x01, 0x09, 0x00, 0x00};
          uint16_t cs = faircon::frame_checksum(bad, 6);
          bad[6] = static_cast<uint8_t>((cs >> 8) & 0xFF);
          bad[7] = static_cast<uint8_t>(cs & 0xFF);
          io_.write(bad, 8);
        } else {
          uint8_t ack[8] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFD};
          io_.write(ack, 8);
        }
        break;
      }
      case 0x01: {  // init2
        uint8_t ack[8] = {0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFC};
        io_.write(ack, 8);
        break;
      }
      case 0x02:  // write registers
        apply_write(buf, size);
        break;
      case 0x03:  // read registers
        respond_read(buf, size);
        break;
      default:
        break;
    }
  }

  void apply_write(const uint8_t *buf, int /*size*/) {
    int count = buf[4] / 4;
    for (int i = 0; i < count; i++) {
      int idx = 5 + i * 4;
      uint16_t addr = static_cast<uint16_t>((buf[idx] << 8) | buf[idx + 1]);
      uint16_t val = static_cast<uint16_t>((buf[idx + 2] << 8) | buf[idx + 3]);
      regs_[addr] = val;
    }
    uint8_t ack[8] = {0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFB};
    io_.write(ack, 8);
  }

  void respond_read(const uint8_t *buf, int /*size*/) {
    int count = buf[4] / 2;
    std::vector<uint8_t> r = {0x03, 0x00, 0x00, 0x00,
                              static_cast<uint8_t>(4 * count + 1), 0x01};
    for (int i = 0; i < count; i++) {
      uint16_t addr = static_cast<uint16_t>((buf[5 + i * 2] << 8) | buf[5 + i * 2 + 1]);
      uint16_t val = reg(addr);
      r.push_back(static_cast<uint8_t>((addr >> 8) & 0xFF));
      r.push_back(static_cast<uint8_t>(addr & 0xFF));
      r.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
      r.push_back(static_cast<uint8_t>(val & 0xFF));
    }
    uint16_t cs = faircon::frame_checksum(r.data(), static_cast<int>(r.size()));
    r.push_back(static_cast<uint8_t>((cs >> 8) & 0xFF));
    r.push_back(static_cast<uint8_t>(cs & 0xFF));
    io_.write(r.data(), r.size());
  }

  // A realistic register snapshot (values borrowed from the upstream DummyUnit):
  // powered on, cooling, setpoint 25.0 C, current 20.0 C, outdoor 14.75 C.
  void seed_defaults() {
    using namespace faircon::reg;
    regs_[VerticalAirflowDirectionCount] = 0x0006;
    regs_[VerticalSwingSupported] = 0x0001;
    regs_[HorizontalAirflowDirectionCount] = 0x0015;
    regs_[HorizontalSwingSupported] = 0x0001;
    regs_[EconomyModeSupported] = 0x0001;
    regs_[MinimumHeatSupported] = 0x0001;
    regs_[HumanSensorSupported] = 0x0001;
    regs_[EnergySavingFanSupported] = 0x0001;
    regs_[PowerfulSupported] = 0x0001;
    regs_[OutdoorUnitLowNoiseSupported] = 0x0001;
    regs_[CoilDrySupported] = 0x0001;

    regs_[Power] = 0x0001;
    regs_[Mode] = 0x0001;          // cool
    regs_[SetpointTemp] = 0x00FA;  // 250 -> 25.0 C
    regs_[FanSpeed] = 0x0002;      // quiet
    regs_[VerticalSwing] = 0x0000;
    regs_[VerticalAirflow] = 0x0001;
    regs_[HorizontalSwing] = 0x0000;
    regs_[ActualTemp] = 0x1B71;   // 7025 -> 20.0 C
    regs_[EconomyMode] = 0x0000;
    regs_[MinimumHeat] = 0x0000;
    regs_[Powerful] = 0x0000;
    regs_[CoilDry] = 0x0000;
    regs_[OutdoorTemp] = 0x1964;  // 6500 -> 14.75 C
  }
};
