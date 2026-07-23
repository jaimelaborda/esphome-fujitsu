# esphome-fujitsu

An [ESPHome](https://esphome.io/) external component that turns an ESP8266/ESP32
into a **local, wired controller** for Fujitsu air conditioners that use the
`UTY-TFSXW1` family of Wi-Fi adapters (the FGLair® dongle). It talks the AC's
serial protocol directly and exposes a native Home Assistant `climate` entity —
no cloud, no FGLair app.

The wire protocol was reverse-engineered by **Benas Ragauskas** in
[Benas09/FujitsuAC](https://github.com/Benas09/FujitsuAC) (an Arduino/MQTT
project for the ESP32). This repository is an **independent ESPHome port** of that
protocol: the protocol logic has been rewritten as a platform-agnostic core with
host-run unit tests, and wrapped in a proper ESPHome climate platform.

> FGLair is a registered trademark of Fujitsu General Limited. This project is not
> affiliated with or endorsed by Fujitsu.

## Compatible units

Any indoor unit that accepts the `UTY-TFSXW1` (and the Z / F / H series)
Wi-Fi adapter — most Fujitsu `ASYG…` / `AUYG…` etc. models made after ~2018 that
have the 4-pin "WLAN adapter" connector (CN). See the upstream
[tested-aircons list](https://github.com/Benas09/FujitsuAC/discussions/24).

If you are not sure your unit speaks this protocol, flash with `log_raw_frames:
true` (see below) and watch the boot log: reaching the `Running` status and seeing
registers populate means it works. Only `invalid checksum` / no response means it
does not (or the wiring/polarity is wrong).

## ⚠️ Hardware & safety

**Do not connect the AC's serial pins to any earth-referenced device (a PC, a USB
programmer sharing ground, …).** The AC bus is not galvanically isolated and its
"ground" is not earth ground; doing so can destroy the AC mainboard fuse and/or
your other device. Power and flash the ESP separately, and update it over the air
(OTA) afterwards.

Typical wiring (see the upstream project for photos and the exact connector
pinout):

```
 AC connector          level shift / supply           ESP
 +12V  ---------------> 12V->5V buck converter ------> 5V
 GND   ------------------------------------ common --> GND
 DATA (AC TX) --------> 3.3V level shifter ----------> RX
 DATA (AC RX) <-------- 3.3V level shifter <--------- TX
```

The ESP GPIOs are 3.3 V. Use a level shifter appropriate for your bus voltage; do
not wire the bus straight to the ESP unless you have confirmed the levels.

### Signal polarity (important)

The protocol uses **inverted** UART signalling. The upstream ESP32 library does
this in firmware (`uart_set_line_inverse`). This component reproduces it with
`inverted: true` on the ESP UART pins, so a plain (non-inverting) level shifter
works out of the box.

If your level shifter is a single-transistor type that already inverts the signal,
set `inverted: false` on both pins so the two inversions cancel. When in doubt,
try one polarity; if you only ever see `invalid checksum` in the log, flip it.

## UART settings

| Setting    | Value          |
| ---------- | -------------- |
| Baud rate  | **9600**       |
| Data bits  | 8              |
| Parity     | **none**       |
| Stop bits  | 1              |
| Signal     | **inverted**   |

On the **ESP8266 D1 mini** the AC bus must use hardware UART0 (`GPIO1`/`GPIO3`),
which is shared with the USB serial console — so the ESPHome `logger` must free it
with `baud_rate: 0`. Logs remain available over the network (API/OTA). On an ESP32
you can use any spare UART pins instead.

## Installation

Add the component to your device YAML via `external_components` and configure a
`uart` bus plus the `climate` platform:

```yaml
external_components:
  - source: github://jaimelaborda/esphome-fujitsu@master
    components: [fujitsu]

logger:
  baud_rate: 0            # ESP8266: free UART0 for the AC bus

uart:
  id: fujitsu_uart
  tx_pin:
    number: GPIO1
    inverted: true
  rx_pin:
    number: GPIO3
    inverted: true
  baud_rate: 9600
  data_bits: 8
  parity: NONE
  stop_bits: 1
  rx_buffer_size: 256

climate:
  - platform: fujitsu
    name: "Fujitsu Aire"
    uart_id: fujitsu_uart
    horizontal_swing: true
    presets: true
    log_raw_frames: false
    outdoor_temperature:
      name: "Fujitsu Outdoor Temperature"
```

A complete device example is in [`example.yaml`](example.yaml).

## Configuration options

`climate` platform `fujitsu`:

| Option                 | Default | Description                                                                                                    |
| ---------------------- | ------- | -------------------------------------------------------------------------------------------------------------- |
| `uart_id`              | —       | ID of the `uart` bus (required).                                                                               |
| `name`                 | —       | Climate entity name.                                                                                           |
| `horizontal_swing`     | `false` | Expose horizontal-swing options (`horizontal`/`both`). The firmware still auto-detects and ignores it if the unit has no horizontal louver. |
| `presets`              | `true`  | Map the unit's **Powerful** function to the `boost` preset and **Economy** to `eco`.                           |
| `log_raw_frames`       | `false` | Log every raw UART frame at `DEBUG`. Very verbose — for bring-up/debugging only.                               |
| `outdoor_temperature`  | —       | Optional `sensor` for the outdoor coil temperature. Takes the usual sensor options (`name`, `id`, filters, …). |

All standard `climate` options are supported too. Modes exposed: `off`,
`heat_cool` (the unit's *Auto*), `cool`, `dry`, `fan_only`, `heat`. Fan modes:
`auto`, `quiet`, `low`, `medium`, `high`.

## How it works

```
Home Assistant  <--API-->  ESPHome
                             └── climate: fujitsu   (fujitsu_climate.*)
                                   └── protocol core (fujitsu_protocol.*)
                                         └── uart (9600 8N1, inverted)  <-->  AC
```

- `components/fujitsu/fujitsu_protocol.{h,cpp}` — a **platform-agnostic** C++ core:
  framing + checksums, the polling state machine (`Init1 → Init2 → read feature/
  state registers → poll A/B/C`), the register table, and the temperature maths.
  It has no ESPHome or Arduino dependencies, which is what makes it unit-testable.
- `components/fujitsu/fujitsu_climate.{h,cpp}` — the ESPHome glue: adapts the UART
  to the core, maps registers ⇄ climate state, and handles Home Assistant commands.

Commands are queued and drained one write-frame per poll cycle, so several
attribute changes in a single HA call are all applied. If the handshake ever
fails, the controller logs the offending bytes and automatically retries after a
few seconds (no reboot needed).

## Unit tests

The protocol core is covered by host-run C++ tests (framing, checksums, frame
re-assembly, temperature conversions, and a full handshake + read/write round-trip
against an in-memory AC simulator). No hardware and no test framework required.

On Windows with Visual Studio 2022 (Desktop C++ workload):

```powershell
powershell -ExecutionPolicy Bypass -File test\run_tests.ps1
```

Or with any C++17 compiler:

```sh
c++ -std=c++17 test/test_protocol.cpp components/fujitsu/fujitsu_protocol.cpp -o fujitsu_tests
./fujitsu_tests
```

## Testing against a PC simulator

[`tools/fujitsu_ac_sim.py`](tools/fujitsu_ac_sim.py) makes a PC pretend to be the
AC, so you can validate the dongle firmware (and that the ESP's UART pins work)
over a safe 3.3 V link — no real AC, no 12 V bus. It answers the handshake, serves
a realistic AC state, and applies writes so Home Assistant commands round-trip.

Wire a **3.3 V** USB-UART TTL adapter to the ESP (crossed: adapter TX→ESP RX,
adapter RX→ESP TX, common GND), flash the ESP with non-inverted UART
(`inverted: false`, 9600 8N1), then:

```sh
pip install pyserial
python tools/fujitsu_ac_sim.py --list        # find your COM port
python tools/fujitsu_ac_sim.py --port COM5   # run the simulator
python tools/fujitsu_ac_sim.py --selftest    # verify the logic without hardware
```

If the ESP reaches `Status: Running` against the simulator, the firmware and the
ESP's serial pins are good, and any remaining problem is the physical AC interface.
The simulator accepts interactive commands (`show`, `power off`, `temp 21.5`, …) to
change the simulated state and watch it appear in Home Assistant.

## Troubleshooting

- **Only `invalid checksum` / no valid frames, never `Running`** → wrong signal
  polarity. Flip `inverted:` on both `tx_pin` and `rx_pin`.
- **`unexpected InitX response, terminating`** with hex bytes in the log → the unit
  answered but not as expected. Capture that log line; the raw bytes tell us
  whether it's a different protocol variant. The controller retries automatically.
- **Everything reads but commands do nothing** → check the unit isn't in *Coil
  Dry* or *Minimum Heat*, which lock out most changes (this is by design).
- Set `log_raw_frames: true` to see every frame during bring-up.

## Credits

- Protocol reverse-engineering: **Benas Ragauskas** —
  [Benas09/FujitsuAC](https://github.com/Benas09/FujitsuAC).
- ESPHome port: this repository.

## License

The ESPHome port in this repository is released under the [MIT License](LICENSE).
The underlying protocol knowledge comes from the upstream FujitsuAC project; please
also review and respect that project's terms.
