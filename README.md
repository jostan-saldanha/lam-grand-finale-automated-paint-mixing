# LAM Research Challenge 2026 - Stage 3 - Day 1: Mustard mix

Target: 3000 mL of Mustard = 10 % red (300 mL) + 10 % blue (300 mL) + 80 % yellow (2400 mL), from one START press.

## Folders

| Folder | Board | Role |
|---|---|---|
| `sensor_esp32_2/` | ESP32 #2 (U7) | Counts FS1..FS5 pulses (PCNT), streams totals to #1 at 50 Hz |
| `controller_esp32_1/` | ESP32 #1 (U2) | Pumps, valves, start switch, dosing sequence, USB CLI and telemetry |

Arduino IDE: board "ESP32 Dev Module", Arduino-ESP32 core 3.x. Open each folder's `.ino`. Flash both.

## Confirmed P&ID & BOM Architecture

Based on the hand-drawn P&ID and the BOM list:
- **5 Tanks (10 L)**: Flush Tank (`FT`), Tank 1 (`T1` - Red), Tank 2 (`T2` - Blue), Tank 3 (`T3` - Yellow), and `Mixing Tank`.
- **4 Solenoid Valves**: Located at the gravity outlet / suction of each tank (`FT`, `T1`, `T2`, `T3`) to isolate the tanks when idle.
- **4 DC Pumps (12V)**:
  - `Pump - 01`: Dedicated Flush pump from `FT`
  - `Pump - 02`: `T1` (Red) dosing pump
  - `Pump - 03`: `T2` (Blue) dosing pump
  - `Pump - 04`: `T3` (Yellow) dosing pump
- **3 Flow Sensors**: Located on the delivery side of each color line (`FT-01` on T1, `FT-02` on T2, `FT-03` on T3) leading directly into the `Mixing Tank`. The Flush line has no flow sensor.
- **3 Manual Ball Valves (`BV-01`, `BV-02`, `BV-03`)**: Selectively route flush water from `Pump - 01` into the suction lines of `Pump - 02`, `Pump - 03`, or `Pump - 04`.
- **Independent Delivery Lines**: Each color line runs directly into the `Mixing Tank`. There is no shared manifold header, no `SV-05`, and no waste discharge valve `SV-06`. Therefore, `HAVE_WASTE_VALVE = false` and cross-color contamination in tubing is eliminated.

| Line / Liquid | Pump Motor Driver | Solenoid Valve Terminal | Flow Transmitter Terminal | ESP32 Pin Mapping |
|---|---|---|---|---|
| **Red Line** (`L_RED` = 0) | Motor Driver 1: `M1A / M1B` | `SOV 1` (SV1) | `FT 1` (`FS1`) | Pump: `PWM_1` (GPIO 32, U2)<br>SV: GPIO 23 (U2)<br>FT: GPIO 33 (U7) |
| **Yellow Line** (`L_YELLOW` = 2) | Motor Driver 1: `M2A / M2B` | `SOV 2` (SV2) | `FT 2` (`FS2`) | Pump: `PWM_2` (GPIO 33, U2)<br>SV: GPIO 22 (U2)<br>FT: GPIO 25 (U7) |
| **Blue Line** (`L_BLUE` = 1) | Motor Driver 2: `M1A / M1B` | `SOV 3` (SV3) | `FT 3` (`FS3`) | Pump: `PWM_3` (GPIO 25, U2)<br>SV: GPIO 21 (U2)<br>FT: GPIO 26 (U7) |
| **Flush Line** (`L_FLUSH` = 3) | Motor Driver 2: `M2A / M2B` | `SOV 4` (SV4) | `FT 4` (`FS4`) | Pump: `PWM_4` (GPIO 26, U2)<br>SV: GPIO 19 (U2)<br>FT: GPIO 14 (U7) |
| **Start Switch** (`SW`) | Tactile Push Button | — | — | GPIO 14 (U2) |

## Batch sequence

1. Prime: first colour (yellow) pushes the old flush water out of the header to waste (SV-06).
2. Yellow, then blue, then red to the tank (SV-05). Each dose: fast to 90 %, slow for the last 10 % (at least 20 mL), stop early by the measured lead, count coast pulses, top up from below only.
3. Water chaser (FT-04) pushes the last colour out of the header into the tank.
4. Auto flush of the header to waste.

The first colour's target is reduced by the header volume (`trunk`), because that much is already waiting in the header from the prime and is pushed into the tank by the next stage. The chaser does the same for the last colour. Net result: exactly 2400 / 300 / 300 mL of colour in 3000 mL.

## Pin map (Populated from Schematic Sheet 1 & 2)

The pin mappings have been populated into `config.h` for both controllers based on the schematic:
- **ESP32 #1 (Actuator controller, U2)**:
  - Valves SV1..SV6 -> GPIO 23, 22, 21, 19, 18, 13
  - Pumps PWM_1..PWM_5 -> GPIO 32, 33, 25, 26, 27 (PWM_4 disabled)
  - Float switches FSW1..FSW4 -> GPIO 36, 39, 34, 35
  - Start switch SW -> Disabled (`PIN_SW = -1`, triggered via Serial Studio / CLI `start` or `batch`)
  - Serial link -> RX2 = GPIO 16, TX2 = GPIO 17
- **ESP32 #2 (Sensor controller, U7)**:
  - Flow sensors FS1..FS5 -> GPIO 33, 25, 26, 14, 27
  - Float switches FSW5..FSW6 -> GPIO 34, 35
  - Serial link -> RX2 = GPIO 16, TX2 = GPIO 17

## Batch Execution Modes (Serial Studio or USB Terminal @ 115200)

1. **`start`** or **`batch`** (Simultaneous 3-Line Parallel):
   - All 3 color lines (Red, Blue, Yellow) dose concurrently directly into the mixing tank.
   - **Red (300 mL)** and **Blue (300 mL)** operate at **equal PWM** (80% fast, 35% slow). When they reach their 300 mL threshold, both pumps and solenoids shut off at the exact same time.
   - **Yellow (2,400 mL)** operates until it reaches its 2,400 mL threshold, then shuts off.
   - Flush pump (Driver 2 M2) and valve (SV4) remain **completely disabled**.
   - When all thresholds are reached, the process completes and stops automatically.

2. **`start 2step`** or **`batch 2step`** (Power-Saving 2-Stage):
   - Stage 1: Yellow doses 2,400 mL alone. Turns off upon reaching 2400 mL.
   - Stage 2: Red and Blue dose simultaneously at equal PWM (300 mL each) and turn off together.
   - Automatically shuts off when complete.

3. **`dose rb 300`**:
   - Doses Red and Blue concurrently at equal PWM for 300 mL (dry-run test). Both turn off together.

4. **`x` or `stop`**:
   - Emergency stops all pumps and closes all valves immediately.

Bring-up checks (USB serial, 115200):
1. ESP32 #2 prints `[PCNT] FS1 counting on GPIO ...`. Spin each flow sensor by blowing through it: its total rises.
2. ESP32 #1 `status` shows `link=OK` and the counts follow. If `link=NONE`, swap `LINK_RX` / `LINK_TX`.
3. `valve 1 1` ... `valve 6 1`: each click matches its tag. `x` turns everything off.
4. `pump 1 40`: the right pump turns, in the right direction (wrong direction: swap the motor wires, or the DIR level on the PCB). `x` to stop.

## Calibration (water first, then each line again with paint)

Per line, using the real tubing and the final sensor orientation. Prime the line first; no bubbles.

1. `deadband <line> waste` : pump start duty. Pick a slow duty well above it. `duty <line> 100 <slow>`.
2. `sweep <line> waste` : duty vs flow table. Choose fast and slow duty (slow flow about 0.4-0.6 L/min, above the sensor's ~0.3 L/min floor).
3. K-factor: `dose <line> 500` into the mixing tank on a scale, then `meas <line> <actual_mL>` (grams / density; water = 1 g/mL). Repeat 3-5 times at 500 mL and again at about the real dose size. The pooled K updates after each `meas`.
4. Lead: `coast <line> <slow duty> waste`, then `lead <line> <value>`.
5. Repeatability check: 5 identical doses. Target: mean within 1 %, spread under 1 %.
6. `save`.
7. With paint in the tanks (stir first): repeat step 3 once or twice per line to trim K, then `save`.

Measure `trunk`: fill the header with water, blow it out into a cup, read the volume, then `trunk <mL>` and `save`.

## Day 1 flow

- Milestone 1 (dry run): water in all three colour tanks, calibrate, run `batch`, check each line's counted volume against its target.
- Milestone 2: paint in the tanks, trim K, run `batch` from the start switch, compare with the reference colour.

## Telemetry

`/*time_s,line,phase,target_mL,delivered_mL,duty_pct,flow_Lmin*/` every 200 ms during a dose. line: 0 red, 1 blue, 2 yellow, 3 flush. phase: 1 fast, 2 slow.

## Known limits

- Not run on hardware: pin map empty, defaults (K = 6.319 pulses/mL from Stage 2, duty 100 / 40 %, lead 3 mL) are placeholders.
- Temperature sensors (DS18B20) and colour sensors are not used in this version.
- Float switches are only used for the optional mixing-tank overfill guard (`USE_MIX_HIGH_FLOAT`).
