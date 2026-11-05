# Project Gripper Firmware

This PlatformIO project turns an ESP32 Dev Module, two Muscle Sensor v3-style EMG boards, and five MG996R servos into a **bench-first prototype** with:

- clean two-channel serial data collection for Edge Impulse;
- live EMG diagnostics and calibration;
- isolated individual-finger servo testing and per-finger travel calibration;
- a deliberately guarded manual or two-channel EMG gripper mode; and
- an optional, initially disabled, Edge Impulse inference hook.

> **Safety first:** This is maker/prototyping firmware, not a medical device and not a therapy prescription. An MG996R can exert injurious force. Keep tendons disconnected while calibrating, provide a physical servo-power emergency cutoff and mechanical quick release, and do not wear the mechanism until its motion limits have been independently validated by a qualified person.

## Direct answers to the hardware questions

The pictured red board appears to be the older Advancer Technologies **Muscle Sensor v3**. Its output is an amplified, rectified, smoothed muscle-activity envelope, not raw EMG. Its `SIG` output can range from 0 V to `+Vs`; its own manual specifies a split supply (`+Vs`, `GND`, `-Vs`) and shows the two-9-V-battery wiring. [Muscle Sensor v3 manual](https://www.pololu.com/file/0J745/Muscle_Sensor_v3_users_manual.pdf) · [output-voltage warning](https://www.pololu.com/product/2726)

One EMG channel is enough for a simple `relaxed` versus `contracted` switch. Two channels are the sensible minimum for a more natural control scheme: one forearm flexor envelope to close and one extensor envelope to open. However, **two surface-EMG channels cannot reliably identify all five fingers individually**: forearm muscles and tendons overlap and their signals cross-talk. Treat the first model as gesture classification. To target dependable individual-finger intent, expect to add more independent EMG channels and/or finger position/force sensors.

## Electrical wiring: do this before flashing

### 1. EMG power and ESP32 ADC protection

The two 9-V batteries can power the two EMG boards **only** as one split supply. They must not power the ESP32 or any servo.

```text
Battery A +  ------------------------> both EMG +Vs
Battery A - --+-- Battery B + --------> both EMG GND, ESP32 GND
              |                         (one quiet/star reference point)
Battery B -  ------------------------> both EMG -Vs
```

Each EMG module needs its own `SIG` connection, but the two boards may share the split-supply rails:

```text
EMG flex SIG   -- 33 kΩ --+--> ESP32 GPIO 34 (ADC1)
                            |
                            +-- 10 kΩ --> quiet GND
                            +-- 10 nF --> quiet GND  (optional, near ESP32)

EMG extend SIG -- 33 kΩ --+--> ESP32 GPIO 35 (ADC1)
                            |
                            +-- 10 kΩ --> quiet GND
                            +-- 10 nF --> quiet GND  (optional, near ESP32)
```

That 33 kΩ / 10 kΩ divider scales 9 V to about 2.1 V. It is a starting protection circuit, not a substitute for measurement. With a multimeter, confirm that each ADC-side signal stays between 0 V and 3.3 V before connecting it to GPIO 34 or 35. Never put the original `SIG` directly into an ESP32 pin while the module is powered from `+9 V`.

Use three proper electrodes per sensor. A cut headset cable can only be a cable if you continuity-map a **three-pole TRS** plug to the board's intended three conductors; do not trust wire colors, do not use a TRRS headset lead, and never use exposed solder or bare wire as a skin electrode. Disposable Ag/AgCl snap electrodes and an appropriate three-lead cable are much safer and more repeatable. Keep the body-side EMG setup battery-powered and isolated; do not use a mains bench supply or an AC-powered/charging computer USB connection while electrodes are attached.

### 2. Servo power and signal pins

| Finger | ESP32 signal GPIO |
| --- | ---: |
| Thumb | 15 |
| Index | 2 |
| Middle | 4 |
| Ring | 16 |
| Pinky | 17 |

Use a separate regulated **5–6 V high-current servo supply**. Do not use the ESP32's 5-V pin and do not use either 9-V EMG battery for an MG996R. Connect each servo red lead to servo `+`, brown/black to servo `-`, and the signal lead to the listed ESP32 pin. Connect servo `-`, ESP32 GND, and the EMG midpoint GND at a single quiet star point; keep high-current servo wiring physically away from the EMG leads.

An MG996R can draw roughly 2.5 A at 6 V when stalled, so five loaded servos call for a conservatively sized, fused supply (about 15 A peak capacity) and heavy wiring. Add bulk capacitance near the servo distribution. GPIO 2, 4, and 15 are ESP32 boot-strapping pins; this requested mapping is used by the firmware, but if boot/upload becomes unreliable, move those three signals to non-strap pins.

Flexible tendon wire transfers tension reliably; it should not be treated as a compression push rod. Use a spool/antagonistic tendon mechanism, physical end stops, and a quick mechanical release.

## Firmware behavior

- Boot is `SAFE`: no PWM signal is attached to a servo.
- `arm BENCH_ONLY` intentionally starts every servo at a neutral 1500 µs pulse. It is for a tendon-disconnected bench setup.
- Full gripper commands are rejected until every finger has distinct calibrated open and close limits.
- Motion is rate-limited, clamped to 1000–2000 µs, and a 120-second inactive session detaches all PWM outputs.
- `stop`, `safe`, and `disarm` detach every PWM output immediately. They cannot replace a physical emergency power cutoff.
- Starting data capture disarms servos, cancels an unfinished EMG calibration, and produces no debug text in Edge Impulse mode.
- The PWM implementation supports both the ESP32 Arduino Core 2 and Core 3 LEDC APIs, so PlatformIO upgrades do not require a servo-code rewrite.

The initial calibration values are intentionally neutral (1500 µs for both directions), so flashing the firmware cannot immediately pull a tendon.

## Code layout

The firmware is split by responsibility. Public declarations live under `include/`; implementations live in the matching `src/` folder.

```text
src/main.cpp                         Arduino setup()/loop() entry point
src/app/GripperApp.cpp               Run modes, sampling loop, capture, and control workflow
src/app/ConsoleOutput.cpp             Serial help, status, settings, and calibration output
src/app/CommandHandlers.cpp           EMG, servo, grip, auto-control, and model commands
src/app/SerialCommands.cpp            Serial input buffering and top-level command dispatch
src/hardware/EmgController.cpp       ADC sampling, filtering, normalization, calibration
src/hardware/ServoController.cpp     PWM attachment, pulse conversion, ramping, timeout
src/settings/SettingsStore.cpp       Validated ESP32 NVS calibration storage
src/ml/EdgeImpulseModel.cpp          Optional exported Edge Impulse model window and inference
src/core/TextUtils.cpp                Command parsing and finger-name parsing helpers
src/core/RunMode.cpp                  Readable names for the firmware modes
```

For most changes, start in the matching header under `include/`. Pin assignments, limits, sample rate, and feature switches stay in `include/GripperConfig.h`; `main.cpp` should normally remain unchanged.

## Build and flash

Open this folder in VS Code with PlatformIO, or run:

```powershell
& 'C:\Users\csrt_2\.platformio\penv\Scripts\platformio.exe' run
& 'C:\Users\csrt_2\.platformio\penv\Scripts\platformio.exe' run --target upload
```

Open the serial monitor at **115200 baud**. The project has no third-party library dependency.

## First safe commissioning sequence

1. Build and flash with **servo power off**, tendons disconnected, and no electrodes attached.
2. Power only the ESP32. Open the monitor and send `status`; it should say `servos=DISARMED`.
3. Power the EMG boards through the split supply, validate their ADC-divider voltages with a meter, then use `emg start` to see both ADC readings.
4. With no tendon connected, power the servo rail. Send `arm BENCH_ONLY`, then use small `servo pulse` steps to find each servo's safe physical range. Stop before any mechanical limit.
5. Record each safe limit and save it. Example for the thumb only:

   ```text
   servo pulse thumb 1450
   servo set-open thumb 1450
   servo pulse thumb 1580
   servo set-close thumb 1580
   settings save
   disarm
   ```

   Repeat for all five fingers. “Open” and “close” describe the mechanism, not a numerical pulse direction; the firmware supports either direction.

6. With servos disarmed, fit electrodes and run:

   ```text
   emg cal idle 5
   emg cal max 5
   settings save
   ```

   Relax during `idle`; during `max`, make repeated controlled contractions of the relevant muscle. Inspect `emg start` output before enabling any control mode.

7. Only after all of that bench validation, test `grip open`, `grip close`, then `auto emg on` at the slow default slew rate. The system must be mechanically fail-safe before body-mounted testing.

## Serial command reference

| Command | Purpose |
| --- | --- |
| `help`, `status` | List commands / show live state and settings. |
| `arm BENCH_ONLY` | Attach servo PWM at neutral. Requires an intentional phrase. |
| `disarm`, `stop`, `safe` | Detach every servo PWM output. |
| `emg start`, `emg stop` | Readable two-channel EMG debug stream. |
| `emg cal idle 5`, `emg cal max 5` | Baseline and maximum EMG calibration. |
| `ei start`, `ei stop` | Headerless `flex,extend` samples at 200 Hz for Edge Impulse. |
| `csv start`, `csv stop` | `timestamp,emg_env_flex,emg_env_extend` capture for CSV upload. |
| `servo pulse index 1500` | Test one unloaded servo at a pulse width. |
| `servo set-open index 1450` | Save a runtime open limit; use `settings save` to persist it. |
| `servo set-close index 1580` | Save a runtime close limit; use `settings save` to persist it. |
| `servo open index`, `servo close index` | Slowly move a single calibrated finger. |
| `grip open`, `grip close` | Slowly move all calibrated fingers. |
| `grip all 50` | Set all fingers to 50% of their individual calibrated travel. |
| `grip finger index 25` | Set one finger's calibrated travel percentage. |
| `auto emg on`, `auto emg off` | Basic two-channel flexor/extensor gripper control. |
| `settings show`, `settings save`, `settings factory RESET` | Inspect, persist, or erase configuration. |
| `model status` | Inspect optional Edge Impulse inference support. |

## Install the Edge Impulse CLI and forwarder (Windows)

The forwarder runs on the computer, not on the ESP32. Install a current Node.js LTS release first; the Edge Impulse CLI currently requires Node.js 16 or later. Then open a fresh PowerShell window and run:

```powershell
node --version
npm --version
npm install -g edge-impulse-cli
edge-impulse-data-forwarder --help
```

If the last command is not found, close and reopen PowerShell so the npm global-bin directory is added to `PATH`. Do **not** run the forwarder while PlatformIO's serial monitor is open: only one program can own the ESP32 serial port at a time. The CLI package and its installation command are maintained by Edge Impulse. [CLI package](https://www.npmjs.com/package/edge-impulse-cli)

## Edge Impulse collection workflow

The default firmware emits **the same unfiltered ADC values** in its training stream and optional on-device model hook. Keep the sampling frequency at 200 Hz and train with exactly two axes in this order:

```text
emg_env_flex, emg_env_extend
```

Do not mix diagnostics into a capture. First close PlatformIO's serial monitor so it releases the port, then run:

```text
edge-impulse-data-forwarder --frequency 200 --start-msg
```

The firmware recognizes the forwarder's `AT+STARTDATAFWD` / `AT+STOPDATAFWD` commands and emits one comma-separated numeric line per sample at 115200 baud. Name the two detected axes `emg_env_flex,emg_env_extend`. The official data-forwarder protocol expects exactly this one-line, comma/tab-separated format; the `--start-msg` option emits the start and stop messages automatically. [Edge Impulse data-forwarder documentation](https://github.com/edgeimpulse/edge-impulse-cli/blob/master/README-data-forwarder.md)

For manual logging instead, send `csv start`, save everything from its header through `csv stop` to a `.csv`, then upload it. Time-series CSV imports require timestamp as the first column, which this firmware provides. [Edge Impulse CSV collection format](https://github.com/edgeimpulse/tool-data-collection-csv)

Collect separate, balanced recordings labelled `rest`, `open`, `close`, and any individual-finger gesture you want to investigate. Include sessions with slight electrode repositioning and fatigue, then hold out entire recording sessions for testing. A high random-split score alone is not evidence that individual fingers are being recognized.

## Optional on-device Edge Impulse inference

Data collection works immediately. Once you have a model that is genuinely accurate on unseen sessions:

1. Export its C++/Arduino library from Edge Impulse and place it in `lib/edge_impulse` so it contains `edge-impulse-sdk/classifier/ei_run_classifier.h`.
2. Change `GRIPPER_ENABLE_EDGE_IMPULSE` in [`include/GripperConfig.h`](include/GripperConfig.h) from `0` to `1`.
3. Build, flash, and run `model start`. This only prints the top label and confidence; servos stay disarmed.
4. Keep `kEnableModelActuation` set to `false` until exhaustive tendon-disconnected validation. If you deliberately enable it later, `model control` still requires explicit arming, per-finger calibration, >=0.85 confidence, repeated matching labels, and a command dwell time.

The expected label mapping is `open`/`rest` for all-open, `close`/`grip` for all-close, and labels starting with a finger name (such as `index_close`) for that finger. Change the mapping in `src/main.cpp` only after proving that the classifier is reliable.
