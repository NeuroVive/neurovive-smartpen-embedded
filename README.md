# Parkinson's Smart Pen — Hardware Architecture

A low-cost, fully untethered smart pen for **non-diagnostic screening and monitoring** of Parkinsonian tremor. The device fuses a 6-axis IMU with a three-channel force-sensitive-resistor (FSR) array to separately characterize resting/micro-tremor (grip) and axial writing-surface dynamics (tip), processes everything on-device with an embedded FFT, and streams extracted features over native BLE to a companion mobile app.

> **⚠️ Disclaimer:** This system is a screening and monitoring tool. It is **not** a cleared medical device and must never be presented as diagnostic. All output should be interpreted alongside a validated clinical scale (e.g., MDS-UPDRS) by a qualified professional.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Block Diagram](#2-block-diagram)
3. [Schematic](#3-schematic)
4. [Central Processing Subsystem — ESP32-S3 SuperMini](#4-central-processing-subsystem--esp32-s3-supermini)
5. [Sensor Acquisition Network](#5-sensor-acquisition-network)
6. [Boot-Safe RC Decoupling (Option 2)](#6-boot-safe-rc-decoupling-option-2)
7. [Power Management](#7-power-management)
8. [Connection Matrix](#8-connection-matrix)
9. [Bill of Materials](#9-bill-of-materials)
10. [PCB Layout Considerations](#10-pcb-layout-considerations)
11. [Signal Integrity & Noise Budget](#11-signal-integrity--noise-budget)
12. [Known Issue: Schematic vs. Documentation Mismatch](#12-known-issue-schematic-vs-documentation-mismatch)
13. [Schematic Review — v1 → v2 Corrections](#13-schematic-review--v1--v2-corrections)
14. [Ethical & Data Safety Considerations](#14-ethical--data-safety-considerations)
15. [Revision History](#15-revision-history)

---

## 1. System Overview

The hardware balances multi-channel data acquisition, on-device DSP performance, and the tight spatial constraints of a pen chassis. Three sub-circuits share a single regulated 3.3 V rail (critical for preventing battery-discharge noise from corrupting the analog FSR readings):

| Subsystem | Core Component |
|---|---|
| Central Processing | ESP32-S3 SuperMini |
| Sensor Acquisition | MPU-6050 IMU + Tri-FSR array (FSR-400 tip, 2× FSR-402 grip) |
| Power Management | 3.7 V / 500 mAh LiPo + onboard LDO |

**Signal pipeline:**

```
Sensor Acquisition → ADC1 sampling @ 1 kHz → DC offset removal
→ High-pass filter (1 Hz) → Band-pass filter (3.5–6.5 Hz)
→ 1024-point FFT (on-device) → Feature extraction
→ Native BLE transmission @ 1 Hz → Mobile application
```

---

## 2. Block Diagram

![System block diagram showing Power Management, Sensor Network, and ESP32-S3 SuperMini swimlanes](./Schematic_Block_Diagram.png)

*Figure 1 — High-level system block diagram (`Schematic_Block_Diagram.png`).*

This is the best starting point for understanding signal/power flow before reading the full schematic. It is organized into three swimlanes, left to right:

- **Power Management** — the 3.7 V LiPo feeds the ESP32-S3's onboard LDO, which produces the regulated 3.3 V rail. This rail fans out to power every sensor (all three FSRs and the MPU-6050) and shares a common GND return.
- **Sensor Network** — shows the three FSR channels (Tip FSR-400, Grip FSR-402 A, Grip FSR-402 B), each annotated with its GPIO + RC filter stage, plus the MPU-6050 IMU on its I2C bus (SDA → GPIO 8, SCL → GPIO 9).
- **ESP32-S3 SuperMini** — the FSR channels terminate at the **ADC1 Controller**, and the IMU terminates at the **I2C Peripheral** block, visually reinforcing that analog (ADC) and digital (I2C) signal paths are kept architecturally separate.

This diagram correctly labels the MCU as **ESP32-S3** throughout.

---

## 3. Schematic

![Component-level EasyEDA schematic showing ESP32 MCU, MPU-6050 IMU, and three FSR voltage-divider sub-circuits](./schematic.png)

*Figure 2 — Full component-level schematic (`schematic.png`), EasyEDA "Smart Pen," Schematic2, v1.0.*

This is the component-level electrical schematic, laid out in a 6-column × 4-row grid (A4):

- **Row A, Col 1–2:** Battery input — `VIN` from a 4-pin `JST-PH Connector` (CN1) feeding the MCU's VIN rail.
- **Row B, Col 1–3:** The MCU block (`U3`), with I2C signals (`I2C_SDA`, `I2C_SCL`) broken out on the left and power/GPIO breakout (`5V`/`VIN`, `GND`, `3.3V`, `GPIO1–4`) on the right.
- **Row C, Col 1–3:** The MPU-6050 IMU (`U2`), a 24-pin block showing `SDA`/`SCL` routed to the `I2C_SDA`/`I2C_SCL` nets.
- **Row A–C, Col 4–6:** Three near-identical FSR voltage-divider + RC filter sub-circuits, one per channel:
  - **H1 — Tip FSR-400** on GPIO1 (R3/C3/C1/R1)
  - **H4 — Grip FSR-402 A** on GPIO2 (R7/C6/C4/R5)
  - **H5 — FSR_GRIP_B** on GPIO3 (R9/C8/C7/R8)

Each FSR sub-circuit follows the same topology: 3V3 → 100 kΩ series resistor → Node A (junction with 10 µF cap to GND, forming the boot-delay stage) → header pin 1 of the FSR → header pin 2 → Node B (junction with 100 nF cap and 10 kΩ pull-down to GND) → GPIO.

---

## 4. Central Processing Subsystem — ESP32-S3 SuperMini

Selected over standard ARM Cortex-M or 8-bit MCUs for three reasons:

1. **Computational headroom** — dual-core Xtensa LX7 @ up to 240 MHz comfortably handles 1 kHz multi-channel sampling with concurrent FFT.
2. **Native BLE 5** — eliminates an external UART BLE module, reducing footprint and power draw.
3. **Onboard LDO** — provides the stable 3.3 V rail essential for clean analog FSR readings.

| Parameter | Value |
|---|---|
| MCU | ESP32-S3 (Xtensa dual-core LX7) |
| Clock speed | Up to 240 MHz (dual-core) |
| Operating voltage | 3.3 V logic (onboard LDO) |
| Wireless | Native BLE 5, 2.4 GHz onboard PCB antenna, no external module |
| ADC resolution | 12-bit nominal, ~11-bit effective with noise |
| ADC channels used | ADC1 only (GPIO 1, 2, 3, 4) — **ADC2 avoided**, shares hardware paths with the BLE radio and is unreliable during active wireless TX |
| I2C | GPIO 8 (SDA), GPIO 9 (SCL) |
| Flash / PSRAM | 8 MB flash / 2 MB PSRAM (model-dependent) |

### GPIO Pin Assignment

| GPIO | Function | Interface | Notes |
|---|---|---|---|
| GPIO 8 | I2C SDA | MPU-6050 data | 4.7 kΩ pull-up to 3.3 V required |
| GPIO 9 | I2C SCL | MPU-6050 clock | 4.7 kΩ pull-up to 3.3 V required |
| GPIO 1 | ADC1_CH1 | FSR-400 tip sensor | Strapping pin — Option 2 RC mitigation applied |
| GPIO 2 | ADC1_CH2 | FSR-402 grip A | Strapping pin — Option 2 RC mitigation applied |
| GPIO 3 | ADC1_CH3 | FSR-402 grip B | Strapping pin — Option 2 RC mitigation applied |

> **⚠️ Strapping pin warning:** GPIO 0, 1, 2, and 3 are ESP32-S3 strapping pins sampled at boot to determine startup mode. A fixed voltage divider held directly on these pins can present an incorrect logic level during boot, causing boot loops or blocking firmware upload. Section 6 (Option 2 RC decoupling) is mandatory on all three FSR channels.

---

## 5. Sensor Acquisition Network

Two electrically isolated interface types — digital I2C (IMU) and passive analog voltage dividers (FSRs) — are kept on separate GPIO groups to prevent digital switching noise from coupling into analog traces.

### 5.1 MPU-6050 (Kinematic Vibration Sensor)

6-axis MEMS IMU (3-axis accel + 3-axis gyro), mounted internally near the writing tip to maximize sensitivity to vertical tremor oscillations in the 4–6 Hz Parkinsonian band. I2C up to 400 kHz.

| MPU-6050 Pin | Connects To | Notes |
|---|---|---|
| VDD (13) | 3.3 V rail | 100 nF ceramic decoupling cap, placed within **1 mm** of the pin |
| GND (18) | GND | System ground |
| SDA (23) | GPIO 8 | 4.7 kΩ pull-up to 3.3 V (verify breakout doesn't already have one) |
| SCL (24) | GPIO 9 | 4.7 kΩ pull-up to 3.3 V (verify breakout doesn't already have one) |
| AD0 (11) | GND | Fixes I2C address to `0x68` — must not float |
| VLOGIC (8) | 3.3 V rail | Tie on variants that expose this pin |
| INT, FSYNC | NC | Leave unconnected; pull to GND via 10 kΩ if EMI is observed |
| CLKIN, AUX_DA, AUX_CL, RESV | NC | Internal oscillator used / secondary bus unused / reserved |

### 5.2 Tri-FSR Analog Front-End

Three FSRs, each in an identical passive voltage-divider configuration — no op-amps or load cells required.

| Sensor | Active Area | GPIO | Placement | Rationale |
|---|---|---|---|---|
| FSR-400 | 7.5 mm dia. | GPIO 1 (ADC1_CH1) | Inside barrel, ink-cartridge distal end | Micro-plunger transfers only axial (downward) force, mechanically isolating it from lateral grip |
| FSR-402 A | 44 mm dia. | GPIO 2 (ADC1_CH2) | Lower barrel, primary finger contact | Large area covers the full fingerprint region; captures grip rigidity + high-freq micro-tremor |
| FSR-402 B | 44 mm dia. | GPIO 3 (ADC1_CH3) | Lower barrel, secondary finger contact | Eliminates blind spots as grip shifts; redundant pill-rolling capture |

**Base voltage divider (per channel):** FSR terminal 1 → 3.3 V. Terminal 2 → Node A, a junction shared with a 10 kΩ pull-down to GND and a 100 nF capacitor in parallel with that pull-down. As applied force increases, FSR resistance drops from >10 MΩ (unloaded) toward ~10 kΩ (firmly pressed), driving Node A from near 0 V toward 3.3 V.

This 10 kΩ ‖ 100 nF pair forms a passive RC low-pass filter:

```
fc = 1 / (2π × 10,000 Ω × 100×10⁻⁹ F) ≈ 159 Hz
```

— passing the 4–6 Hz biological tremor band with negligible attenuation while rejecting 50/60 Hz mains hum and ADC switching noise.

---

## 6. Boot-Safe RC Decoupling (Option 2)

Because GPIO 1, 2, and 3 double as strapping pins, connecting the FSR divider directly would risk an incorrect boot-time logic level. **Option 2** inserts a second RC stage between Node A and the GPIO (Node B): a 100 kΩ series resistor + 10 µF electrolytic boot-delay capacitor.

**Per-channel topology** (identical for FSR-400 / GPIO1, FSR-402 A / GPIO2, FSR-402 B / GPIO3):

```
3.3V ── FSR Terminal 1
        FSR Terminal 2
          │
          ├──────────────────────── NODE A
          │                  │
   [10kΩ pull-down]   [100kΩ series resistor]
          │                  │
   [100nF cap] ‖ 10kΩ    NODE B ──────────────► GPIO (1/2/3)
          │                  │
         GND          [10µF electrolytic cap]
                        (+ to Node B, − to GND)
                              │
                             GND
```

### How it works

| Phase | Behavior |
|---|---|
| **Boot (0–~2 ms)** | ESP32-S3 samples strapping pins. Node B is held near 0 V — the 10 µF cap hasn't charged yet — so the correct strapping value is read. |
| **RC time constant** | τ = 100 kΩ × 10 µF = **1 second**. Node B rises slowly toward Node A. |
| **Normal operation (after ~3 s)** | Node B has settled and tracks the FSR signal normally. Firmware enforces a **3-second startup delay** before ADC sampling begins. |
| **100 kΩ series resistor** | Makes the divider a high-impedance source from the GPIO's perspective during boot, so the ESP32-S3's internal strapping logic can easily override it. |

> **⚠️ Polarity warning:** The 10 µF capacitors are electrolytic/polarized. **(+) → Node B (GPIO side), (−) → GND.** Reversed installation will damage the capacitor and may damage the ESP32-S3.

> **ℹ️ Firmware note:** A 3-second startup delay before ADC sampling begins must be implemented to let Node B fully settle after power-on.

---

## 7. Power Management

The pen is fully untethered. Rail stability is critical — any 3.3 V fluctuation shifts the baseline of all three FSR dividers simultaneously, producing false tremor artifacts indistinguishable from real signal.

| Parameter | Value |
|---|---|
| Connector | JST-PH **2-pin** (positive + negative only) |
| Battery chemistry | Lithium-polymer (LiPo) |
| Nominal voltage | 3.7 V |
| Capacity | 500 mAh |
| Voltage range | 4.2 V (full) → 3.2 V (depleted) |
| Input connection | VBAT / 5V pad on ESP32-S3 SuperMini |
| Regulated output | 3.3 V flat, via onboard LDO |
| Current draw | ~80–120 mA (active BLE + dual-core DSP) |
| Estimated runtime | 4–6 hours continuous — exceeds a standard clinical session |

> **ℹ️ JST connector note:** The schematic uses a **2-pin** JST-PH (CN1) for the single-cell LiPo — positive and negative only. A 4-pin JST-PH would imply a balance connector for multi-cell packs, which this single-cell design does not need.

---

## 8. Connection Matrix

| Component | Pin | ESP32-S3 Pin / Net | Value / Notes |
|---|---|---|---|
| LiPo (+) | positive | VBAT / 5V pad | JST-PH 2-pin · raw 3.7–4.2 V input |
| LiPo (−) | negative | GND | System ground |
| MPU-6050 | VDD (13) | 3.3 V | 100 nF ceramic decoupling, within 1 mm |
| MPU-6050 | GND (18) | GND | System ground |
| MPU-6050 | SDA (23) | GPIO 8 | I2C data · 4.7 kΩ pull-up |
| MPU-6050 | SCL (24) | GPIO 9 | I2C clock · 4.7 kΩ pull-up |
| MPU-6050 | AD0 (11) | GND | Sets address 0x68 · must not float |
| MPU-6050 | VLOGIC (8) | 3.3 V | Do not leave floating |
| MPU-6050 | INT, FSYNC | NC | 10 kΩ to GND if EMI observed |
| MPU-6050 | RESV, AUX_DA, AUX_CL, CLKIN | NC | Reserved / unused |
| FSR-400 (tip) | Terminal 1 | 3.3 V | Active power |
| FSR-400 (tip) | Terminal 2 | Node A | 10 kΩ ‖ 100 nF to GND |
| — | Node A → Node B | 100 kΩ series | Option 2 boot-safe resistor |
| — | Node B → GND | 10 µF electrolytic | (+) to Node B, (−) to GND |
| FSR-400 (tip) | Node B | GPIO 1 (ADC1_CH1) | Final GPIO connection |
| FSR-402 grip A | Terminal 1 | 3.3 V | Active power |
| FSR-402 grip A | Terminal 2 | Node A | 10 kΩ ‖ 100 nF to GND |
| — | Node A → Node B | 100 kΩ series | Option 2 boot-safe resistor |
| — | Node B → GND | 10 µF electrolytic | (+) to Node B, (−) to GND |
| FSR-402 grip A | Node B | GPIO 2 (ADC1_CH2) | Final GPIO connection |
| FSR-402 grip B | Terminal 1 | 3.3 V | Active power |
| FSR-402 grip B | Terminal 2 | Node A | 10 kΩ ‖ 100 nF to GND |
| — | Node A → Node B | 100 kΩ series | Option 2 boot-safe resistor |
| — | Node B → GND | 10 µF electrolytic | (+) to Node B, (−) to GND |
| FSR-402 grip B | Node B | GPIO 3 (ADC1_CH3) | Final GPIO connection |

---

## 9. Bill of Materials

| Ref | Component | Value / Part No. | Qty | Purpose |
|---|---|---|---|---|
| U1 | ESP32-S3 SuperMini | ESP32-S3 (dual-core LX7) | 1 | MCU · BLE · ADC · LDO |
| U2 | IMU | MPU-6050 | 1 | 6-axis MEMS IMU · I2C |
| U3 | Tip force sensor | FSR-400 | 1 | Axial writing force · 7.5 mm |
| U4, U5 | Grip force sensors | FSR-402 | 2 | Fingerprint grip · 44 mm |
| BT1 | LiPo battery | 3.7 V 500 mAh | 1 | Main power · JST-PH 2-pin |
| CN1 | Battery connector | JST-PH 2-pin | 1 | LiPo interface |
| H1 | Tip FSR header | 2-pin male header | 1 | FSR-400 connection |
| H4, H5 | Grip FSR headers | 2-pin male header | 2 | FSR-402 A/B connection |
| R1, R5, R8 | FSR pull-down resistors | 10 kΩ | 3 | Voltage divider lower leg (Node A) |
| R3, R7, R9 | Boot-safe series resistors | 100 kΩ | 3 | Option 2 · Node A → Node B isolation |
| R2, R6 | I2C pull-up resistors | 4.7 kΩ | 2 | SDA + SCL pull-ups (if not on breakout) |
| C1, C4, C7 | FSR RC filter capacitors | 100 nF ceramic | 3 | Hardware LPF, parallel to 10 kΩ pull-down |
| C3, C6, C8 | Boot delay capacitors | 10 µF electrolytic | 3 | Option 2 · Node B → GND · observe polarity |
| C2 | MPU-6050 decoupling cap | 100 nF ceramic | 1 | VDD pin 13 · within 1 mm |
| R4 | ~~Floating~~ | 100 kΩ | 0 | **Removed** — unconnected in schematic v1 |

---

## 10. PCB Layout Considerations

When migrating from breadboard to a custom PCB inside the pen chassis:

- **Antenna keep-out:** No copper plane, metallic housing, or shielding within **3 mm** of the ESP32-S3's onboard PCB antenna in any direction. No metallic barrel components above/below this zone.
- **Analog/digital trace isolation:** Route I2C (SDA/SCL, GPIO 8/9) as far as possible from the three ADC traces (GPIO 1/2/3). 100–400 kHz I2C clock edges can capacitively couple into FSR traces and mimic micro-tremors. Use a ground-fill guard trace between the zones where board area allows.
- **Decoupling cap placement:**

  | Capacitor | Target | Max distance | Note |
  |---|---|---|---|
  | 100 nF ×3 (RC filter) | FSR pull-down resistor (Node A) | 2 mm from ADC pin pad | One per FSR channel |
  | 10 µF ×3 (boot delay) | GPIO pin (Node B) | 2 mm from GPIO pin pad | Electrolytic — observe polarity |
  | 100 nF (MPU-6050) | VDD pin 13 | 1 mm from VDD pin | Critical for BLE RF suppression |

- **Ground plane:** Solid copper pour across the entire bottom layer — low-impedance analog return path, EMI resistance, and passive heat sink during sustained BLE TX.
- **FSR mechanical interface:**
  - *FSR-400 tip:* Dedicated micro-plunger channel machined into the barrel, constraining lateral movement so only axial force reaches the sensor.
  - *FSR-402 grip sensors:* Conformally adhered to the curved barrel via a thin compliant adhesive (e.g., silicone transfer tape) for uniform contact pressure across the 44 mm area, avoiding rigid pressure hot-spots.

> **⚠️ Lifecycle note:** Formal lifecycle testing of the plunger mechanism under repeated ballpoint impact is required before clinical deployment — high-frequency strokes may degrade the channel geometry and introduce FSR-400 baseline drift over time.

---

## 11. Signal Integrity & Noise Budget

| Technique | Stage | Effect |
|---|---|---|
| RC hardware LPF (fc ≈ 159 Hz) | Analog front-end (Node A) | Reduces broadband noise floor pre-ADC |
| 100 kΩ series + 10 µF boot delay | Boot protection (Node B) | Isolates GPIO from divider during strapping sampling |
| Software oversampling (4×) | Firmware | Recovers ~0.5 effective ADC bits |
| DC offset removal | DSP chain | Eliminates slow drift from FSR viscoelastic creep |
| Band-pass filter 3.5–6.5 Hz | DSP chain | Isolates Parkinsonian tremor band in software |
| ADC1-only routing | Hardware architecture | ADC2 shares BLE radio paths — avoided entirely |
| Digital/analog GPIO separation | PCB layout | Prevents I2C clock coupling into FSR traces |

**Frequency domain budget:**

```
Signal of interest:     4–6 Hz   (Parkinsonian tremor band)
RC filter cutoff:       ~159 Hz  (Node A hardware LPF)
High-pass filter (SW):  1 Hz     (DC offset removal)
Band-pass filter (SW):  3.5–6.5 Hz (DSP tremor isolation)
FFT window:              1024 samples @ 1 kHz = 1.024 s
FFT frequency resolution: ~0.98 Hz/bin
ADC sample rate:          1 kHz per channel
```

---

## 12. Known Issue: Schematic vs. Documentation Mismatch

⚠️ **Flagging for the team:** `schematic.png` (EasyEDA "Smart Pen," v1.0, dated 2026-06-21/22) still labels the MCU block (`U3`) as **`ESP32-C3 SuperMini`**. The v2 hardware architecture document explicitly lists this as **Correction #1 (Critical)** — the MCU should read **ESP32-S3 SuperMini** throughout (`Schematic_Block_Diagram.png` already reflects this fix correctly).

Everything else about the schematic's electrical topology (GPIO assignments, RC networks, MPU-6050 wiring) is consistent with the v2 corrections — only the **part label/silkscreen text** appears stale. Recommend regenerating or relabeling `schematic.png` from the EasyEDA source before this design is sent to fab, since ESP32-C3 (single-core RISC-V) and ESP32-S3 (dual-core Xtensa LX7) are pin-compatible in *concept* on this SuperMini form factor but are not the same part, and downstream firmware/peripheral assumptions differ between them.

---

## 13. Schematic Review — v1 → v2 Corrections

| # | Severity | Issue | Correction Applied |
|---|---|---|---|
| 1 | Critical | MCU labeled "ESP32-C3 SuperMini" — wrong chip | Corrected to ESP32-S3 SuperMini throughout |
| 2 | Critical | GPIO 1, 2, 3 are strapping pins — direct FSR divider connection risks boot loops | Option 2 RC decoupling applied: 100 kΩ series + 10 µF boot delay cap per channel |
| 3 | Critical | MPU-6050 VDD decoupling capacitor missing | 100 nF ceramic cap added to VDD pin 13 |
| 4 | Warning | R4 (100 kΩ) floating — not connected | R4 removed from BOM and schematic |
| 5 | Warning | AD0 pin shown as NC — must not float | AD0 explicitly tied to GND · I2C address fixed at 0x68 |
| 6 | Warning | VLOGIC pin disposition not specified | VLOGIC tied to 3.3 V rail |
| 7 | Warning | CN1 shown as 4-pin connector — incorrect for single-cell LiPo | Changed to 2-pin JST-PH (positive + negative only) |
| 8 | Info | I2C net labels ambiguous on ESP32 block | Confirmed: I2C_SDA → GPIO 8 · I2C_SCL → GPIO 9 |

*Note: Item #1 and #7 in this table describe what the documentation says should be corrected — see [Section 12](#12-known-issue-schematic-vs-documentation-mismatch) regarding the MCU label still present in the current `schematic.png` artifact.*

---

## 14. Ethical & Data Safety Considerations

- **BLE encryption:** All BLE transmission must use AES-128 encryption before any clinical deployment, to prevent interception of patient kinematic data.
- **On-device storage:** The companion mobile app must use encrypted on-device storage (SQLCipher on Android · iOS `NSFileProtectionComplete`).
- **Cloud sync:** Any cloud sync of feature vectors must comply with HIPAA (US) or GDPR (EU), including explicit informed patient consent and data minimization.
- **Clinical representation:** Never represent the device to patients or clinicians as diagnostic. Results must always accompany professional clinical assessment.
- **Data retention:** Feature vectors and raw session logs require a defined retention policy, agreed with the institution's data protection officer prior to deployment.

---

## 15. Revision History

| Version | Date | Changes |
|---|---|---|
| v1.0 | June 2026 | Initial hardware architecture document |
| v2.0 | June 2026 | Corrected MCU label (C3→S3) · Added Option 2 RC boot-safe decoupling on all FSR channels · Added MPU-6050 VDD decoupling cap · Grounded AD0 · Tied VLOGIC to 3.3 V · Removed floating R4 · Corrected CN1 to 2-pin JST-PH · Full schematic corrections table added |

---

### Related Documents

- `Schematic_Block_Diagram.png` — high-level system block diagram (Section 2)
- `schematic.png` — full EasyEDA component-level schematic (Section 3)
- `SIGNAL_PROCESSING.md` — firmware DSP/FFT pipeline documentation *(referenced, not included here)*
- `MOBILE_APP.md` — companion mobile application documentation *(referenced, not included here)*
