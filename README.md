# 🚀 Catch the Rocket — Team ByteForce

> **FCA Mid-Semester Hackathon | Autonomous Ground Station on ESP32**

Team ByteForce built an autonomous, real-time rocket ground station on a physical **ESP32** that processes a 100,000-sample altitude stream under strict memory, timing, accuracy, and robustness constraints.

The system was developed incrementally from **Level 0 to Level 5**, with mathematical derivation, embedded implementation, hardware measurement, numerical-stability checks, and verification against both the Official Tracker and Practice Tracker B.

---

## 🛰️ Project Overview

The challenge is to process **100,000 altitude readings** representing a 100-second simulated rocket flight.

The ESP32 must:

- Process every reading within **12,000 CPU cycles**
- Keep all persistent reading-derived state inside **one struct ≤ 4096 bytes**
- Avoid flash storage, SPIFFS, NVS, Wi-Fi and Bluetooth
- Avoid hard-coding flight parameters
- Estimate:
  - Gravity
  - Apogee time and altitude
  - Parachute deployment time and altitude
  - Descent velocity
  - Landing time
  - Altitude at arbitrary time `ALT t`
  - Number of sensor glitches
- Continue running safely for malformed serial commands

The final firmware satisfies the constraints on physical ESP32 hardware.

---

## ⚙️ Hardware

| Component | Specification |
|---|---|
| Microcontroller | ESP32 Dev Module |
| CPU | Xtensa LX6 |
| Clock | 240 MHz |
| SRAM | 520 KB |
| Serial Monitor | 115200 baud |
| Persistent project state | 2336 B / 4096 B |
| Worst-case reading | 6502 cycles / 12000 cycles |

---

## 🧠 Core Engineering Idea

The key design principle was:

> **Don't store the history. Store the information extracted from the history.**

Instead of attempting to store 100,000 altitude readings, the firmware maintains compact mathematical summaries, regression accumulators, a quantized ring buffer, and cached model coefficients.

This makes the solution feasible within the 4 KB persistent-state limit.

---

# 🏗️ System Architecture

```text
                 ┌─────────────────────┐
                 │  nextAltitude()     │
                 │  100,000 readings   │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │  Causal 5-Point     │
                 │  Median Filter      │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │  Glitch Detection   │
                 │  >20 m deviation    │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │ Compressed State /  │
                 │ Running Accumulators│
                 └──────────┬──────────┘
                            │
             ┌──────────────┴──────────────┐
             ▼                             ▼
    ┌──────────────────┐         ┌──────────────────┐
    │ Coast Parabola   │         │ Descent Line     │
    │ h = A+Bq+Cq²     │         │ h = A+Bk         │
    └────────┬─────────┘         └────────┬─────────┘
             │                            │
             └─────────────┬──────────────┘
                           ▼
                 ┌─────────────────────┐
                 │ Deployment Trigger  │
                 │ Actual vs Predicted │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │ Cache Final Model   │
                 │ Coefficients        │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │ Serial Commands     │
                 │ G / APOGEE / DEPLOY │
                 │ DESCENT / LAND      │
                 │ ALT / GLITCHES      │
                 │ STATS               │
                 └─────────────────────┘
```

---

# 📈 Six Development Levels

## Level 0 — Bring-Up

Established the ESP32 processing loop, serial command parser and cycle-count measurement.

**Result:**

- 100,000 / 100,000 readings processed
- State: **44 B**
- Worst reading: **903 cycles**

---

## Level 1 — Descent and Landing

The parachute phase is modeled as a straight line:

```text
y = A + Bk
```

A centered time coordinate is used:

```text
k = i - 72500
```

Only running regression quantities are stored rather than raw readings.

From the fitted line:

```text
vD = 10B
k_land = -A/B
```

This provides descent velocity and predicted touchdown time.

**Official Tracker:**

- Descent: **−4.500 m/s**
- Landing: **133.709 s**
- State: **88 B**
- Worst reading: **992 cycles**

---

## Level 2 — Gravity and Apogee

The initial ballistic coast is modeled as a parabola:

```text
y = A + Bq + Cq²
```

with centered coordinate:

```text
q = 2i - 7999
```

Centering the coordinate makes the odd power sums vanish and improves numerical stability.

Gravity is obtained from:

```text
g = -80000C
```

The apogee occurs where:

```text
dy/dq = 0
```

therefore:

```text
q_apogee = -B/(2C)
```

The implementation uses **int64 accumulators** for large exact sums and **double precision** where division and coefficient calculation require it.

**Official Tracker:**

- Gravity: **9.814 m/s²**
- Apogee: **8.001 s / 626.07 m**
- State: **136 B**
- Worst reading: **2404 cycles**

---

## Level 3 — Deployment and ALT Queries

An intermediate block-compression approach represented the stream using:

- 100 blocks
- 1000 samples per block
- Three moments per block

This required approximately **2000 bytes** for the block representation.

Deployment was found by intersecting:

```text
Coast parabola P(t)
        ×
Descent line L(t)
```

The quadratic produces two roots. The **later physical root** is selected as the parachute deployment point because apogee must occur before deployment.

The final Level 4/5 implementation replaced the block representation with a running-accumulator architecture and quantized ring buffer.

---

## Level 4 — Glitch Rejection and Real-Time Deployment

The tracker can introduce glitches between **50 m and 300 m**, while normal sensor noise is limited to approximately **±2 m**.

A causal 5-point median filter is used:

```text
[r(i-4), r(i-3), r(i-2), r(i-1), r(i)]
```

The median removes isolated spikes without requiring future readings.

A reading is treated as a glitch when its deviation from the median exceeds:

```text
20 m
```

### Quantization

Altitude is stored in 5 cm units:

```text
v = (h_cm + 2) / 5
```

This allows the 1024-element ring buffer to use `uint16_t`.

Memory:

```text
1024 × uint16_t = 2048 bytes
```

### Deployment Trigger

The firmware compares the actual altitude change over the recent history with the ballistic prediction:

```text
Δactual - Δpredicted > 500 cm
```

A transition window is then frozen so that parachute-inflation readings do not contaminate either model.

**Official Tracker:**

- Glitches detected: **739 / 739**
- Gravity: **9.810 m/s²**
- Apogee: **8.002 s / 626.08 m**
- Deployment: **12.000 s / 547.68 m**
- Descent: **−4.500 m/s**
- Landing: **133.706 s**
- Worst reading: **6416 cycles**
- State: **2336 B**

---

# ⚡ Level 5 — Optimization and Code Freeze

The final implementation introduced several optimizations.

### 1. Closed-form power sums

Repeated loops for power sums were replaced with closed-form Faulhaber identities.

### 2. Single-pass curve caching

The expensive model solution is performed once after ingestion.

The resulting coefficients are cached so that later commands can be answered quickly.

### 3. Fast serial queries

Examples of measured command costs:

| Command | Official cycles |
|---|---:|
| STATS | 10 |
| GLITCHES | 13 |
| DEPLOY | 57 |
| DESCENT | 61 |
| LAND | 391 |
| ALT | 632–1212 |
| APOGEE | 2244 |
| G (first call) | 104,337 |

The first `G` calculation is a post-flight computation and therefore does not violate the **12,000-cycle per-reading ingestion deadline**.

---

# 💾 Memory Architecture

The final `GroundState` uses:

```text
2336 bytes / 4096 bytes
```

That means:

- **57.0% used**
- **1760 bytes free**

Major memory consumers:

| Component | Approx. Size |
|---|---:|
| Ring buffer | 2048 B |
| Cached coefficients | 56 B |
| Descent sums | 36 B |
| Coast sums | 32 B |
| Deployment state | 26 B |
| Median window | 12 B |
| Counters | 8 B |
| Glitch counter | 4 B |
| Serial parser | 34 B |
| Remaining fit state/alignment | remainder |

A compile-time assertion ensures the structure stays within the required limit:

```cpp
static_assert(sizeof(GroundState) <= 4096);
```

---

# 🧪 Final Verification

The final firmware was tested on:

1. **Official Tracker**
2. **Practice Tracker B**

### Official Tracker

| Metric | Result |
|---|---:|
| Gravity | 9.810 m/s² |
| Apogee | 8.002 s / 626.08 m |
| Deployment | 12.000 s / 547.68 m |
| Descent | −4.500 m/s |
| Landing | 133.706 s |
| Glitches | 739 |
| State memory | 2336 B |
| Worst reading | 6418 cycles |

### Practice Tracker B

| Metric | Result |
|---|---:|
| Gravity | 3.711 m/s² |
| Apogee | 11.321 s / 1737.75 m |
| Deployment | 23.999 s / 1439.54 m |
| Descent | −8.500 m/s |
| Landing | 193.351 s |
| Glitches | 727 |
| State memory | 2336 B |
| Worst reading | 6502 cycles |

---

# ✅ Compliance Summary

| Requirement | Final Result |
|---|---|
| 100,000 readings | ✅ Passed |
| ≤ 12,000 cycles per reading | ✅ 6502 worst case |
| ≤ 4096 B persistent state | ✅ 2336 B |
| No hard-coded flight parameters | ✅ |
| No flash/NVS/SPIFFS | ✅ |
| No Wi-Fi/Bluetooth | ✅ |
| Gravity accuracy | ✅ |
| Apogee accuracy | ✅ |
| Deployment accuracy | ✅ |
| Descent accuracy | ✅ |
| Landing accuracy | ✅ |
| ALT queries | ✅ |
| Exact glitch count | ✅ |
| Official Tracker | ✅ |
| Practice Tracker B | ✅ |

---

# 📂 Repository Structure

```text
Team-ByteForce/
│
├── Project/
│   ├── LEVEL_0_FINAL.ino
│   ├── LEVEL_1_OFFICIAL.ino
│   ├── LEVEL__1_PRACTICE.ino
│   ├── LEVEL_2_OFFICIAL.ino
│   ├── level2_practice.ino
│   ├── LEVEL_3_OFFICIAL.ino
│   ├── LEVEL_3_PRACTICE.ino
│   ├── LEVEL_4_OFFICIAL.ino
│   ├── LEVEL_4_PRACTICE.ino
│   ├── LEVEL_5_OFFICIAL.ino
│   ├── LEVEL_5_PRACTICE.ino
│   └── outputs.docx
│
├── README.md
└── FinalReport/
    └── FinalReport_FCA_TeamByteForce.pdf
```

---

# 👥 Team ByteForce

| Member | Role |
|---|---|
| **Chaithra Gana G** | Skeptic and Code Integrate |
| **Sindhu GV** | Builder |
| **Samudra Kar** | Measurer |
| **Brunda** | Scribe |

The development process followed a closed-loop workflow:

```text
Mathematical Planning
        ↓
Firmware Implementation
        ↓
Skeptic Code Audit
        ↓
ESP32 Hardware Test
        ↓
Compare Against Truth
        ↓
Document + Freeze
```

---

# 🔬 Engineering Highlights

### Numerical Stability
Centered coordinates and 64-bit accumulators prevent catastrophic floating-point cancellation during the ballistic fit.

### Memory Efficiency
A quantized `uint16_t` ring buffer reduces the deployment-history requirement to **2048 B**, instead of consuming the entire memory budget with floating-point storage.

### Real-Time Processing
The final worst-case ingestion cost is **6502 cycles**, leaving approximately **5498 cycles of headroom** under the 12,000-cycle deadline.

### Robustness
The causal median filter rejects isolated telemetry glitches before they enter the trajectory models.

### Generalization
The firmware was verified on both the Official Tracker and a substantially different Practice Tracker B rather than relying on a single flight.

---

# 📌 Key Takeaway

This project demonstrates how **mathematical modeling, numerical analysis, memory-aware data structures, signal filtering, and embedded optimization** can turn a 100,000-sample telemetry stream into a complete flight-analysis system on a resource-constrained ESP32.

> **Store the information extracted from the data — not the data itself.**

---

## 📄 Documentation

The repository includes the complete engineering report, level-by-level firmware sketches, and serial-monitor verification outputs.

