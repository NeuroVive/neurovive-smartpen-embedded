# Parkinson-Smart-Pen-Firmware

This repository contains the firmware for a low-cost, untethered smart pen system designed for the non-diagnostic screening and longitudinal monitoring of Parkinsonian motor symptoms, specifically resting tremor.

---

## 1. Project Overview

The system leverages an **ESP32-S3 SuperMini** to perform high-speed edge computing. By integrating vibration sensing via an **MPU-6050 IMU** and multi-point force sensing via a **Tri-FSR (Force-Sensitive Resistor) array**, the pen captures high-fidelity kinematic and kinetic data without being tethered to a host device.

### Core Features

* **Real-time Edge DSP**: On-device filtering (DC removal, Butterworth BPF) and 1024-point FFT analysis.
* **Mechanical Decoupling**: Tri-FSR architecture isolates axial writing force from fingerprint grip rigidity and micro-tremors.
* **Native BLE**: Efficient 1 Hz feature vector transmission via Bluetooth Low Energy 5.
* **Dual-Core Architecture**: ISR-driven data acquisition (Core 0) is logically separated from intensive signal processing (Core 1).

---

## 2. Hardware Architecture

The system is built on a distributed sensor network:

* **Kinematic Sensing**: MPU-6050 IMU communicating over I2C (GPIO 8/9).
* **Axial Force**: FSR-400 (tip) connected via a micro-plunger for writing state isolation.
* **Grip Force**: Dual FSR-402 array (lower barrel) for capturing pill-rolling tremors.
* **ADC Strategy**: All analog sensors are mapped to **ADC1 channels** to avoid conflicts with the ESP32-S3's internal BLE/Wi-Fi radio arbitration.

---

## 3. Firmware Workflow

The firmware follows a deterministic execution pipeline designed for 1 kHz data throughput:

1. **Data Acquisition (Core 0 - ISR):**
* A `GPTimer` triggers a 1 kHz interrupt.
* Samples are read from the MPU-6050 and the FSR array.
* A cascaded 2nd-order Butterworth Band-Pass Filter (3.5–6.5 Hz) isolates the tremor band.
* Filtered data is pushed into a **double-buffered** circular array.


2. **Edge DSP (Core 1 - Task):**
* Triggered by a semaphore when the 1024-sample buffer is full.
* Applies a **Hann window** to reduce spectral leakage.
* Executes an **in-place radix-2 Cooley-Tukey FFT**.
* Extracts dominant tremor frequency, RMS acceleration, and jerk magnitude.


3. **Communication (Core 0 - Task):**
* Assembles a 53-byte custom binary BLE frame.
* Transmits the feature vector to the companion mobile application at 1 Hz.



---

## 4. Code Structure

The project is organized into modular components for maintainability:

* `main/`: Contains `main.c`, the application entry point, task scheduling, and BLE GATT server implementation.
* `components/mpu6050/`: Low-level driver for the IMU, including calibration routines and self-test logic.
* `components/fsr/`: Driver for the FSR array, including polynomial force-conversion logic (ADC raw to grams).

---

## 5. Build & Setup

### Prerequisites

* **ESP-IDF v5.x** installed and configured.
* **CMake** and **Ninja** build system.

## 6. Safety & Ethical Disclaimer

**This system is a non-diagnostic research and monitoring tool.** It is not cleared as a medical device by any regulatory body. All output data must be interpreted in conjunction with professional clinical assessments (e.g., MDS-UPDRS scales). Patient data privacy should be maintained through AES-128 encryption during BLE transmission and secure local storage on the companion mobile device.