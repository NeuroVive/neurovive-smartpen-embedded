/**
 * @file fsr.h
 * @brief Force-Sensitive Resistor (FSR) driver for Parkinson's Smart Pen
 *
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  CRITICAL: ALL FSR channels use ADC1 ONLY.                      ║
 * ║  ADC2 is shared with the ESP32-S3 Wi-Fi/BLE radio hardware.     ║
 * ║  Any attempt to read ADC2 while BLE is active will return        ║
 * ║  ESP_ERR_INVALID_STATE. This driver explicitly uses only ADC1    ║
 * ║  channels to guarantee interference-free sensor readings.        ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * Hardware wiring:
 *
 *   3.3V ──┬── [FSR-400 tip]  ──┬── GPIO 1  (ADC1_CH0)
 *           │                   └── [10kΩ pull-down] ── GND
 *           │                   └── [100nF] ── GND  (primary LPF, fc≈159Hz)
 *           │
 *           ├── [FSR-402 A]    ──┬── GPIO 2  (ADC1_CH1)
 *           │                   └── [10kΩ pull-down] ── GND
 *           │                   └── [100nF] ── GND
 *           │
 *           └── [FSR-402 B]    ──┬── GPIO 3  (ADC1_CH2)
 *                               └── [10kΩ pull-down] ── GND
 *                               └── [100nF] ── GND
 *
 * Boot-safe isolation: A 100kΩ series resistor + 10µF to GND on each GPIO
 * creates a secondary RC stage (fc≈0.16Hz) that holds ADC pins at stable
 * voltage during ESP32-S3 power-on strapping, preventing boot-mode errors.
 *
 * Circuit transfer function (voltage divider):
 *   V_adc = 3.3V × R_pd / (R_fsr + R_pd)
 *   where R_pd = 10kΩ pull-down, R_fsr decreases with applied force.
 *
 * Calibration LUT maps ADC voltage → force in grams using a 3-point
 * polynomial fit to the FSR datasheet resistance-force curve.
 */

#ifndef FSR_H
#define FSR_H

#include <stdint.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* =========================================================================
 * Hardware pin / ADC channel mapping  (ADC1 ONLY — see file header)
 * ========================================================================= */

/** ADC unit: must be ADC_UNIT_1. ADC_UNIT_2 is forbidden with BLE active. */
#define FSR_ADC_UNIT            ADC_UNIT_1

/**
 * Tip axial pressure sensor: FSR-400 (small active area, internal plunger).
 * GPIO 1 maps to ADC1 channel 0 on ESP32-S3.
 */
#define FSR_TIP_GPIO            1
#define FSR_TIP_ADC_CHANNEL     ADC_CHANNEL_0   /**< GPIO 1 = ADC1_CH0 */

/**
 * Fingerprint grip sensor A: FSR-402 (large active area, lower barrel).
 * GPIO 2 maps to ADC1 channel 1 on ESP32-S3.
 */
#define FSR_GRIP_A_GPIO         2
#define FSR_GRIP_A_ADC_CHANNEL  ADC_CHANNEL_1   /**< GPIO 2 = ADC1_CH1 */

/**
 * Fingerprint grip sensor B: FSR-402 (large active area, lower barrel).
 * GPIO 3 maps to ADC1 channel 2 on ESP32-S3.
 */
#define FSR_GRIP_B_GPIO         3
#define FSR_GRIP_B_ADC_CHANNEL  ADC_CHANNEL_2   /**< GPIO 3 = ADC1_CH2 */

/* =========================================================================
 * ADC sampling configuration
 * ========================================================================= */

/** 12-bit resolution: 0–4095 counts, maps to 0–3.3V (post-calibration) */
#define FSR_ADC_BITWIDTH        ADC_BITWIDTH_12

/**
 * Attenuation 11dB: input range 0–3.1V (covers the full 3.3V supply rail
 * with slight headroom). Required since FSR voltage spans 0–3.3V.
 */
#define FSR_ADC_ATTEN           ADC_ATTEN_DB_11

/** Pull-down resistor value in ohms (10kΩ) — used in R_fsr calculation */
#define FSR_PULLDOWN_OHMS       10000.0f

/** Number of ADC samples averaged per read call (oversampling for noise) */
#define FSR_OVERSAMPLE_COUNT    8

/**
 * Pen-state threshold: tip FSR raw ADC counts above this value are
 * considered "writing". Adjust during calibration per unit. Default: 200.
 */
#define FSR_PEN_STATE_THRESHOLD 200

/* =========================================================================
 * Data structures
 * ========================================================================= */

/** Aggregated readings from all three FSR channels */
typedef struct {
    uint16_t tip_raw;           /**< Raw ADC count, tip FSR-400            */
    uint16_t tip_force_gram_x10;/**< Calibrated tip force: grams × 10      */
    uint16_t grip_a_raw;        /**< Raw ADC count, grip FSR-402 A         */
    uint16_t grip_b_raw;        /**< Raw ADC count, grip FSR-402 B         */
    uint16_t grip_mean_x10;     /**< Calibrated mean grip: grams × 10      */
    uint8_t  pen_state;         /**< 0=lifted, 1=writing (from tip FSR)    */
} fsr_reading_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialise ADC1 driver with calibration (esp_adc_cal).
 *
 *         Configures ADC1 channels 0, 1, 2 (GPIO 1, 2, 3) with 12-bit
 *         resolution and 11dB attenuation.
 *
 *         SAFETY: This function calls adc_oneshot_new_unit(ADC_UNIT_1)
 *         explicitly. It will never touch ADC_UNIT_2.
 *
 * @return ESP_OK on success; ESP_ERR_NOT_SUPPORTED if calibration eFuses
 *         are not burned (falls back to approximate curve).
 */
esp_err_t fsr_init(void);

/**
 * @brief  Read all three FSR channels, apply oversampling and calibration.
 *
 *         Each channel is read FSR_OVERSAMPLE_COUNT times and averaged.
 *         The voltage is converted to resistance using the voltage-divider
 *         equation, then to grams via the polynomial LUT.
 *
 * @param[out] out  Pointer to fsr_reading_t to populate.
 * @return ESP_OK on success.
 */
esp_err_t fsr_read(fsr_reading_t *out);

/**
 * @brief  Force a new calibration by sampling all channels at zero-force
 *         state (pen resting unsupported on a flat surface).
 *
 *         Stores offset corrections in driver-internal static storage.
 *         Should be called after fsr_init() before first clinical use.
 *
 * @param  num_samples  Samples to average for zero-force baseline.
 * @return ESP_OK on success.
 */
esp_err_t fsr_calibrate_zero(uint16_t num_samples);

/**
 * @brief  Convert a raw ADC count to estimated force in grams × 10.
 *
 *         Uses the polynomial model fitted to the FSR-402 datasheet curve:
 *           R_fsr = R_pd × (V_supply / V_adc - 1)
 *           Force = a × R_fsr^b  [power-law fit: a=9.32e6, b=-1.42 for FSR-402]
 *
 *         Public so main.c / tests can call it independently.
 *
 * @param  adc_raw   12-bit ADC reading (0–4095).
 * @param  is_fsr402 true for FSR-402 (grip), false for FSR-400 (tip).
 * @return Force estimate in grams × 10 (e.g., 200 = 20.0g). Returns 0
 *         if adc_raw is below noise floor.
 */
uint16_t fsr_raw_to_gram_x10(uint16_t adc_raw, bool is_fsr402);

/**
 * @brief  Deinitialise and release ADC1 driver resources.
 */
void fsr_deinit(void);

#endif /* FSR_H */
