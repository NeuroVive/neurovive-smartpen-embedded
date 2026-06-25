/**
 * @file fsr.c
 * @brief FSR driver implementation — ADC1 only, BLE-safe
 *
 * Force conversion model:
 *   The FSR is wired as the top leg of a voltage divider:
 *     V_adc = 3.3V × R_pd / (R_fsr + R_pd)
 *
 *   Rearranging:
 *     R_fsr = R_pd × (V_supply / V_adc - 1)
 *
 *   The FSR datasheet provides a log-log resistance-force curve.
 *   A power-law fit over the 10g–1000g clinical range gives:
 *     Force_g = k / R_fsr^n
 *
 *   FSR-402 fit:  k = 9.318e6, n = 1.417  (R in Ω, Force in grams)
 *   FSR-400 fit:  k = 7.854e6, n = 1.391  (smaller active area → lower k)
 *
 * esp_adc_cal usage:
 *   The ESP32-S3 has eFuse-stored two-point calibration data. The
 *   esp_adc/adc_cali API reads these values and produces millivolt
 *   readings corrected for non-linearity and gain error. We use
 *   adc_cali_raw_to_voltage() to get mV, then apply the divider math.
 */

#include "fsr.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FSR";

/* =========================================================================
 * Module-private state
 * ========================================================================= */

/** ADC oneshot handle (ADC1 only) */
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

/** Calibration handle for converting raw counts to millivolts */
static adc_cali_handle_t s_cali_handle = NULL;
static bool              s_cali_valid  = false;

/** Zero-force ADC baseline offsets per channel (set by fsr_calibrate_zero) */
static uint16_t s_zero_offset[3] = {0, 0, 0};  /* [tip, grip_a, grip_b] */

/* Supply voltage in millivolts (nominally 3300 mV) */
#define FSR_VSUPPLY_MV  3300.0f

/* =========================================================================
 * Power-law calibration constants
 * ========================================================================= */

/* FSR-402 (grip sensors A and B, larger active area) */
#define FSR402_K    9318000.0f   /**< Power-law coefficient k  */
#define FSR402_N    1.417f       /**< Power-law exponent n     */

/* FSR-400 (tip sensor, smaller active area) */
#define FSR400_K    7854000.0f
#define FSR400_N    1.391f

/** ADC count noise floor below which force is treated as zero */
#define FSR_NOISE_FLOOR_COUNTS  30

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/**
 * @brief Read a single ADC1 channel N times and return the integer average.
 *        Uses esp_adc_cal millivolt conversion if calibration is valid,
 *        otherwise returns raw count.
 */
static uint16_t adc1_read_averaged(adc_channel_t channel)
{
    uint32_t sum = 0;
    int raw = 0;

    for (int i = 0; i < FSR_OVERSAMPLE_COUNT; i++) {
        /* adc_oneshot_read is thread-safe with a mutex inside the driver */
        adc_oneshot_read(s_adc1_handle, channel, &raw);
        sum += (uint32_t)raw;
    }
    return (uint16_t)(sum / FSR_OVERSAMPLE_COUNT);
}

/* =========================================================================
 * Public API implementation
 * ========================================================================= */

esp_err_t fsr_init(void)
{
    /* -----------------------------------------------------------------------
     * Step 1: Create ADC1 oneshot unit handle.
     *
     * IMPORTANT: We explicitly pass ADC_UNIT_1 here. The ADC2 peripheral
     * (ADC_UNIT_2) is shared with the ESP32-S3 Wi-Fi/BLE co-existence
     * arbitration hardware. Initialising ADC2 while BLE is running will
     * cause adc_oneshot_read() to return ESP_ERR_INVALID_STATE with
     * corrupted readings. This driver enforces ADC1-only at compile time
     * through FSR_ADC_UNIT = ADC_UNIT_1.
     * ----------------------------------------------------------------------- */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = FSR_ADC_UNIT,   /* ADC_UNIT_1 — do NOT change to ADC_UNIT_2 */
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle));
    ESP_LOGI(TAG, "ADC1 unit initialised (ADC2 not touched — BLE safe)");

    /* -----------------------------------------------------------------------
     * Step 2: Configure each FSR channel on ADC1.
     * ----------------------------------------------------------------------- */
    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = FSR_ADC_BITWIDTH,
        .atten    = FSR_ADC_ATTEN,
    };

    /* Tip FSR-400: GPIO 1 = ADC1_CH0 */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle,
                    FSR_TIP_ADC_CHANNEL, &ch_cfg));
    ESP_LOGI(TAG, "Tip FSR-400:  GPIO %d -> ADC1_CH%d",
             FSR_TIP_GPIO, FSR_TIP_ADC_CHANNEL);

    /* Grip FSR-402 A: GPIO 2 = ADC1_CH1 */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle,
                    FSR_GRIP_A_ADC_CHANNEL, &ch_cfg));
    ESP_LOGI(TAG, "Grip FSR-402A: GPIO %d -> ADC1_CH%d",
             FSR_GRIP_A_GPIO, FSR_GRIP_A_ADC_CHANNEL);

    /* Grip FSR-402 B: GPIO 3 = ADC1_CH2 */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle,
                    FSR_GRIP_B_ADC_CHANNEL, &ch_cfg));
    ESP_LOGI(TAG, "Grip FSR-402B: GPIO %d -> ADC1_CH%d",
             FSR_GRIP_B_GPIO, FSR_GRIP_B_ADC_CHANNEL);

    /* -----------------------------------------------------------------------
     * Step 3: Initialise esp_adc_cal calibration.
     *
     * The ESP32-S3 supports "curve fitting" calibration (reads eFuse
     * two-point trim values burned at factory). Falls back to "line
     * fitting" if two-point data is absent from eFuse.
     * ----------------------------------------------------------------------- */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = FSR_ADC_UNIT,
        .atten    = FSR_ADC_ATTEN,
        .bitwidth = FSR_ADC_BITWIDTH,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_cfg,
                                                               &s_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id   = FSR_ADC_UNIT,
        .atten     = FSR_ADC_ATTEN,
        .bitwidth  = FSR_ADC_BITWIDTH,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_line_fitting(&cali_cfg,
                                                               &s_cali_handle);
#else
    esp_err_t cali_ret = ESP_ERR_NOT_SUPPORTED;
#endif

    if (cali_ret == ESP_OK) {
        s_cali_valid = true;
        ESP_LOGI(TAG, "ADC calibration loaded from eFuse");
    } else {
        s_cali_valid = false;
        ESP_LOGW(TAG, "ADC calibration unavailable (ret=0x%x) — using raw counts",
                 cali_ret);
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

uint16_t fsr_raw_to_gram_x10(uint16_t adc_raw, bool is_fsr402)
{
    /* Below noise floor -> no contact, return 0g */
    if (adc_raw <= FSR_NOISE_FLOOR_COUNTS) return 0;

    /* Convert raw ADC count to millivolts via calibration table */
    float v_adc_mv = 0.0f;
    if (s_cali_valid && s_cali_handle != NULL) {
        int mv = 0;
        /* adc_cali_raw_to_voltage expects int raw, returns int mV */
        adc_cali_raw_to_voltage(s_cali_handle, (int)adc_raw, &mv);
        v_adc_mv = (float)mv;
    } else {
        /* Fallback: linear approximation
         * 12-bit ADC, 11dB atten → full scale ≈ 3100 mV                   */
        v_adc_mv = (float)adc_raw * (3100.0f / 4095.0f);
    }

    /* Guard against divide-by-zero (if V_adc ≈ 0, FSR is open circuit) */
    if (v_adc_mv < 1.0f) return 0;

    /* -----------------------------------------------------------------------
     * Voltage divider → FSR resistance:
     *   R_fsr = R_pd × (V_supply / V_adc - 1)
     * ----------------------------------------------------------------------- */
    float r_fsr = FSR_PULLDOWN_OHMS * ((FSR_VSUPPLY_MV / v_adc_mv) - 1.0f);

    /* Guard: R_fsr < 200Ω indicates saturation (>10N, above clinical range) */
    if (r_fsr < 200.0f) r_fsr = 200.0f;

    /* -----------------------------------------------------------------------
     * Power-law: Force_g = k / R_fsr^n
     * Use coefficients for the appropriate FSR model.
     * ----------------------------------------------------------------------- */
    float k = is_fsr402 ? FSR402_K : FSR400_K;
    float n = is_fsr402 ? FSR402_N : FSR400_N;

    float force_g = k / powf(r_fsr, n);

    /* Clamp to a sensible clinical range (0–2000g) */
    if (force_g < 0.0f)    force_g = 0.0f;
    if (force_g > 2000.0f) force_g = 2000.0f;

    /* Return grams × 10 as uint16 (e.g. 20.5g → 205) */
    return (uint16_t)(force_g * 10.0f);
}

/* -------------------------------------------------------------------------- */

esp_err_t fsr_read(fsr_reading_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    /* Read each channel with oversampling */
    uint16_t tip_raw    = adc1_read_averaged(FSR_TIP_ADC_CHANNEL);
    uint16_t grip_a_raw = adc1_read_averaged(FSR_GRIP_A_ADC_CHANNEL);
    uint16_t grip_b_raw = adc1_read_averaged(FSR_GRIP_B_ADC_CHANNEL);

    /* Subtract zero-force baseline offsets (set during calibrate_zero) */
    tip_raw    = (tip_raw    > s_zero_offset[0]) ?
                  tip_raw    - s_zero_offset[0] : 0;
    grip_a_raw = (grip_a_raw > s_zero_offset[1]) ?
                  grip_a_raw - s_zero_offset[1] : 0;
    grip_b_raw = (grip_b_raw > s_zero_offset[2]) ?
                  grip_b_raw - s_zero_offset[2] : 0;

    out->tip_raw    = tip_raw;
    out->grip_a_raw = grip_a_raw;
    out->grip_b_raw = grip_b_raw;

    /* Calibrated force: tip uses FSR-400 coefficients, grip uses FSR-402 */
    out->tip_force_gram_x10 = fsr_raw_to_gram_x10(tip_raw,    false);
    uint16_t fa = fsr_raw_to_gram_x10(grip_a_raw, true);
    uint16_t fb = fsr_raw_to_gram_x10(grip_b_raw, true);
    out->grip_mean_x10 = (uint16_t)((fa + fb) / 2);

    /* Pen state: writing if tip pressure exceeds threshold */
    out->pen_state = (tip_raw > FSR_PEN_STATE_THRESHOLD) ? 1 : 0;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

esp_err_t fsr_calibrate_zero(uint16_t num_samples)
{
    ESP_LOGI(TAG, "Zero-force calibration (%u samples) — remove all pressure",
             num_samples);

    uint32_t sum[3] = {0, 0, 0};
    int raw = 0;

    for (uint16_t i = 0; i < num_samples; i++) {
        adc_oneshot_read(s_adc1_handle, FSR_TIP_ADC_CHANNEL,    &raw);
        sum[0] += (uint32_t)raw;
        adc_oneshot_read(s_adc1_handle, FSR_GRIP_A_ADC_CHANNEL, &raw);
        sum[1] += (uint32_t)raw;
        adc_oneshot_read(s_adc1_handle, FSR_GRIP_B_ADC_CHANNEL, &raw);
        sum[2] += (uint32_t)raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    for (int i = 0; i < 3; i++) {
        s_zero_offset[i] = (uint16_t)(sum[i] / num_samples);
    }

    ESP_LOGI(TAG, "Zero offsets: tip=%u  gripA=%u  gripB=%u",
             s_zero_offset[0], s_zero_offset[1], s_zero_offset[2]);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

void fsr_deinit(void)
{
    if (s_cali_valid && s_cali_handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(s_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(s_cali_handle);
#endif
        s_cali_handle = NULL;
        s_cali_valid  = false;
    }
    if (s_adc1_handle) {
        adc_oneshot_del_unit(s_adc1_handle);
        s_adc1_handle = NULL;
    }
    ESP_LOGI(TAG, "FSR driver deinitialised");
}
