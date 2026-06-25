/**
 * @file main.c
 * @brief Parkinson's Smart Pen — Main Firmware Entry Point
 *
 * ESP32-S3 SuperMini | FreeRTOS | ESP-IDF v5.x
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  SYSTEM OVERVIEW                                                         ║
 * ║                                                                          ║
 * ║  Peripheral init order (mandatory):                                      ║
 * ║    1. I2C  → MPU6050 (GPIO 8 SDA, GPIO 9 SCL)                           ║
 * ║    2. ADC1 → FSR tri-sensor array (GPIO 1, 2, 3)   [ADC2 NEVER USED]    ║
 * ║    3. GPTimer 1 kHz → acquisition ISR                                    ║
 * ║    4. NimBLE stack → BLE GATT server                                     ║
 * ║                                                                          ║
 * ║  FreeRTOS task map:                                                      ║
 * ║    ┌─────────────────────────────────────────────────────────────┐       ║
 * ║    │ Task              Core  Priority  Stack   Period            │       ║
 * ║    │ dsp_task          1     5         8192B   triggered by ISR  │       ║
 * ║    │ ble_tx_task       0     4         4096B   1 Hz (1000 ms)    │       ║
 * ║    └─────────────────────────────────────────────────────────────┘       ║
 * ║                                                                          ║
 * ║  Data flow:                                                              ║
 * ║    GPTimer ISR (1 kHz)                                                   ║
 * ║      └─ reads MPU6050 + 3×FSR                                            ║
 * ║      └─ applies DC-removal + HPF + BPF (3.5–6.5 Hz Butterworth)         ║
 * ║      └─ appends to 1024-sample circular buffer                           ║
 * ║      └─ signals dsp_task when buffer full                                ║
 * ║                                                                          ║
 * ║    dsp_task (Core 1)                                                     ║
 * ║      └─ 1024-point in-place FFT on ax channel                           ║
 * ║      └─ extracts dominant frequency + RMS amplitude                      ║
 * ║      └─ computes jerk magnitude                                          ║
 * ║      └─ updates g_dsp_result (mutex-protected)                           ║
 * ║                                                                          ║
 * ║    ble_tx_task (Core 0)                                                  ║
 * ║      └─ wakes every 1 s                                                  ║
 * ║      └─ assembles 53-byte BLE frame                                      ║
 * ║      └─ notifies connected GATT client                                   ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * BLE Packet layout (53 bytes, big-endian, two's complement signed fields):
 *
 *   [00]       pkt_type          uint8   = 0x01
 *   [01]       seq_num           uint8   rollover counter
 *   [02..05]   timestamp_ms      uint32  millis() since boot
 *   [06..07]   ax_raw            int16   MPU6050 accel X
 *   [08..09]   ay_raw            int16   MPU6050 accel Y
 *   [10..11]   az_raw            int16   MPU6050 accel Z
 *   [12..13]   gx_raw            int16   MPU6050 gyro X
 *   [14..15]   gy_raw            int16   MPU6050 gyro Y
 *   [16..17]   gz_raw            int16   MPU6050 gyro Z
 *   [18..19]   pitch_x100        int16   pitch °×100
 *   [20..21]   roll_x100         int16   roll  °×100
 *   [22..23]   tip_fsr400_raw    uint16  tip ADC count
 *   [24..25]   tip_force_x10     uint16  tip grams×10
 *   [26..27]   grip_a_raw        uint16  grip A ADC count
 *   [28..29]   grip_b_raw        uint16  grip B ADC count
 *   [30..31]   grip_mean_x10     uint16  mean grip grams×10
 *   [32..33]   tremor_freq_x100  uint16  dominant FFT freq Hz×100
 *   [34..35]   tremor_rms_x1000  uint16  RMS accel m/s²×1000
 *   [36..37]   jerk_mag_x100     uint16  jerk magnitude m/s³×100
 *   [38]       pen_state         uint8   0=lifted 1=writing
 *   [39]       lift_count        uint8   lifts in current window
 *   [40..41]   cal_ax            int16   accel X bias offset
 *   [42..43]   cal_ay            int16   accel Y bias offset
 *   [44..45]   cal_az            int16   accel Z bias offset
 *   [46..47]   cal_gx            int16   gyro X bias offset
 *   [48..49]   cal_gy            int16   gyro Y bias offset
 *   [50..51]   cal_gz            int16   gyro Z bias offset
 *   [52]       checksum          uint8   XOR of bytes [00..51]
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* ESP-IDF */
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"

/* NimBLE (ESP-IDF bundled BLE stack) */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Project drivers */
#include "mpu6050.h"
#include "fsr.h"

static const char *TAG = "SMARTPEN";

/* ==========================================================================
 * Constants
 * ========================================================================== */

#define SAMPLE_RATE_HZ          1000        /**< 1 kHz acquisition rate      */
#define FFT_WINDOW_SIZE         1024        /**< Samples per FFT window       */
#define BLE_PACKET_SIZE         53          /**< Total bytes per BLE frame    */
#define BLE_TX_PERIOD_MS        1000        /**< Feature vector TX interval   */

/** Tremor band limits (Hz) used to mask FFT bins outside PD range */
#define TREMOR_BAND_LOW_HZ      3.5f
#define TREMOR_BAND_HIGH_HZ     6.5f

/** BLE UUIDs — 16-bit shorthand for custom service/characteristics */
#define BLE_SVC_UUID            0xFFE0
#define BLE_CHAR_FEATURE_UUID   0xFFE1      /**< 1 Hz feature vector notify  */
#define BLE_CHAR_RAW_UUID       0xFFE2      /**< Raw waveform stream notify   */

/** Device advertising name (max 29 bytes for BLE ADV payload) */
#define BLE_DEVICE_NAME         "SmartPen-PD"

/* ==========================================================================
 * Butterworth Bandpass Filter Coefficients (3.5 – 6.5 Hz @ 1 kHz)
 *
 * Design: 2nd-order cascaded Butterworth IIR bandpass.
 * Tool:   scipy.signal.butter(2, [3.5, 6.5], btype='band', fs=1000, output='sos')
 *
 * Two second-order sections (SOS):
 *   Section 0 (HPF stage):
 *     b = [1, 0, -1]   a = [1, -1.97789, 0.97815]
 *   Section 1 (LPF stage):
 *     b = [1, 0, -1]   a = [1, -1.99589, 0.99607]
 *
 * Each section implements:
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
 *          - a1*y[n-1] - a2*y[n-2]
 *
 * The gain scalar brings the pass-band response to unity.
 * ========================================================================== */

/** Coefficients for SOS section 0 (high-pass at 3.5 Hz) */
#define BPF_S0_B0    1.0f
#define BPF_S0_B1    0.0f
#define BPF_S0_B2   -1.0f
#define BPF_S0_A1   -1.97789f
#define BPF_S0_A2    0.97815f

/** Coefficients for SOS section 1 (low-pass at 6.5 Hz) */
#define BPF_S1_B0    1.0f
#define BPF_S1_B1    0.0f
#define BPF_S1_B2   -1.0f
#define BPF_S1_A1   -1.99589f
#define BPF_S1_A2    0.99607f

/** Pass-band gain correction (product of both section gains at 5 Hz) */
#define BPF_GAIN     3.21e-3f

/* State variables for cascaded IIR filter (two sections × two delay taps) */
typedef struct {
    float x1, x2;   /**< Input  delay line [n-1], [n-2] */
    float y1, y2;   /**< Output delay line [n-1], [n-2] */
} biquad_state_t;

static biquad_state_t s_bpf[2] = {0};  /**< Section 0 and Section 1 states */

/* ==========================================================================
 * Circular sample buffer (double-buffered for lock-free ISR write)
 * ========================================================================== */

/**
 * Two physical buffers allow the ISR to write into the inactive buffer
 * while the DSP task processes the full active buffer — no mutex needed
 * for the buffer swap itself, only for the DSP result struct.
 */
static float    s_acq_buf[2][FFT_WINDOW_SIZE];  /**< ax filtered samples     */
static uint16_t s_buf_write_idx = 0;            /**< Current write position  */
static uint8_t  s_active_buf    = 0;            /**< Buffer the ISR writes to*/
static uint8_t  s_lift_count    = 0;            /**< Pen lifts in window      */
static uint8_t  s_prev_pen_state = 0;

/* ==========================================================================
 * Shared data structures (ISR -> DSP task -> BLE task)
 * ========================================================================== */

/** Latest raw sensor snapshot (written by ISR, read by BLE task) */
typedef struct {
    mpu6050_raw_t   imu;
    mpu6050_orient_t orient;
    fsr_reading_t   fsr;
    uint32_t        timestamp_ms;
} sensor_snapshot_t;

/** DSP output computed by dsp_task */
typedef struct {
    uint16_t tremor_freq_x100;   /**< Dominant FFT freq, Hz × 100          */
    uint16_t tremor_rms_x1000;   /**< RMS acceleration amplitude, m/s²×1000*/
    uint16_t jerk_mag_x100;      /**< Jerk magnitude, m/s³ × 100           */
    uint8_t  lift_count;         /**< Pen-lift transitions in window        */
} dsp_result_t;

static sensor_snapshot_t g_snapshot  = {0};
static dsp_result_t      g_dsp_result = {0};

/** Mutex protecting g_dsp_result and g_snapshot from concurrent access */
static SemaphoreHandle_t s_data_mutex = NULL;

/** Binary semaphore signalled by ISR when buffer is full → wakes dsp_task */
static SemaphoreHandle_t s_dsp_trigger = NULL;

/* ==========================================================================
 * BLE state
 * ========================================================================== */

static uint16_t s_ble_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_char_feature_val_handle = 0;
static uint8_t  s_seq_num          = 0;
static bool     s_ble_subscribed   = false;

/* Forward declarations */
static void ble_advertise(void);
static int  gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);

/* ==========================================================================
 * FFT implementation (Cooley-Tukey radix-2 DIT, in-place)
 *
 * A lightweight real-to-complex FFT for the 1024-point window.
 * We only need the magnitude spectrum up to Nyquist (512 bins),
 * so a full complex FFT on a real input is computed by treating
 * the real array as a complex array with zero imaginary parts.
 *
 * For N=1024 at 160 MHz (ESP32-S3 Xtensa LX7) this takes ~2 ms,
 * well within the 1-second window budget.
 * ========================================================================== */

#define FFT_N   FFT_WINDOW_SIZE

/**
 * @brief In-place Cooley-Tukey radix-2 DIT FFT.
 * @param re  Real part array (N floats, overwritten with FFT real output).
 * @param im  Imaginary part array (N floats, must be zero-initialised for
 *            real-input signals, overwritten with FFT imag output).
 * @param n   Transform length (must be power of 2, e.g. 1024).
 */
static void fft_radix2(float *re, float *im, int n)
{
    /* Bit-reversal permutation */
    int j = 0;
    for (int i = 1; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tmp;
            tmp = re[i]; re[i] = re[j]; re[j] = tmp;
            tmp = im[i]; im[i] = im[j]; im[j] = tmp;
        }
    }

    /* Butterfly stages */
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wre = cosf(ang);
        float wim = sinf(ang);
        for (int i = 0; i < n; i += len) {
            float cur_re = 1.0f, cur_im = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                float u_re = re[i + k];
                float u_im = im[i + k];
                float v_re = re[i + k + len/2] * cur_re
                           - im[i + k + len/2] * cur_im;
                float v_im = re[i + k + len/2] * cur_im
                           + im[i + k + len/2] * cur_re;
                re[i + k]         = u_re + v_re;
                im[i + k]         = u_im + v_im;
                re[i + k + len/2] = u_re - v_re;
                im[i + k + len/2] = u_im - v_im;
                /* Advance twiddle factor: cur *= (wre + j*wim) */
                float next_re = cur_re * wre - cur_im * wim;
                float next_im = cur_re * wim + cur_im * wre;
                cur_re = next_re;
                cur_im = next_im;
            }
        }
    }
}

/* ==========================================================================
 * DSP helper: apply one sample through the cascaded Butterworth BPF
 * ========================================================================== */

/**
 * @brief Apply a single sample through one biquad section.
 *        Direct Form II transposed for numerical stability.
 */
static inline float biquad_process(biquad_state_t *s,
                                    float b0, float b1, float b2,
                                    float a1, float a2,
                                    float x)
{
    float y = b0 * x + b1 * s->x1 + b2 * s->x2
                     - a1 * s->y1 - a2 * s->y2;
    s->x2 = s->x1;  s->x1 = x;
    s->y2 = s->y1;  s->y1 = y;
    return y;
}

/**
 * @brief Run one sample through the full cascaded 2-stage BPF (3.5–6.5 Hz).
 *        Stage 0 is the high-pass roll-off, stage 1 is the low-pass roll-off.
 *        The gain scalar compensates for the combined pass-band attenuation.
 */
static float bpf_process(float x)
{
    float y0 = biquad_process(&s_bpf[0],
                               BPF_S0_B0, BPF_S0_B1, BPF_S0_B2,
                               BPF_S0_A1, BPF_S0_A2, x);
    float y1 = biquad_process(&s_bpf[1],
                               BPF_S1_B0, BPF_S1_B1, BPF_S1_B2,
                               BPF_S1_A1, BPF_S1_A2, y0);
    return y1 * BPF_GAIN;
}

/* ==========================================================================
 * 1 kHz GPTimer ISR — data acquisition and filtering
 * ========================================================================== */

/**
 * @brief GPTimer alarm callback: fires every 1 ms (1 kHz).
 *
 *        Execution context: timer ISR (no FreeRTOS blocking calls allowed).
 *        Reads MPU6050 and all three FSR channels, applies the BPF to ax,
 *        and appends the filtered sample to the active circular buffer.
 *        When the buffer reaches FFT_WINDOW_SIZE samples it signals the
 *        DSP task via a FreeRTOS binary semaphore.
 *
 * @return true to keep the timer running (auto-reload).
 */
static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t        timer,
                                          const gptimer_alarm_event_data_t *edata,
                                          void                    *user_ctx)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    /* ----- Read IMU (non-blocking, returns immediately if I2C busy) ----- */
    mpu6050_raw_t imu = {0};
    mpu6050_read_raw(&imu);   /* Returns ESP_ERR_TIMEOUT silently if busy */

    /* ----- Read FSR channels (ADC1 oneshot, ~10 µs per call) ----------- */
    fsr_reading_t fsr = {0};
    fsr_read(&fsr);

    /* ----- DC removal: subtract running mean from ax ------------------- */
    static float s_dc_mean = 0.0f;
    float ax_f = (float)imu.ax_raw;
    /* Single-pole IIR DC blocker: y = x - mean(x), fc ≈ 1 Hz */
    s_dc_mean += 0.00628f * (ax_f - s_dc_mean);  /* α = 2π×1/1000 */
    float ax_ac = ax_f - s_dc_mean;

    /* ----- Butterworth BPF 3.5–6.5 Hz --------------------------------- */
    float ax_filtered = bpf_process(ax_ac);

    /* ----- Append to circular buffer ----------------------------------- */
    s_acq_buf[s_active_buf][s_buf_write_idx] = ax_filtered;
    s_buf_write_idx++;

    /* ----- Track pen-lift transitions (for lift_count metric) ---------- */
    if (s_prev_pen_state == 1 && fsr.pen_state == 0) {
        s_lift_count++;
    }
    s_prev_pen_state = fsr.pen_state;

    /* ----- Update snapshot under no lock (ISR context) ----------------- */
    /* This is an atomic struct copy; worst case the BLE task reads a
     * sample that is one ISR tick stale — acceptable for 1 Hz BLE TX.    */
    g_snapshot.imu          = imu;
    g_snapshot.fsr          = fsr;
    g_snapshot.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    /* ----- When buffer full: swap, signal DSP task --------------------- */
    if (s_buf_write_idx >= FFT_WINDOW_SIZE) {
        s_buf_write_idx = 0;
        s_active_buf   ^= 1;   /* Toggle active buffer index (0→1 or 1→0) */

        /* Signal DSP task — use FromISR variant */
        xSemaphoreGiveFromISR(s_dsp_trigger, &higher_priority_task_woken);
    }

    return higher_priority_task_woken == pdTRUE;
}

/* ==========================================================================
 * DSP Task — runs on Core 1, triggered every ~1 second by ISR
 * ========================================================================== */

/**
 * @brief Process the completed 1024-sample window.
 *
 *        1. Copy the inactive buffer (the one the ISR just filled) into a
 *           local working array — the ISR is now writing into the other one.
 *        2. Apply a Hann window to reduce spectral leakage.
 *        3. Run the 1024-point radix-2 FFT.
 *        4. Find peak magnitude bin in the 3.5–6.5 Hz tremor band.
 *        5. Compute RMS of the filtered signal.
 *        6. Compute jerk magnitude from successive IMU samples.
 *        7. Update g_dsp_result under mutex.
 */
static void dsp_task(void *pvParameters)
{
    /* Working arrays for the FFT (stack-allocated — dsp_task has 8 kB stack) */
    static float fft_re[FFT_N];
    static float fft_im[FFT_N];

    /* Previous ax sample for jerk (derivative of acceleration) */
    static float prev_ax_g = 0.0f;

    ESP_LOGI(TAG, "DSP task started on core %d", xPortGetCoreID());

    while (true) {
        /* Block indefinitely until ISR signals buffer full */
        xSemaphoreTake(s_dsp_trigger, portMAX_DELAY);

        /* The ISR has just toggled s_active_buf, so the buffer we want is
         * the one that was active BEFORE the toggle:  (s_active_buf ^ 1)   */
        uint8_t proc_buf = s_active_buf ^ 1;

        /* ---- Copy buffer into local FFT working array ------------------ */
        memcpy(fft_re, s_acq_buf[proc_buf], sizeof(float) * FFT_N);
        memset(fft_im, 0,                   sizeof(float) * FFT_N);

        /* ---- Apply Hann window ----------------------------------------- */
        /* w[n] = 0.5 × (1 - cos(2π·n / (N-1)))
         * Reduces spectral leakage between adjacent frequency bins.        */
        for (int n = 0; n < FFT_N; n++) {
            float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI
                                           * (float)n / (float)(FFT_N - 1)));
            fft_re[n] *= w;
        }

        /* ---- Compute RMS of windowed (pre-FFT) signal ------------------ */
        float sum_sq = 0.0f;
        for (int n = 0; n < FFT_N; n++) {
            sum_sq += fft_re[n] * fft_re[n];
        }
        float rms_lsb = sqrtf(sum_sq / (float)FFT_N);

        /* Convert from ADC LSB to m/s²:
         *   accel_g = LSB / 16384.0
         *   accel_mss = accel_g × 9.81
         * rms_x1000 = rms_lsb / 16384 × 9.81 × 1000                       */
        float rms_mss = rms_lsb / MPU6050_ACCEL_SENSITIVITY * 9.81f;
        uint16_t tremor_rms_x1000 = (uint16_t)(rms_mss * 1000.0f);
        if (tremor_rms_x1000 > 60000) tremor_rms_x1000 = 60000;  /* clamp */

        /* ---- Execute 1024-point radix-2 FFT ---------------------------- */
        fft_radix2(fft_re, fft_im, FFT_N);

        /* ---- Find dominant frequency in tremor band [3.5, 6.5] Hz ------ */
        /*
         * Frequency resolution: df = Fs / N = 1000 / 1024 = 0.9766 Hz/bin
         *
         * Bin range for tremor band:
         *   bin_low  = ceil (3.5 / df) = ceil (3.584) = 4
         *   bin_high = floor(6.5 / df) = floor(6.656) = 6
         *
         * Only bins 4–6 are inspected (inclusive). Magnitude = sqrt(re²+im²).
         */
        float df = (float)SAMPLE_RATE_HZ / (float)FFT_N;  /* Hz per bin    */
        int bin_low  = (int)ceilf (TREMOR_BAND_LOW_HZ  / df);
        int bin_high = (int)floorf(TREMOR_BAND_HIGH_HZ / df);

        float  peak_mag = 0.0f;
        int    peak_bin = bin_low;

        for (int k = bin_low; k <= bin_high; k++) {
            float mag = sqrtf(fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k]);
            if (mag > peak_mag) {
                peak_mag = mag;
                peak_bin = k;
            }
        }

        /* Frequency of peak bin, encoded as Hz × 100 for BLE packing */
        float    dom_freq_hz     = (float)peak_bin * df;
        uint16_t tremor_freq_x100 = (uint16_t)(dom_freq_hz * 100.0f);

        /* ---- Compute jerk magnitude ------------------------------------- */
        /* Jerk = d(accel)/dt.  Using ax from the latest snapshot.
         * jerk_mss3 = |Δax_g × 9.81| / dt_s
         *           = |Δax_g × 9.81| × SAMPLE_RATE_HZ
         * We use the inter-window difference (Δt = 1/1024 of a second).    */
        float ax_g_now = (float)g_snapshot.imu.ax_raw / MPU6050_ACCEL_SENSITIVITY;
        float jerk_mss3 = fabsf((ax_g_now - prev_ax_g) * 9.81f
                                 * (float)SAMPLE_RATE_HZ);
        prev_ax_g = ax_g_now;

        uint16_t jerk_mag_x100 = (uint16_t)(jerk_mss3 * 100.0f);
        if (jerk_mag_x100 > 60000) jerk_mag_x100 = 60000;  /* clamp */

        /* ---- Compute orientation from current snapshot IMU ------------- */
        mpu6050_orient_t orient;
        mpu6050_compute_orientation(&g_snapshot.imu, &orient);

        /* ---- Update shared result under mutex -------------------------- */
        if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_dsp_result.tremor_freq_x100 = tremor_freq_x100;
            g_dsp_result.tremor_rms_x1000 = tremor_rms_x1000;
            g_dsp_result.jerk_mag_x100    = jerk_mag_x100;
            g_dsp_result.lift_count       = s_lift_count;
            g_snapshot.orient             = orient;
            s_lift_count = 0;   /* Reset lift counter for next window */
            xSemaphoreGive(s_data_mutex);
        }

        ESP_LOGD(TAG, "DSP: freq=%.2fHz rms=%.4fm/s² jerk=%.2fm/s³",
                 dom_freq_hz, rms_mss, jerk_mss3);
    }
}

/* ==========================================================================
 * BLE Packet Assembly
 * ========================================================================== */

/**
 * @brief Pack all sensor and DSP data into the 53-byte BLE frame.
 *
 *        All multi-byte fields are packed big-endian (network byte order)
 *        to match the specification. Signed values use two's complement
 *        naturally via C int16_t cast → two uint8_t writes.
 *
 * @param[out] buf  Output buffer of at least BLE_PACKET_SIZE bytes.
 */
static void pack_ble_frame(uint8_t *buf)
{
    /* Take a consistent snapshot of all shared data under mutex */
    sensor_snapshot_t snap;
    dsp_result_t      dsp;
    const mpu6050_cal_t *cal = mpu6050_get_cal();

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        snap = g_snapshot;
        dsp  = g_dsp_result;
        xSemaphoreGive(s_data_mutex);
    }

    /* Convenience macro: write uint16 big-endian at buf[pos] */
#define PUT_U16(pos, val) \
    do { \
        buf[(pos)]     = (uint8_t)(((uint16_t)(val)) >> 8); \
        buf[(pos) + 1] = (uint8_t)(((uint16_t)(val)) & 0xFF); \
    } while(0)

    /* Convenience macro: write int16 big-endian (two's complement) */
#define PUT_I16(pos, val) PUT_U16(pos, (uint16_t)(int16_t)(val))

    /* Convenience macro: write uint32 big-endian */
#define PUT_U32(pos, val) \
    do { \
        buf[(pos)]     = (uint8_t)(((uint32_t)(val)) >> 24); \
        buf[(pos) + 1] = (uint8_t)(((uint32_t)(val)) >> 16); \
        buf[(pos) + 2] = (uint8_t)(((uint32_t)(val)) >>  8); \
        buf[(pos) + 3] = (uint8_t)(((uint32_t)(val))  & 0xFF); \
    } while(0)

    /* ---- Header -------------------------------------------------------- */
    buf[0]  = 0x01;                       /* [00]       pkt_type             */
    buf[1]  = s_seq_num++;                /* [01]       seq_num (post-inc)   */
    PUT_U32(2, snap.timestamp_ms);        /* [02..05]   timestamp_ms         */

    /* ---- MPU6050 raw IMU ----------------------------------------------- */
    PUT_I16( 6, snap.imu.ax_raw);         /* [06..07]   ax_raw               */
    PUT_I16( 8, snap.imu.ay_raw);         /* [08..09]   ay_raw               */
    PUT_I16(10, snap.imu.az_raw);         /* [10..11]   az_raw               */
    PUT_I16(12, snap.imu.gx_raw);         /* [12..13]   gx_raw               */
    PUT_I16(14, snap.imu.gy_raw);         /* [14..15]   gy_raw               */
    PUT_I16(16, snap.imu.gz_raw);         /* [16..17]   gz_raw               */

    /* ---- Computed orientation ------------------------------------------ */
    PUT_I16(18, snap.orient.pitch_x100);  /* [18..19]   pitch_x100           */
    PUT_I16(20, snap.orient.roll_x100);   /* [20..21]   roll_x100            */

    /* ---- FSR tri-sensor channels --------------------------------------- */
    PUT_U16(22, snap.fsr.tip_raw);            /* [22..23] tip_fsr400_raw     */
    PUT_U16(24, snap.fsr.tip_force_gram_x10); /* [24..25] tip_force_x10      */
    PUT_U16(26, snap.fsr.grip_a_raw);         /* [26..27] grip_a_raw         */
    PUT_U16(28, snap.fsr.grip_b_raw);         /* [28..29] grip_b_raw         */
    PUT_U16(30, snap.fsr.grip_mean_x10);      /* [30..31] grip_mean_x10      */

    /* ---- DSP outputs --------------------------------------------------- */
    PUT_U16(32, dsp.tremor_freq_x100);    /* [32..33]   tremor_freq_x100     */
    PUT_U16(34, dsp.tremor_rms_x1000);    /* [34..35]   tremor_rms_x1000     */
    PUT_U16(36, dsp.jerk_mag_x100);       /* [36..37]   jerk_mag_x100        */

    /* ---- Pen state ----------------------------------------------------- */
    buf[38] = snap.fsr.pen_state;         /* [38]       pen_state            */
    buf[39] = dsp.lift_count;             /* [39]       lift_count           */

    /* ---- IMU calibration offsets (all 6 axes) -------------------------- */
    PUT_I16(40, cal->cal_ax);             /* [40..41]   cal_ax               */
    PUT_I16(42, cal->cal_ay);             /* [42..43]   cal_ay               */
    PUT_I16(44, cal->cal_az);             /* [44..45]   cal_az               */
    PUT_I16(46, cal->cal_gx);             /* [46..47]   cal_gx               */
    PUT_I16(48, cal->cal_gy);             /* [48..49]   cal_gy               */
    PUT_I16(50, cal->cal_gz);             /* [50..51]   cal_gz               */

    /* ---- Checksum: XOR of bytes [00..51] ------------------------------- */
    uint8_t xor_sum = 0;
    for (int i = 0; i < 52; i++) xor_sum ^= buf[i];
    buf[52] = xor_sum;                    /* [52]       checksum             */

#undef PUT_U16
#undef PUT_I16
#undef PUT_U32
}

/* ==========================================================================
 * BLE TX Task — runs on Core 0, wakes every 1 second
 * ========================================================================== */

static void ble_tx_task(void *pvParameters)
{
    static uint8_t frame[BLE_PACKET_SIZE];

    ESP_LOGI(TAG, "BLE TX task started on core %d", xPortGetCoreID());

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(BLE_TX_PERIOD_MS));

        if (!s_ble_subscribed ||
            s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            continue;   /* No client connected or not subscribed */
        }

        pack_ble_frame(frame);

        /* Create an mbuf and write the frame into it */
        struct os_mbuf *om = ble_hs_mbuf_from_flat(frame, BLE_PACKET_SIZE);
        if (om == NULL) {
            ESP_LOGE(TAG, "Failed to allocate BLE mbuf");
            continue;
        }

        int rc = ble_gatts_notify_custom(s_ble_conn_handle,
                                          s_char_feature_val_handle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "BLE notify error: rc=%d", rc);
        } else {
            ESP_LOGD(TAG, "BLE frame sent: seq=%u ts=%lu ms",
                     frame[1], (unsigned long)
                     ((frame[2]<<24)|(frame[3]<<16)|(frame[4]<<8)|frame[5]));
        }
    }
}

/* ==========================================================================
 * GATT Server definition
 * ========================================================================== */

static const struct ble_gatt_svc_def s_gatt_svr_svcs[] = {
    {
        /* Primary service: Smart Pen sensor data */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Feature vector characteristic (1 Hz notify) */
                .uuid       = BLE_UUID16_DECLARE(BLE_CHAR_FEATURE_UUID),
                .access_cb  = gatt_svr_chr_access,
                .val_handle = &s_char_feature_val_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Raw waveform characteristic (higher rate notify) */
                .uuid      = BLE_UUID16_DECLARE(BLE_CHAR_RAW_UUID),
                .access_cb = gatt_svr_chr_access,
                .flags     = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },  /* Terminator */
        },
    },
    { 0 },  /* Terminator */
};

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Read requests are not supported on notify-only characteristics */
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

/* ==========================================================================
 * BLE GAP event handler
 * ========================================================================== */

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_ble_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected: conn_handle=%u",
                     s_ble_conn_handle);
        } else {
            ESP_LOGW(TAG, "BLE connection failed: status=%d",
                     event->connect.status);
            s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_advertise();   /* Restart advertising */
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected: reason=%d",
                 event->disconnect.reason);
        s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ble_subscribed  = false;
        ble_advertise();   /* Restart advertising after disconnect */
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_char_feature_val_handle) {
            s_ble_subscribed = (event->subscribe.cur_notify == 1);
            ESP_LOGI(TAG, "Notify subscription: %s",
                     s_ble_subscribed ? "ENABLED" : "DISABLED");
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: conn=%u mtu=%u",
                 event->mtu.conn_handle, event->mtu.value);
        break;

    default:
        break;
    }
    return 0;
}

/* ==========================================================================
 * BLE advertising
 * ========================================================================== */

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields  fields     = {0};

    /* Set the device name in the advertising payload */
    fields.name            = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len        = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    /* Advertise the primary service UUID */
    ble_uuid16_t svc_uuid16 = BLE_UUID16_INIT(BLE_SVC_UUID);
    fields.uuids16          = &svc_uuid16;
    fields.num_uuids16      = 1;
    fields.uuids16_is_complete = 1;

    /* General discoverable mode, not connectable on iOS requires
     * BLE_GAP_CONN_MODE_UND (undirected connectable)                        */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    /* 100 ms advertising interval (160 × 0.625 ms) */
    adv_params.itvl_min  = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    adv_params.itvl_max  = BLE_GAP_ADV_FAST_INTERVAL1_MAX;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: rc=%d", rc);
        return;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                            &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE advertising started as '%s'", BLE_DEVICE_NAME);
}

/* ==========================================================================
 * NimBLE host task (required by ESP-IDF NimBLE port)
 * ========================================================================== */

static void nimble_host_task(void *pvParameters)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();              /* Blocks until nimble_port_stop() called */
    nimble_port_freertos_deinit();
}

/* ==========================================================================
 * BLE host synchronisation callback
 * ========================================================================== */

static void ble_on_sync(void)
{
    /* Ensure we have a valid public address before advertising */
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: rc=%d", rc);
        return;
    }
    ble_advertise();
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset: reason=%d", reason);
}

/* ==========================================================================
 * GPTimer setup (1 kHz acquisition timer)
 * ========================================================================== */

static gptimer_handle_t s_acq_timer = NULL;

static esp_err_t timer_init(void)
{
    gptimer_config_t timer_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   /* 1 MHz tick → 1 µs resolution         */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &s_acq_timer));

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count             = 1000,    /* 1000 µs = 1 ms → 1 kHz       */
        .reload_count            = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_acq_timer, &alarm_cfg));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_acq_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_acq_timer));
    ESP_ERROR_CHECK(gptimer_start(s_acq_timer));

    ESP_LOGI(TAG, "GPTimer started: 1 kHz acquisition ISR");
    return ESP_OK;
}

/* ==========================================================================
 * app_main — peripheral init in mandatory order, then task launch
 * ========================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Parkinson's Smart Pen Firmware v1.0");
    ESP_LOGI(TAG, "  ESP32-S3 SuperMini | ESP-IDF v5.x  ");
    ESP_LOGI(TAG, "======================================");

    /* -----------------------------------------------------------------------
     * NVS flash — required by BLE stack for pairing key storage
     * ----------------------------------------------------------------------- */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* -----------------------------------------------------------------------
     * Create synchronisation primitives before starting any tasks or ISRs
     * ----------------------------------------------------------------------- */
    s_data_mutex  = xSemaphoreCreateMutex();
    s_dsp_trigger = xSemaphoreCreateBinary();
    configASSERT(s_data_mutex  != NULL);
    configASSERT(s_dsp_trigger != NULL);

    /* -----------------------------------------------------------------------
     * Step 1: I2C + MPU6050 initialisation
     * Must come first — the timer ISR will start reading MPU6050 immediately
     * after the timer is started.
     * ----------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[1/4] Initialising MPU6050 on I2C (SDA=GPIO%d SCL=GPIO%d)",
             MPU6050_PIN_SDA, MPU6050_PIN_SCL);
    ESP_ERROR_CHECK(mpu6050_init());

    /* Optional self-test (comment out for faster boot in production) */
    esp_err_t st_ret = mpu6050_self_test();
    if (st_ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 self-test failed — continuing (check sensor)");
    }

    /* Calibrate with 500 samples (~0.5 s) — pen must be stationary */
    ESP_ERROR_CHECK(mpu6050_calibrate(500));

    /* -----------------------------------------------------------------------
     * Step 2: ADC1 + FSR sensor array
     * ADC2 is NOT initialised — it is shared with the BLE radio hardware.
     * See fsr.h for the detailed explanation.
     * ----------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[2/4] Initialising FSR array on ADC1 "
                  "(Tip=GPIO%d GripA=GPIO%d GripB=GPIO%d) — ADC2 NOT touched",
             FSR_TIP_GPIO, FSR_GRIP_A_GPIO, FSR_GRIP_B_GPIO);
    ESP_ERROR_CHECK(fsr_init());
    ESP_ERROR_CHECK(fsr_calibrate_zero(200));

    /* -----------------------------------------------------------------------
     * Step 3: 1 kHz acquisition timer
     * Started AFTER sensor drivers are ready so the ISR never reads
     * uninitialised hardware.
     * ----------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[3/4] Starting 1 kHz GPTimer acquisition ISR");
    ESP_ERROR_CHECK(timer_init());

    /* -----------------------------------------------------------------------
     * Step 4: NimBLE stack
     * Initialised AFTER ADC1 is configured — NimBLE internally uses the
     * radio co-existence arbitration on ADC2 lines; if ADC2 were already
     * initialised this would cause a conflict.
     * ----------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[4/4] Initialising NimBLE BLE stack");
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    /* MTU: negotiate up to 256 bytes (packet is 53 bytes, well within range) */
    ble_att_set_preferred_mtu(256);

    /* Register GATT services */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(s_gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: rc=%d", rc);
        return;
    }

    /* Set device name for GAP */
    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed: rc=%d", rc);
        return;
    }

    /* -----------------------------------------------------------------------
     * Launch FreeRTOS tasks
     *
     *   dsp_task   → Core 1 (dedicated DSP core, priority 5)
     *   ble_tx_task → Core 0 (shared with BLE stack, priority 4)
     *   nimble host → Core 0 (created by nimble_port_freertos_init)
     * ----------------------------------------------------------------------- */
    xTaskCreatePinnedToCore(
        dsp_task,           /* Task function              */
        "dsp_task",         /* Name                       */
        8192,               /* Stack bytes                */
        NULL,               /* Parameters                 */
        5,                  /* Priority (higher = urgent) */
        NULL,               /* Task handle (unused)       */
        1                   /* Core 1                     */
    );

    xTaskCreatePinnedToCore(
        ble_tx_task,        /* Task function              */
        "ble_tx_task",      /* Name                       */
        4096,               /* Stack bytes                */
        NULL,               /* Parameters                 */
        4,                  /* Priority                   */
        NULL,               /* Task handle (unused)       */
        0                   /* Core 0                     */
    );

    /* NimBLE host task — must run on Core 0 alongside the BLE stack */
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "All tasks launched. System running.");

    /* app_main returns here — FreeRTOS scheduler continues running tasks */
}
