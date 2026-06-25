/**
 * @file mpu6050.c
 * @brief MPU6050 driver implementation for Parkinson's Smart Pen
 *
 * I2C burst reads (14 bytes starting at 0x3B) are used for all sensor
 * data to ensure all axes are sampled at the same instant, avoiding
 * skew artefacts that would corrupt the FFT tremor analysis.
 *
 * Non-blocking strategy: i2c_master_cmd_begin() is called with a short
 * timeout (MPU6050_I2C_TIMEOUT_MS). If the bus is busy (e.g., previous
 * transaction still in flight from an ISR) the function returns
 * ESP_ERR_TIMEOUT immediately so the 1 kHz timer ISR can reschedule
 * rather than stall.
 */

#include "mpu6050.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

/* Driver-internal calibration storage */
static mpu6050_cal_t s_cal = {0};

/* =========================================================================
 * Low-level I2C helpers
 * ========================================================================= */

/**
 * @brief Write a single byte to an MPU6050 register.
 */
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg,  true);
    i2c_master_write_byte(cmd, val,  true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        MPU6050_I2C_PORT, cmd,
        pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS));

    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read one or more consecutive bytes from MPU6050 registers.
 *        Uses repeated-start (write address, then read) for atomic burst.
 */
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    /* Write phase: send device address + register pointer */
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    /* Repeated start + read phase */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ,  true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        MPU6050_I2C_PORT, cmd,
        pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS));

    i2c_cmd_link_delete(cmd);
    return ret;
}

/* =========================================================================
 * Public API implementation
 * ========================================================================= */

esp_err_t mpu6050_init(void)
{
    /* --- Install I2C master driver ---------------------------------------- */
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = MPU6050_PIN_SDA,
        .scl_io_num       = MPU6050_PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(MPU6050_I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(MPU6050_I2C_PORT,
                                       I2C_MODE_MASTER,
                                       0, 0, 0));

    /* Allow MPU6050 to power-up (datasheet: 30 ms after VDD stable) */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* --- Verify device identity ------------------------------------------- */
    uint8_t who_am_i = 0;
    ESP_ERROR_CHECK(mpu6050_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1));
    if (who_am_i != MPU6050_ADDR) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: expected 0x68, got 0x%02X", who_am_i);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MPU6050 detected (WHO_AM_I=0x%02X)", who_am_i);

    /* --- Reset device ----------------------------------------------------- */
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_RESET));
    vTaskDelay(pdMS_TO_TICKS(100));  /* Wait for reset to complete */

    /* --- Wake up, select PLL clock source --------------------------------- */
    /* PLL with X-axis gyroscope reference provides better frequency stability
     * than the internal 8 MHz oscillator, which is important for FFT accuracy */
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1,
                                       MPU6050_CLK_PLL_XGYRO));

    /* --- Sample rate = 1 kHz (required for 1024-point FFT window @ 1 s) -- */
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV,
                                       MPU6050_SMPLRT_DIV_1KHZ));

    /* --- DLPF at 42 Hz ----------------------------------------------------- */
    /* Hardware low-pass at 42 Hz eliminates aliasing well above the 6.5 Hz
     * tremor band. This also sets FSYNC to disabled (bits[5:3] = 000).       */
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG));

    /* --- Gyro full-scale: ±250 °/s ---------------------------------------- */
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG,
                                       MPU6050_GYRO_FS_SEL));

    /* --- Accel full-scale: ±2g -------------------------------------------- */
    /* ±2g gives maximum resolution (16384 LSB/g). Pen tremor accelerations
     * are well within this range (<0.5g peak in clinical studies).            */
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG,
                                       MPU6050_ACCEL_FS_SEL));

    ESP_LOGI(TAG, "MPU6050 initialised: 1kHz, DLPF=42Hz, ±2g, ±250dps");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

esp_err_t mpu6050_self_test(void)
{
    /* The MPU6050 self-test procedure (from register-map rev 4.2 §4.4):
     * 1. Read normal output (self-test disabled)
     * 2. Enable self-test on all axes, read self-test output
     * 3. Compute STR = (ST_output - normal_output)
     * 4. Read factory trim values (FT) from registers 0x0D-0x10
     * 5. Check: |STR - FT| / FT < 14% for each axis
     */

    uint8_t st_data[4] = {0};
    int16_t normal[6] = {0};
    int16_t st_out[6] = {0};
    uint8_t raw14[14] = {0};

    /* Step 1: read normal (no self-test) output averaged over 20 samples */
    int32_t sum[6] = {0};
    for (int i = 0; i < 20; i++) {
        ESP_ERROR_CHECK(mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw14, 14));
        sum[0] += (int16_t)((raw14[0]  << 8) | raw14[1]);
        sum[1] += (int16_t)((raw14[2]  << 8) | raw14[3]);
        sum[2] += (int16_t)((raw14[4]  << 8) | raw14[5]);
        sum[3] += (int16_t)((raw14[8]  << 8) | raw14[9]);
        sum[4] += (int16_t)((raw14[10] << 8) | raw14[11]);
        sum[5] += (int16_t)((raw14[12] << 8) | raw14[13]);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    for (int i = 0; i < 6; i++) normal[i] = (int16_t)(sum[i] / 20);

    /* Step 2: enable self-test on all axes (bits[7:5] of ACCEL/GYRO_CONFIG) */
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG,
                                       MPU6050_ACCEL_FS_SEL | 0xE0));
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG,
                                       MPU6050_GYRO_FS_SEL  | 0xE0));
    vTaskDelay(pdMS_TO_TICKS(20));  /* Settling time */

    memset(sum, 0, sizeof(sum));
    for (int i = 0; i < 20; i++) {
        ESP_ERROR_CHECK(mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw14, 14));
        sum[0] += (int16_t)((raw14[0]  << 8) | raw14[1]);
        sum[1] += (int16_t)((raw14[2]  << 8) | raw14[3]);
        sum[2] += (int16_t)((raw14[4]  << 8) | raw14[5]);
        sum[3] += (int16_t)((raw14[8]  << 8) | raw14[9]);
        sum[4] += (int16_t)((raw14[10] << 8) | raw14[11]);
        sum[5] += (int16_t)((raw14[12] << 8) | raw14[13]);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    for (int i = 0; i < 6; i++) st_out[i] = (int16_t)(sum[i] / 20);

    /* Step 3: restore normal config */
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL));
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG,  MPU6050_GYRO_FS_SEL));

    /* Step 4: read factory trim bytes */
    ESP_ERROR_CHECK(mpu6050_read_regs(MPU6050_REG_SELF_TEST_X, st_data, 4));

    /* Factory trim values decoded per register-map Table 6 */
    float ft_ax = (float)(st_data[0] >> 3);
    float ft_ay = (float)(st_data[1] >> 3);
    float ft_az = (float)(st_data[2] >> 3);
    /* Gyro bits: XG[4:0]=ST_X[4:0], YG[4:0]=ST_Y[4:0], ZG[4:0]=ST_Z[4:0] */
    float ft_gx = (float)(st_data[0] & 0x1F);
    float ft_gy = (float)(st_data[1] & 0x1F);
    float ft_gz = (float)(st_data[2] & 0x1F);

    /* Convert to expected self-test response (per MPU-6000/6050 Product
     * Specification rev 3.4, Section 4.1) */
    if (ft_ax != 0.0f) ft_ax = 4096.0f * 0.34f * powf(0.92f / 0.34f,
                                    (ft_ax - 1.0f) / 30.0f);
    if (ft_ay != 0.0f) ft_ay = 4096.0f * 0.34f * powf(0.92f / 0.34f,
                                    (ft_ay - 1.0f) / 30.0f);
    if (ft_az != 0.0f) ft_az = 4096.0f * 0.34f * powf(0.92f / 0.34f,
                                    (ft_az - 1.0f) / 30.0f);
    if (ft_gx != 0.0f) ft_gx =   25.0f * 131.0f * powf(1.046f,
                                    ft_gx - 1.0f);
    if (ft_gy != 0.0f) ft_gy =  -25.0f * 131.0f * powf(1.046f,
                                    ft_gy - 1.0f);
    if (ft_gz != 0.0f) ft_gz =   25.0f * 131.0f * powf(1.046f,
                                    ft_gz - 1.0f);

    /* Step 5: check deviation < 14% */
    float str[6];
    str[0] = (float)(st_out[0] - normal[0]);
    str[1] = (float)(st_out[1] - normal[1]);
    str[2] = (float)(st_out[2] - normal[2]);
    str[3] = (float)(st_out[3] - normal[3]);
    str[4] = (float)(st_out[4] - normal[4]);
    str[5] = (float)(st_out[5] - normal[5]);

    bool pass = true;
    const char *axis_names[6] = {"AX","AY","AZ","GX","GY","GZ"};
    float ft_vals[6] = {ft_ax, ft_ay, ft_az, ft_gx, ft_gy, ft_gz};

    for (int i = 0; i < 6; i++) {
        if (ft_vals[i] == 0.0f) continue;  /* Skip if factory trim = 0 */
        float pct = fabsf((str[i] - ft_vals[i]) / ft_vals[i]) * 100.0f;
        if (pct > 14.0f) {
            ESP_LOGE(TAG, "Self-test FAIL %s: deviation %.1f%%", axis_names[i], pct);
            pass = false;
        } else {
            ESP_LOGI(TAG, "Self-test OK   %s: deviation %.1f%%", axis_names[i], pct);
        }
    }
    return pass ? ESP_OK : ESP_FAIL;
}

/* -------------------------------------------------------------------------- */

esp_err_t mpu6050_calibrate(uint16_t num_samples)
{
    ESP_LOGI(TAG, "Calibrating MPU6050 (%u samples) - keep device stationary",
             num_samples);

    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint8_t raw14[14];

    for (uint16_t i = 0; i < num_samples; i++) {
        if (mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw14, 14) != ESP_OK) {
            continue;
        }
        sum_ax += (int16_t)((raw14[0]  << 8) | raw14[1]);
        sum_ay += (int16_t)((raw14[2]  << 8) | raw14[3]);
        sum_az += (int16_t)((raw14[4]  << 8) | raw14[5]);
        sum_gx += (int16_t)((raw14[8]  << 8) | raw14[9]);
        sum_gy += (int16_t)((raw14[10] << 8) | raw14[11]);
        sum_gz += (int16_t)((raw14[12] << 8) | raw14[13]);
        vTaskDelay(pdMS_TO_TICKS(1));  /* 1 ms -> 1 kHz paced sampling */
    }

    s_cal.cal_ax = (int16_t)(sum_ax / num_samples);
    s_cal.cal_ay = (int16_t)(sum_ay / num_samples);
    /* For AZ, expected value is +1g (16384 LSB) when pen is flat */
    s_cal.cal_az = (int16_t)(sum_az / num_samples) - (int16_t)MPU6050_ACCEL_SENSITIVITY;
    s_cal.cal_gx = (int16_t)(sum_gx / num_samples);
    s_cal.cal_gy = (int16_t)(sum_gy / num_samples);
    s_cal.cal_gz = (int16_t)(sum_gz / num_samples);

    ESP_LOGI(TAG, "Calibration complete: "
             "cal_ax=%d cal_ay=%d cal_az=%d cal_gx=%d cal_gy=%d cal_gz=%d",
             s_cal.cal_ax, s_cal.cal_ay, s_cal.cal_az,
             s_cal.cal_gx, s_cal.cal_gy, s_cal.cal_gz);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

esp_err_t mpu6050_read_raw(mpu6050_raw_t *out)
{
    uint8_t raw14[14];

    /* 14-byte burst: ACCEL_XH, XL, YH, YL, ZH, ZL, TMPH, TMPL,
     *                GYRO_XH, XL, YH, YL, ZH, ZL
     * Temperature bytes [6..7] are discarded.                                */
    esp_err_t ret = mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw14, 14);
    if (ret != ESP_OK) return ret;

    /* Reconstruct signed 16-bit values (big-endian from MPU6050) */
    int16_t ax = (int16_t)((raw14[0]  << 8) | raw14[1]);
    int16_t ay = (int16_t)((raw14[2]  << 8) | raw14[3]);
    int16_t az = (int16_t)((raw14[4]  << 8) | raw14[5]);
    int16_t gx = (int16_t)((raw14[8]  << 8) | raw14[9]);
    int16_t gy = (int16_t)((raw14[10] << 8) | raw14[11]);
    int16_t gz = (int16_t)((raw14[12] << 8) | raw14[13]);

    /* Apply calibration offsets */
    out->ax_raw = ax - s_cal.cal_ax;
    out->ay_raw = ay - s_cal.cal_ay;
    out->az_raw = az - s_cal.cal_az;
    out->gx_raw = gx - s_cal.cal_gx;
    out->gy_raw = gy - s_cal.cal_gy;
    out->gz_raw = gz - s_cal.cal_gz;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

void mpu6050_compute_orientation(const mpu6050_raw_t *raw,
                                  mpu6050_orient_t    *orient)
{
    /* Convert raw to g units */
    float ax_g = (float)raw->ax_raw / MPU6050_ACCEL_SENSITIVITY;
    float ay_g = (float)raw->ay_raw / MPU6050_ACCEL_SENSITIVITY;
    float az_g = (float)raw->az_raw / MPU6050_ACCEL_SENSITIVITY;

    /* Pitch: rotation around Y-axis (forward/back tilt of pen)
     * pitch = atan2(-ax, sqrt(ay² + az²))                                   */
    float pitch_rad = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

    /* Roll: rotation around X-axis (pen rotation left/right)
     * roll  = atan2(ay, az)                                                  */
    float roll_rad  = atan2f(ay_g, az_g);

    /* Convert to degrees, scale ×100 for integer BLE packing */
    orient->pitch_x100 = (int16_t)(pitch_rad * (180.0f / (float)M_PI) * 100.0f);
    orient->roll_x100  = (int16_t)(roll_rad  * (180.0f / (float)M_PI) * 100.0f);
}

/* -------------------------------------------------------------------------- */

const mpu6050_cal_t *mpu6050_get_cal(void)
{
    return &s_cal;
}
