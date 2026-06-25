/**
 * @file mpu6050.h
 * @brief MPU6050 6-axis IMU driver for Parkinson's Smart Pen
 *
 * Hardware mapping:
 *   SDA  -> GPIO 8  (ESP32-S3, I2C port 0)
 *   SCL  -> GPIO 9  (ESP32-S3, I2C port 0)
 *   I2C address: 0x68 (AD0 pin pulled LOW)
 *
 * Provides:
 *   - Full register-map definitions
 *   - Initialisation with self-test verification
 *   - Non-blocking poll for raw accel/gyro axes
 *   - Static calibration offset storage
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

/* =========================================================================
 * Hardware pin definitions
 * ========================================================================= */
#define MPU6050_I2C_PORT        I2C_NUM_0   /**< ESP32-S3 I2C peripheral 0  */
#define MPU6050_PIN_SDA         8           /**< SDA -> GPIO 8               */
#define MPU6050_PIN_SCL         9           /**< SCL -> GPIO 9               */
#define MPU6050_I2C_FREQ_HZ     400000      /**< Fast-mode 400 kHz           */
#define MPU6050_I2C_TIMEOUT_MS  10          /**< Per-transaction timeout      */

/* =========================================================================
 * Device address
 * ========================================================================= */
#define MPU6050_ADDR            0x68        /**< AD0 = GND -> address 0x68   */

/* =========================================================================
 * Register map
 * ========================================================================= */

/* Configuration */
#define MPU6050_REG_SELF_TEST_X     0x0D
#define MPU6050_REG_SELF_TEST_Y     0x0E
#define MPU6050_REG_SELF_TEST_Z     0x0F
#define MPU6050_REG_SELF_TEST_A     0x10
#define MPU6050_REG_SMPLRT_DIV      0x19  /**< Sample Rate = Gyro/(1+SMPLRT_DIV) */
#define MPU6050_REG_CONFIG           0x1A  /**< DLPF and FSYNC config               */
#define MPU6050_REG_GYRO_CONFIG      0x1B  /**< Gyro full-scale select              */
#define MPU6050_REG_ACCEL_CONFIG     0x1C  /**< Accel full-scale select             */
#define MPU6050_REG_FIFO_EN          0x23
#define MPU6050_REG_INT_PIN_CFG      0x37
#define MPU6050_REG_INT_ENABLE       0x38
#define MPU6050_REG_INT_STATUS       0x3A

/* Sensor data - 14 bytes: ACCEL_XH..GZ_L */
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H       0x41
#define MPU6050_REG_TEMP_OUT_L       0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48

/* Power management */
#define MPU6050_REG_PWR_MGMT_1       0x6B  /**< Device reset & clock source */
#define MPU6050_REG_PWR_MGMT_2       0x6C
#define MPU6050_REG_WHO_AM_I         0x75  /**< Should read 0x68            */

/* =========================================================================
 * Configuration bit-fields
 * ========================================================================= */

/* PWR_MGMT_1 */
#define MPU6050_PWR_RESET           (1 << 7)  /**< Device reset              */
#define MPU6050_PWR_SLEEP           (1 << 6)  /**< Sleep mode enable         */
#define MPU6050_CLK_PLL_XGYRO       0x01       /**< X-axis gyro PLL (recommended) */

/* DLPF config (REG_CONFIG bits[2:0]) - sets bandwidth for both accel & gyro */
#define MPU6050_DLPF_BW_256         0x00   /**< Accel 260Hz, Gyro 256Hz    */
#define MPU6050_DLPF_BW_188         0x01   /**< Accel 184Hz, Gyro 188Hz    */
#define MPU6050_DLPF_BW_98          0x02   /**< Accel 94Hz,  Gyro 98Hz     */
#define MPU6050_DLPF_BW_42          0x03   /**< Accel 44Hz,  Gyro 42Hz     */
#define MPU6050_DLPF_BW_20          0x04   /**< Accel 21Hz,  Gyro 20Hz     */
/* Use BW_42 so hardware pre-filters far above the 6.5 Hz tremor band      */
#define MPU6050_DLPF_CFG            MPU6050_DLPF_BW_42

/* Accel full-scale (bits[4:3] of ACCEL_CONFIG) */
#define MPU6050_ACCEL_FS_2G         0x00   /**< ±2g  -> LSB/g = 16384      */
#define MPU6050_ACCEL_FS_4G         0x08   /**< ±4g  -> LSB/g = 8192       */
#define MPU6050_ACCEL_FS_SEL        MPU6050_ACCEL_FS_2G
#define MPU6050_ACCEL_SENSITIVITY   16384.0f  /**< LSB per g for ±2g range  */

/* Gyro full-scale (bits[4:3] of GYRO_CONFIG) */
#define MPU6050_GYRO_FS_250DPS      0x00   /**< ±250°/s -> LSB/(°/s) = 131 */
#define MPU6050_GYRO_FS_500DPS      0x08   /**< ±500°/s -> LSB/(°/s) = 65.5*/
#define MPU6050_GYRO_FS_SEL         MPU6050_GYRO_FS_250DPS
#define MPU6050_GYRO_SENSITIVITY    131.0f    /**< LSB per °/s for ±250°/s  */

/* Sample rate: SMPLRT_DIV = (Gyro_Rate / target) - 1
 * With DLPF enabled Gyro_Rate = 1000 Hz.
 * For 1 kHz output: SMPLRT_DIV = (1000/1000) - 1 = 0                     */
#define MPU6050_SMPLRT_DIV_1KHZ    0x00

/* =========================================================================
 * Data structures
 * ========================================================================= */

/** Raw 16-bit signed sensor readings directly from registers */
typedef struct {
    int16_t ax_raw;   /**< Accelerometer X [LSB]    */
    int16_t ay_raw;   /**< Accelerometer Y [LSB]    */
    int16_t az_raw;   /**< Accelerometer Z [LSB]    */
    int16_t gx_raw;   /**< Gyroscope X [LSB]        */
    int16_t gy_raw;   /**< Gyroscope Y [LSB]        */
    int16_t gz_raw;   /**< Gyroscope Z [LSB]        */
} mpu6050_raw_t;

/** Static calibration offsets (computed during mpu6050_calibrate()) */
typedef struct {
    int16_t cal_ax;
    int16_t cal_ay;
    int16_t cal_az;
    int16_t cal_gx;
    int16_t cal_gy;
    int16_t cal_gz;
} mpu6050_cal_t;

/** Computed orientation angles (degrees × 100, stored as int16 for BLE packing) */
typedef struct {
    int16_t pitch_x100;  /**< Pitch in 0.01° units */
    int16_t roll_x100;   /**< Roll  in 0.01° units */
} mpu6050_orient_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Install I2C driver and wake the MPU6050.
 *         Sets clock source, sample rate, DLPF, accel/gyro full-scale.
 * @return ESP_OK on success, or an esp_err_t on I2C/WHO_AM_I failure.
 */
esp_err_t mpu6050_init(void);

/**
 * @brief  Perform the MPU6050 hardware self-test sequence.
 *         Reads factory trim values and compares to measured response.
 * @return ESP_OK if all axes pass (<14% deviation), ESP_FAIL otherwise.
 */
esp_err_t mpu6050_self_test(void);

/**
 * @brief  Compute static bias offsets by averaging N samples at rest.
 *         Stores results in the driver-internal calibration struct.
 * @param  num_samples  Number of samples to average (recommend 500-1000).
 * @return ESP_OK on success.
 */
esp_err_t mpu6050_calibrate(uint16_t num_samples);

/**
 * @brief  Non-blocking burst read of all 6 raw axes (14-byte burst).
 *         Subtracts stored calibration offsets before returning.
 * @param[out] out  Pointer to mpu6050_raw_t to fill.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if I2C bus is busy.
 */
esp_err_t mpu6050_read_raw(mpu6050_raw_t *out);

/**
 * @brief  Compute pitch and roll from the latest accel reading.
 *         Uses atan2 approximation (no division, fast on Xtensa LX7).
 * @param[in]  raw    Calibrated raw readings.
 * @param[out] orient Computed orientation in ×100 degree units.
 */
void mpu6050_compute_orientation(const mpu6050_raw_t *raw,
                                  mpu6050_orient_t    *orient);

/**
 * @brief  Return a const pointer to the current calibration offsets.
 *         Used by main.c when packing the BLE frame.
 */
const mpu6050_cal_t *mpu6050_get_cal(void);

#endif /* MPU6050_H */
