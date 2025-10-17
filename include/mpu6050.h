#pragma once
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define MPU6050_I2C_ADDRESS         0x68u /*!< I2C address with AD0 pin low */
#define MPU6050_I2C_ADDRESS_1       0x69u /*!< I2C address with AD0 pin high */
#define MPU6050_WHO_AM_I_VAL        0x68u 

typedef enum {
    MPU6050_ACCE_FS_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
    MPU6050_ACCE_FS_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
    MPU6050_ACCE_FS_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
    MPU6050_ACCE_FS_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
} mpu6050_acce_fs_t;

typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per sencond */
    MPU6050_GYRO_FS_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    MPU6050_GYRO_FS_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    MPU6050_GYRO_FS_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
} mpu6050_gyro_fs_t;

typedef enum {
    MPU6050_INTR_PIN_ACTIVE_H = 0,          /*!< The mpu6050 sets its INT pin HIGH on interrupt */
    MPU6050_INTR_PIN_ACTIVE_L  = 1           /*!< The mpu6050 sets its INT pin LOW on interrupt */
} mpu6050_int_pin_active_level_t;

typedef enum {
    MPU6050_INTR_PUSH_PULL   = 0,          /*!< The mpu6050 configures its INT pin as push-pull */
    MPU6050_INTR_OD  = 1           /*!< The mpu6050 configures its INT pin as open drain*/
} mpu6050_int_pin_mode_t;

typedef enum {
    MPU6050_INTR_LATCH_50            = 0,    /*!< The mpu6050 produces a 50 microsecond pulse on interrupt */
    MPU6050_INTR_LATCH_UNTIL_CLEARED   = 1     /*!< The mpu6050 latches its INT pin to its active level, until interrupt is cleared */
} mpu6050_int_latch_t;

typedef enum {
    MPU6050_INTR_CLEAR_ON_ANY     = 0,    /*!< INT_STATUS register bits are cleared on any register read */
    MPU6050_INTR_CLEAR_ON_STATUS  = 1     /*!< INT_STATUS register bits are cleared only by reading INT_STATUS value*/
} mpu6050_int_clear_t;

extern const uint8_t MPU6050_INTR_DATA_RDY;      /*!< DATA READY interrupt bit               */
extern const uint8_t MPU6050_INTR_I2C_MASTER;    /*!< I2C MASTER interrupt bit               */
extern const uint8_t MPU6050_INTR_FIFO_OVERFLOW; /*!< FIFO Overflow interrupt bit            */
extern const uint8_t MPU6050_INTR_MOT_DETECT;    /*!< MOTION DETECTION interrupt bit         */
extern const uint8_t MPU6050_INTR_ALL;        /*!< All interrupts supported by mpu6050    */

typedef struct {
    gpio_num_t interrupt_pin;                       /*!< GPIO connected to mpu6050 INT pin       */
    gpio_config_t int_gpio_config;                  
    mpu6050_int_pin_active_level_t active_level;    /*!< Active level of mpu6050 INT pin         */
    mpu6050_int_pin_mode_t pin_mode;                /*!< Push-pull or open drain mode for INT pin*/
    mpu6050_int_latch_t interrupt_latch;            /*!< The interrupt pulse behavior of INT pin */
    mpu6050_int_clear_t interrupt_clear_behavior;   /*!< Interrupt status clear behavior         */
    uint8_t int_source_bitmask;
    struct{
        uint8_t mot_thr;
        uint8_t mot_dur;
        uint8_t zmot_thr;
        uint8_t zmot_dur;
        uint8_t ff_thr;
        uint8_t ff_dur;
    }treshold_cfg;
    struct{
        uint8_t ff_cnt  :2;
        uint8_t mot_cnt :2;
        uint8_t acc_del :2;
    }treshold_cnt;
    
} mpu6050_intr_config_t;

typedef struct {
    uint16_t dev_addr;
    i2c_device_config_t i2c_device_config;
    i2c_master_dev_handle_t i2c_master_dev_handle;
    i2c_master_bus_handle_t i2c_master_bus_handle;
    mpu6050_intr_config_t intr_config;
    float temp;

    struct 
    {
        uint32_t counter;
        float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
        struct timeval *timer;
        float roll;
        float pitch;
    }complimentary_filter;
    
    struct{
        float gyro_x;
        float gyro_y;
        float gyro_z;
    }gyro_value;
    struct{
        int16_t raw_gyro_x;
        int16_t raw_gyro_y;
        int16_t raw_gyro_z;
    }gyro_raw;
    struct {
        float acce_x;
        float acce_y;
        float acce_z;
    }acce_value;
    struct{
        int16_t raw_acce_x;
        int16_t raw_acce_y;
        int16_t raw_acce_z;
    }acce_raw;
} mpu6050_handle_t;

/**
 * @brief  inint mpu6050
 * @param  handle mpu6050 handle
 * @param  i2c_bus bus to attatch device to
 * @param  adderess I2C device address 
 * @return ESP_OK Success
 */
esp_err_t mpu6050_init(mpu6050_handle_t *handle, i2c_master_bus_handle_t *i2c_bus, uint8_t address);

/**
 * @brief Get id
 * @param handle 
 * @param deviceid a pointer of device ID
 * @return ESP_OK Success
 */
esp_err_t mpu6050_get_deviceid(mpu6050_handle_t *handle, uint8_t *const deviceid);

/**
 * @brief Wake up mpu6050
 * @param handle mpu6050 handle
 * @return ESP_OK Success
 */
esp_err_t mpu6050_wake_up(mpu6050_handle_t *handle);

/**
 * @brief Enter sleep mode
 * @param handle mpu6050 handle
 * @return - ESP_OK Success
 */
esp_err_t mpu6050_sleep(mpu6050_handle_t *handle);

/**
 * @brief Set accelerometer and gyroscope full scale range
 * @param handle mpu6050 handle
 * @param acce_fs accelerometer full scale range
 * @param gyro_fs gyroscope full scale range
 * @return - ESP_OK Success
 */
esp_err_t mpu6050_config(mpu6050_handle_t *handle, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs);

/**
 * @brief Get accelerometer sensitivity
 * @param handle mpu6050 handle
 * @param acce_sensitivity accelerometer sensitivity
 * @return - ESP_OK Success
 */
esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t *handle, float *const acce_sensitivity);

/**
 * @brief Get gyroscope sensitivity
 * @param handle mpu6050 handle
 * @param gyro_sensitivity gyroscope sensitivity
 * @return - ESP_OK Success
 */

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t *handle, float *const gyro_sensitivity);

/**
 * @brief Configure INT pin behavior and setup target GPIO.
 * @param handle mpu6050 handle
 * @return - ESP_OK Success
 *         - ESP_ERR_INVALID_ARG A parameter is NULL or incorrect
 *         - ESP_FAIL Failed to configure INT pin on mpu6050
 */

esp_err_t mpu6050_intr_config(mpu6050_handle_t *handle);

/**
 * @brief Register an Interrupt Service Routine to handle mpu6050 interrupts.
 * @param handle mpu6050 handle
 * @param isr function to handle interrupts produced by mpu6050
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to register ISR
 */
esp_err_t mpu6050_intr_isr_add(mpu6050_handle_t *handle, const gpio_isr_t isr);

/**
 * @brief Enable specific interrupts from mpu6050
 * @param handle mpu6050 handle
 * @param interrupt_sources bit mask with interrupt sources to enable
 * This function does not disable interrupts not set in interrupt_sources. To disable
 * specific mpu6050 interrupts, use mpu6050_intr_disable().
 * To enable all mpu6050 interrupts, pass MPU6050_INTR_ALL as the argument
 * for interrupt_sources.
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
 */
esp_err_t mpu6050_intr_src_enable(mpu6050_handle_t *handle);

/**
 * @brief Disable specific interrupts from mpu6050
 * @param handle mpu6050 handle
 * @param interrupt_sources bit mask with interrupt sources to disable
 * This function does not enable interrupts not set in interrupt_sources. To enable
 * specific mpu6050 interrupts, use mpu6050_intr_src_enable().
 * To disable all mpu6050 interrupts, pass MPU6050_INTR_ALL as the
 * argument for interrupt_sources.
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
 */
esp_err_t mpu6050_intr_disable(mpu6050_handle_t *handle);

/**
 * @brief Get the interrupt status of mpu6050
 * @param handle mpu6050 handle
 * @param out_intr_status[out] bit mask that is assigned a value representing the interrupts triggered by the mpu6050
 *
 * This function can be used by the mpu6050 ISR to determine the source of
 * the mpu6050 interrupt that it is handling.
 *
 * After this function returns, the bits set in out_intr_status are
 * the sources of the latest interrupt triggered by the mpu6050. For example,
 * if MPU6050_INTR_DATA_RDY is set in out_intr_status, the last interrupt
 * from the mpu6050 was a DATA READY interrupt.
 *
 * The behavior of the INT_STATUS register of the mpu6050 may change depending on
 * the value of mpu6050_int_clear_t used on interrupt configuration.
 *
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to retrieve interrupt status from mpu6050
 */
esp_err_t mpu6050_intr_get_status(mpu6050_handle_t *handle, uint8_t *const out_intr_status);

/**
 * @brief Determine if the last mpu6050 interrupt was due to data ready.
 * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_intr_get_status()
 * @return
 *      - 0: The interrupt was not produced due to data ready
 *      - Any other positive integer: Interrupt was a DATA_READY interrupt
 */
extern uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status);

/**
 * @brief Determine if the last mpu6050 interrupt was an I2C master interrupt.
 * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_intr_get_status()
 * @return
 *      - 0: The interrupt is not an I2C master interrupt
 *      - Any other positive integer: Interrupt was an I2C master interrupt
 */
extern uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status);

/**
 * @brief Determine if the last mpu6050 interrupt was triggered by a fifo overflow.
 * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_intr_get_status()
 * @return
 *      - 0: The interrupt is not a fifo overflow interrupt
 *      - Any other positive integer: Interrupt was triggered by a fifo overflow
 */
extern uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status);

/**
 * @brief Read raw accelerometer measurements
 * @param handle mpu6050 handle
 * @param raw_acce_value raw accelerometer measurements
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t *handle);

/**
 * @brief Read raw gyroscope measurements
 * @param handle mpu6050 handle
 * @param raw_gyro_value raw gyroscope measurements
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t *handle);

/**
 * @brief Read accelerometer measurements
 * @param handle mpu6050 handle
 * @param acce_value accelerometer measurements
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_acce(mpu6050_handle_t *handle);

/**
 * @brief Read gyro values
 * @param handle mpu6050 handle
 * @param gyro_value gyroscope measurements
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_gyro(mpu6050_handle_t *handle);

/**
 * @brief Read temperature values
 * @param handle mpu6050 handle
 * @param temp_value temperature measurements
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6050_get_temp(mpu6050_handle_t *handle);

/**
 * @brief Use complimentory filter to calculate roll and pitch
 * @param handle mpu6050 handle
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complimentary_angle complimentary angle
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
void mpu6050_complimentory_filter(mpu6050_handle_t *handle);

