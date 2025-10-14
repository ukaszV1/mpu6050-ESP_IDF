#include <stdio.h>
#include <math.h>
#include <time.h>
#include "string.h"
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"

#define RETURN_ON_ERROR(x) do {        \
    esp_err_t __err_rc = (x);          \
    if (__err_rc != ESP_OK) return __err_rc; \
} while (0)

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define ALPHA                       0.99f        /*!< Weight of gyroscope */
#define RAD_TO_DEG                  57.27272727f /*!< Radians to degrees */

/* MPU6050 register */
#define MPU6050_GYRO_CONFIG         0x1Bu
#define MPU6050_ACCEL_CONFIG        0x1Cu
#define MPU6050_INTR_PIN_CFG        0x37u
#define MPU6050_INTR_ENABLE         0x38u
#define MPU6050_INTR_STATUS         0x3Au
#define MPU6050_ACCEL_XOUT_H        0x3Bu
#define MPU6050_GYRO_XOUT_H         0x43u
#define MPU6050_TEMP_XOUT_H         0x41u
#define MPU6050_PWR_MGMT_1          0x6Bu
#define MPU6050_WHO_AM_I            0x75u

#define MPU6050_I2C_TIMEOUT         100
#define MPU6050_I2C_DEFAULT_FREQ    100000


const uint8_t MPU6050_DATA_RDY_INT_BIT =      (uint8_t) BIT0;
const uint8_t MPU6050_I2C_MASTER_INT_BIT =    (uint8_t) BIT3;
const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT = (uint8_t) BIT4;
const uint8_t MPU6050_MOT_DETECT_INT_BIT =    (uint8_t) BIT6;
const uint8_t MPU6050_ALL_INTERRUPTS = (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT | MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT);

static esp_err_t mpu6050_write(mpu6050_handle_t *handle, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    uint8_t buf[data_len + 1];
    buf[0] = reg_start_addr;
    memcpy(&buf[1], data_buf, data_len);
    RETURN_ON_ERROR(i2c_master_transmit(handle->i2c_master_dev_handle, buf, data_len+1, MPU6050_I2C_TIMEOUT));
    return ESP_OK;
}

static esp_err_t mpu6050_read(mpu6050_handle_t *handle, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    RETURN_ON_ERROR(i2c_master_transmit_receive(handle->i2c_master_dev_handle, (uint8_t[]){reg_start_addr},1,data_buf,data_len, MPU6050_I2C_TIMEOUT));
    return ESP_OK;
}


esp_err_t mpu6050_init(mpu6050_handle_t *handle, i2c_master_bus_handle_t *i2c_bus, uint8_t address){
    CHECK_ARG(handle && i2c_bus && address);
    memset(handle, 0, sizeof(mpu6050_handle_t));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = MPU6050_I2C_DEFAULT_FREQ
    };
    //add device to bus
    RETURN_ON_ERROR(i2c_master_bus_add_device(*i2c_bus, &dev_cfg, &handle->i2c_master_dev_handle));
    handle->counter = 0;
    handle->dt = 0;
    handle->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    handle->i2c_master_bus_handle = *i2c_bus;
    return ESP_OK;
}


esp_err_t mpu6050_get_deviceid(mpu6050_handle_t *handle, uint8_t *const deviceid)
{
    return mpu6050_read(handle, MPU6050_WHO_AM_I, deviceid, 1);
}

esp_err_t mpu6050_wake_up(mpu6050_handle_t *handle)
{
    uint8_t tmp;
    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_PWR_MGMT_1, &tmp, 1));
    tmp &= (~BIT6);
    return mpu6050_write(handle, MPU6050_PWR_MGMT_1, &tmp, 1);
}

esp_err_t mpu6050_sleep(mpu6050_handle_t *handle)
{
    uint8_t tmp;
    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_PWR_MGMT_1, &tmp, 1));
    tmp |= BIT6;
    return mpu6050_write(handle, MPU6050_PWR_MGMT_1, &tmp, 1);
    
}

esp_err_t mpu6050_config(mpu6050_handle_t *handle, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3,  acce_fs << 3};
    return mpu6050_write(handle, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t *handle, float *const acce_sensitivity)
{
    uint8_t acce_fs;
    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_ACCEL_CONFIG, &acce_fs, 1));
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ESP_OK;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t *handle, float *const gyro_sensitivity)
{
    uint8_t gyro_fs;
    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_GYRO_CONFIG, &gyro_fs, 1));
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
        
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ESP_OK;
}

esp_err_t mpu6050_config_interrupts(mpu6050_handle_t *handle, const mpu6050_int_config_t *const interrupt_configuration)
{
    CHECK_ARG(interrupt_configuration);

    if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin)) {
        // Set GPIO connected to MPU6050 INT pin only when user configures interrupts.
        handle->int_pin = interrupt_configuration->interrupt_pin;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t int_pin_cfg = 0x00;

    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1));

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        int_pin_cfg |= BIT7;
    }
    if (INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode) {
        int_pin_cfg |= BIT6;
    }
    if (INTERRUPT_LATCH_UNTIL_CLEARED == interrupt_configuration->interrupt_latch) {
        int_pin_cfg |= BIT5;
    }
    if (INTERRUPT_CLEAR_ON_ANY_READ == interrupt_configuration->interrupt_clear_behavior) {
        int_pin_cfg |= BIT4;
    }

    RETURN_ON_ERROR(mpu6050_write(handle, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1));
    gpio_int_type_t gpio_intr_type;

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        gpio_intr_type = GPIO_INTR_NEGEDGE;
    } else {
        gpio_intr_type = GPIO_INTR_POSEDGE;
    }

    gpio_config_t int_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = gpio_intr_type,
        .pin_bit_mask = (1ul << interrupt_configuration->interrupt_pin)
    };

    return gpio_config(&int_gpio_config);
}

esp_err_t mpu6050_register_isr(mpu6050_handle_t *handle, const mpu6050_isr_t isr)
{
    CHECK_ARG(handle);

    RETURN_ON_ERROR(gpio_isr_handler_add(handle->int_pin,
        ((gpio_isr_t) * (isr)),((void *) handle)));

    return gpio_intr_enable(handle->int_pin);
}

esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t *handle, uint8_t interrupt_sources)
{
    uint8_t enabled_interrupts = 0x00;
    RETURN_ON_ERROR (mpu6050_read(handle, MPU6050_INTR_ENABLE, &enabled_interrupts, 1));

    if (enabled_interrupts != interrupt_sources) {
        enabled_interrupts |= interrupt_sources;
        RETURN_ON_ERROR(mpu6050_write(handle, MPU6050_INTR_ENABLE, &enabled_interrupts, 1));
    }
    return ESP_OK;
}

esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t *handle, uint8_t interrupt_sources)
{
    uint8_t enabled_interrupts = 0x00;
    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_INTR_ENABLE, &enabled_interrupts, 1));

    if (0 != (enabled_interrupts & interrupt_sources)) {
        enabled_interrupts &= (~interrupt_sources);
        RETURN_ON_ERROR(mpu6050_write(handle, MPU6050_INTR_ENABLE, &enabled_interrupts, 1));
    }
    return ESP_OK;
}

esp_err_t mpu6050_get_interrupt_status(mpu6050_handle_t *handle, uint8_t *const out_intr_status)
{
    CHECK_ARG(out_intr_status);
    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_INTR_STATUS, out_intr_status, 1));
    return ESP_OK;
}

inline uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status)
{
    return (MPU6050_DATA_RDY_INT_BIT == (MPU6050_DATA_RDY_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status)
{
    return (uint8_t) (MPU6050_I2C_MASTER_INT_BIT == (MPU6050_I2C_MASTER_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status)
{
    return (uint8_t) (MPU6050_FIFO_OVERFLOW_INT_BIT == (MPU6050_FIFO_OVERFLOW_INT_BIT & interrupt_status));
}

esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t *handle)
{
    uint8_t data_rd[6];
    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd)));
    handle->acce_raw_value.raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    handle->acce_raw_value.raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    handle->acce_raw_value.raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ESP_OK;
}

esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t *handle)
{
    uint8_t data_rd[6];
    RETURN_ON_ERROR(mpu6050_read(handle, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd)));
    handle->gyro_raw_value.raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    handle->gyro_raw_value.raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    handle->gyro_raw_value.raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ESP_OK;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t *handle)
{
    float acce_sensitivity;
    RETURN_ON_ERROR(mpu6050_get_acce_sensitivity(handle, &acce_sensitivity));
    RETURN_ON_ERROR(mpu6050_get_raw_acce(handle));
    handle->acce_value.acce_x = handle->acce_raw_value.raw_acce_x / acce_sensitivity;
    handle->acce_value.acce_y = handle->acce_raw_value.raw_acce_y / acce_sensitivity;
    handle->acce_value.acce_z = handle->acce_raw_value.raw_acce_z/ acce_sensitivity;
    return ESP_OK;
}
esp_err_t mpu6050_get_gyro(mpu6050_handle_t *handle)
{
    float gyro_sensitivity;
    RETURN_ON_ERROR(mpu6050_get_gyro_sensitivity(handle, &gyro_sensitivity));
    RETURN_ON_ERROR(mpu6050_get_raw_gyro(handle));
    handle->gyro_value.gyro_x = handle->gyro_raw_value.raw_gyro_x / gyro_sensitivity;
    handle->gyro_value.gyro_y = handle->gyro_raw_value.raw_gyro_y / gyro_sensitivity;
    handle->gyro_value.gyro_z = handle->gyro_raw_value.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_temp(mpu6050_handle_t *handle)
{
    uint8_t data_rd[2];
    esp_err_t ret = mpu6050_read(handle, MPU6050_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    handle->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53;
    return ret;
}

void mpu6050_complimentory_filter(mpu6050_handle_t *handle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    handle->counter++;
    if (handle->counter == 1) {
        acce_angle[0] = (atan2(handle->acce_value.acce_y, handle->acce_value.acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(handle->acce_value.acce_x, handle->acce_value.acce_z) * RAD_TO_DEG);
        handle->complimentary_angle.roll = acce_angle[0];
        handle->complimentary_angle.pitch = acce_angle[1];
        gettimeofday(handle->timer, NULL);
        return;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, handle->timer, &dt_t);
    handle->dt = (float) (dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(handle->timer, NULL);

    acce_angle[0] = (atan2(handle->acce_value.acce_y, handle->acce_value.acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(handle->acce_value.acce_x, handle->acce_value.acce_z) * RAD_TO_DEG);

    gyro_rate[0] = handle->gyro_value.gyro_x;
    gyro_rate[1] = handle->gyro_value.gyro_y;
    gyro_angle[0] = gyro_rate[0] * handle->dt;
    gyro_angle[1] = gyro_rate[1] * handle->dt;

    handle->complimentary_angle.roll = (ALPHA * (handle->complimentary_angle.roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    handle->complimentary_angle.pitch = (ALPHA * (handle->complimentary_angle.pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);
    return;
}
