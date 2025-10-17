
#include <stdio.h>
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 21     /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO 22      /*!< GPIO number for I2C master data */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050_app";

static void mpu6050_task(void *arg)
{
    mpu6050_handle_t mpu6050;
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    ESP_ERROR_CHECK(mpu6050_init(&mpu6050, &i2c_bus, MPU6050_I2C_ADDRESS));
    ESP_ERROR_CHECK(mpu6050_config(&mpu6050, MPU6050_ACCE_FS_2G, MPU6050_GYRO_FS_500DPS));
    ESP_ERROR_CHECK(mpu6050_wake_up(&mpu6050));

    uint8_t device_id;
    if (mpu6050_get_deviceid(&mpu6050, &device_id) == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02X", device_id);
    } else {
        ESP_LOGE(TAG, "Failed to read device ID");
    }



    while (1) {
        if (mpu6050_get_acce(&mpu6050) == ESP_OK) {
            ESP_LOGI(TAG, "ACCEL -> %f, %f , %f",
                    mpu6050.acce_value.acce_x, mpu6050.acce_value.acce_y, mpu6050.acce_value.acce_z);
        }

        if (mpu6050_get_gyro(&mpu6050) == ESP_OK) {
                    ESP_LOGI(TAG, "gyro -> %f, %f ,%f",
                    mpu6050.gyro_value.gyro_x,  mpu6050.gyro_value.gyro_y,  mpu6050.gyro_value.gyro_z);
        }

        if (mpu6050_get_temp(&mpu6050) == ESP_OK) {
            ESP_LOGI(TAG, "TEMP -> %.2f °C", mpu6050.temp);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // read every second
    }
    i2c_driver_delete(I2C_MASTER_NUM);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MPU6050 example...");
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
}
