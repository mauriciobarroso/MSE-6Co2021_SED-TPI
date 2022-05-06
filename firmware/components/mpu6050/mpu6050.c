/**
  ******************************************************************************
  * @file           : mpu6050.c
  * @author         : Mauricio Barroso Benavides
  * @date           : May 2, 2022
  * @brief          : todo: write brief
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2022 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "esp_log.h"
#include "driver/i2c.h"

/* Private define ------------------------------------------------------------*/
/**
* @brief
*/
#define I2C_TIMEOUT_MS	(1000)

/**
* @brief
*/
#define ALPHA 0.99             /*!< Weight for gyroscope */
#define RAD_TO_DEG 57.27272727 /*!< Radians to degrees */

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char * TAG = "mpu6050";

/* Private function prototypes -----------------------------------------------*/
esp_err_t mpu6050_read_reg(mpu6050_t * const me, uint8_t reg_addr, uint8_t * data, size_t data_len);
esp_err_t mpu6050_write_reg(mpu6050_t * const me, uint8_t reg_addr, uint8_t * data);

/* Exported functions --------------------------------------------------------*/
esp_err_t mpu6050_init(mpu6050_t * const me, uint8_t dev_addr, i2c_port_t i2c_num, mpu6050_acce_fs_t acce_fs, mpu6050_gyro_fs_t gyro_fs) {
	ESP_LOGI(TAG, "Initializing %s component...", TAG);

	esp_err_t ret;

	/* Copy configuration */
	if(dev_addr >= 0xFF) {
		ESP_LOGE(TAG, "Device address invalid");

		return ESP_FAIL;
	}

	me->dev_addr = dev_addr;

	if(i2c_num > I2C_NUM_MAX - 1) {
		ESP_LOGE(TAG, "I2C port number invalid");

		return ESP_FAIL;
	}

	me->i2c_num = i2c_num;

	ret = mpu6050_wake_up(me);

    if (ret != ESP_OK) {
        return ret;
    }

	ret = mpu6050_set_acce_fs(me, acce_fs);

    if (ret != ESP_OK) {
        return ret;
    }

	ret = mpu6050_set_gyro_fs(me, gyro_fs);

	return ret;
}

esp_err_t mpu6050_get_device_id(mpu6050_t * const me, uint8_t * device_id) {
	esp_err_t ret;
    uint8_t data_tmp;

    ret = mpu6050_read_reg(me, MPU6050_WHO_AM_I, &data_tmp, 1);
    * device_id = data_tmp;

	return ret;
}

esp_err_t mpu6050_wake_up(mpu6050_t * const me) {
	esp_err_t ret;
    uint8_t data_tmp;

    ret = mpu6050_read_reg(me, MPU6050_PWR_MGMT_1, &data_tmp, 1);

    if (ret != ESP_OK) {
        return ret;
    }

    data_tmp &= (~BIT6);
    ret = mpu6050_write_reg(me, MPU6050_PWR_MGMT_1, &data_tmp);

	return ret;
}

esp_err_t mpu6050_sleep(mpu6050_t * const me) {
	esp_err_t ret;
    uint8_t data_tmp;

    ret = mpu6050_read_reg(me, MPU6050_PWR_MGMT_1, &data_tmp, 1);

    if (ret != ESP_OK) {
        return ret;
    }

    data_tmp |= BIT6;
    ret = mpu6050_write_reg(me, MPU6050_PWR_MGMT_1, &data_tmp);

	return ret;
}

esp_err_t mpu6050_set_acce_fs(mpu6050_t * const me, mpu6050_acce_fs_t acce_fs) {
	esp_err_t ret;
    uint8_t data_tmp;

    ret = mpu6050_read_reg(me, MPU6050_ACCEL_CONFIG, &data_tmp, 1);

    if (ret != ESP_OK) {
        return ret;
    }

    data_tmp &= (~BIT3);
    data_tmp &= (~BIT4);
    data_tmp |= (acce_fs << 3);

    ret = mpu6050_write_reg(me, MPU6050_ACCEL_CONFIG, &data_tmp);

	return ret;
}

esp_err_t mpu6050_set_gyro_fs(mpu6050_t * const me, mpu6050_gyro_fs_t gyro_fs) {
	esp_err_t ret;
    uint8_t data_tmp;

    ret = mpu6050_read_reg(me, MPU6050_GYRO_CONFIG, &data_tmp, 1);

    if (ret != ESP_OK) {
        return ret;
    }

    data_tmp &= (~BIT3);
    data_tmp &= (~BIT4);
    data_tmp |= (gyro_fs << 3);
    ret = mpu6050_write_reg(me, MPU6050_GYRO_CONFIG, &data_tmp);

	return ret;
}

esp_err_t mpu6050_get_acce_fs(mpu6050_t * const me, mpu6050_acce_fs_t * acce_fs) {
	esp_err_t ret;
    uint8_t data_tmp;

    ret = mpu6050_read_reg(me, MPU6050_ACCEL_CONFIG, &data_tmp, 1);
    data_tmp = (data_tmp >> 3) & 0x03;
    * acce_fs = data_tmp;

	return ret;
}

esp_err_t mpu6050_get_gyro_fs(mpu6050_t * const me, mpu6050_gyro_fs_t * gyro_fs) {
	esp_err_t ret;
    uint8_t data_tmp;

    ret = mpu6050_read_reg(me, MPU6050_GYRO_CONFIG, &data_tmp, 1);
    data_tmp = (data_tmp >> 3) & 0x03;
    * gyro_fs = data_tmp;

	return ret;
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_t * const me, float * acce_sensitivity) {
	esp_err_t ret;
    uint8_t acce_fs;

    ret = mpu6050_read_reg(me, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;

    switch(acce_fs) {
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

	return ret;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_t * const me, float * gyro_sensitivity) {
	esp_err_t ret;
    uint8_t gyro_fs;

    ret = mpu6050_read_reg(me, MPU6050_ACCEL_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;

    switch(gyro_fs) {
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

	return ret;
}

esp_err_t mpu6050_get_raw_acce(mpu6050_t * const me, mpu6050_raw_acce_value_t * raw_acce_value) {
	esp_err_t ret;
    uint8_t data_tmp[6] = {0};

    ret = mpu6050_read_reg(me, MPU6050_ACCEL_XOUT_H, data_tmp, 6);
    raw_acce_value->raw_acce_x = (int16_t)((data_tmp[0] << 8) + (data_tmp[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_tmp[2] << 8) + (data_tmp[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_tmp[4] << 8) + (data_tmp[5]));

	return ret;
}

esp_err_t mpu6050_get_raw_gyro(mpu6050_t * const me, mpu6050_raw_gyro_value_t * raw_gyro_value) {
	esp_err_t ret;
    uint8_t data_tmp[6] = {0};

    ret = mpu6050_read_reg(me, MPU6050_GYRO_XOUT_H, data_tmp, 6);
    raw_gyro_value->raw_gyro_x = (int16_t)((data_tmp[0] << 8) + (data_tmp[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_tmp[2] << 8) + (data_tmp[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_tmp[4] << 8) + (data_tmp[5]));

	return ret;
}

esp_err_t mpu6050_get_acce(mpu6050_t * const me, mpu6050_acce_value_t * acce_value) {
	esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;

    ret = mpu6050_get_acce_sensitivity(me, &acce_sensitivity);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = mpu6050_get_raw_acce(me, &raw_acce);

    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;

	return ret;
}

esp_err_t mpu6050_get_gyro(mpu6050_t * const me, mpu6050_gyro_value_t * gyro_value) {
	esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;

    ret = mpu6050_get_gyro_sensitivity(me, &gyro_sensitivity);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = mpu6050_get_raw_gyro(me, &raw_gyro);

    if (ret != ESP_OK) {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;

	return ret;
}

/* Private functions ---------------------------------------------------------*/
esp_err_t mpu6050_read_reg(mpu6050_t * const me, uint8_t reg_addr, uint8_t * data, size_t data_len) {
    return i2c_master_write_read_device(me->i2c_num, me->dev_addr, &reg_addr, 1, data, data_len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_write_reg(mpu6050_t * const me, uint8_t reg_addr, uint8_t * data) {
    uint8_t write_buf[2] = {reg_addr, * data};

    return i2c_master_write_to_device(me->i2c_num, me->dev_addr, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/***************************** END OF FILE ************************************/
