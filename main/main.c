/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Mauricio Barroso Benavides
  * @date           : Apr 8, 2022
  * @brief          : Main program body
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
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "driver/i2c.h"
#include "mqtt_client.h"
#include "mpu6050.h"
#include "file_server.c"

/* Private includes ----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MPU6050_DEV_ADDR			(0x68)
#define I2C_MASTER_SCL_IO           (11)      	/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           (10)      	/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              (0)			/*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          (100000)	/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   (0)			/*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   (0)			/*!< I2C master doesn't need buffer */

#define WIFI_STA_SSID				"test"
#define WIFI_STA_PASS				"orcobebe"
#define WIFI_AP_SSID				"sd-tp"
#define WIFI_AP_MAX_CONN			(3)
#define MQTT_BROKER_URL				"mqtt://broker.hivemq.com:1883"
#define MQTT_STATE_TOPIC			"state"
#define MQTT_INCOMING_DATA_TOPIC	"incoming_data"

#define IMU_SAMPLING_RATE_HZ		(100.0)
#define IMU_SAMPLING_RATE_MS		((1 / IMU_SAMPLING_RATE_HZ) * 1000)

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Tag for debug */
static const char * TAG = "app";

/* Application variables */
static mpu6050_t mpu6050;
static esp_mqtt_client_handle_t mqtt_client;
static TaskHandle_t get_imu_data_handle = NULL;
static char incoming_mqtt_data[256];

/* Private function prototypes -----------------------------------------------*/
/* Initialization functions */
static esp_err_t nvs_init(void);
static esp_err_t wifi_init(void);
static esp_err_t mqtt_init(void);
static esp_err_t i2c_master_init(void);
static esp_err_t spiffs_init(void);

/* Event handlers */
static void wifi_event_handler(void * arg, esp_event_base_t event_base,
		int32_t event_id, void * event_data);

static void ip_event_handler(void * arg, esp_event_base_t event_base,
		int32_t event_id, void * event_data);

static void mqtt_event_handler(void * arg, esp_event_base_t event_base,
		int32_t event_id, void * event_data);

/* RTOS tasks */
void get_imu_data_task(void * arg);

/* Private user code ---------------------------------------------------------*/
/* main */
void app_main(void) {
	/* Modules initialization */
    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(wifi_init());
    ESP_ERROR_CHECK(mqtt_init());
    ESP_ERROR_CHECK(file_server_init());
    ESP_ERROR_CHECK(spiffs_init());
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(mpu6050_init(&mpu6050, MPU6050_DEV_ADDR, I2C_MASTER_NUM, ACCE_FS_4G, GYRO_FS_250DPS));

    ESP_LOGI(TAG, "sampling rate in ms is %d", (int)IMU_SAMPLING_RATE_MS);

    /* RTOS tasks creation */
}

/* Initialization functions */
static esp_err_t nvs_init(void) {
	esp_err_t ret;

	ESP_LOGI(TAG, "Initializing NVS...");

    ret = nvs_flash_init();

    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    	ret = nvs_flash_erase();

    	if(ret == ESP_OK) {
    		ret = nvs_flash_init();

    		if(ret != ESP_OK) {
    			ESP_LOGE(TAG, "Error initializing NVS");

    			return ret;
    		}
    	}
    	else {
    		ESP_LOGE(TAG, "Error erasing NVS");

    		return ret;
    	}

    }

	return ret;
}

static esp_err_t wifi_init(void) {
	esp_err_t ret;

	ESP_LOGI(TAG, "Initializing Wi-Fi...");

    /* Initialize stack TCP/IP */
    ret = esp_netif_init();

    if(ret != ESP_OK) {
    	return ret;
    }

    /* Create event loop */
    ret = esp_event_loop_create_default();

    if(ret != ESP_OK) {
    	return ret;
    }

    /* Create netif instances */
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    /* Declare event handler instances for Wi-Fi and IP */
    esp_event_handler_instance_t instance_any_wifi;
    esp_event_handler_instance_t instance_got_ip;

    /* Register Wi-Fi, IP and SmartConfig event handlers */
    ret = esp_event_handler_instance_register(WIFI_EVENT,
    										  ESP_EVENT_ANY_ID,
											  &wifi_event_handler,
											  NULL,
											  &instance_any_wifi);

    if(ret != ESP_OK) {
    	return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT,
    										  IP_EVENT_STA_GOT_IP,
											  &ip_event_handler,
											  NULL,
											  &instance_got_ip);

    if(ret != ESP_OK) {
    	return ret;
    }

    /* Initialize Wi-Fi driver */
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&wifi_config);

    if(ret != ESP_OK) {
    	return ret;
    }

    /* Set Wi-Fi AP and STA configurations */
    wifi_config_t wifi_conf = {0};

	strcpy((char *)wifi_conf.ap.ssid , WIFI_AP_SSID);
	wifi_conf.ap.ssid_len = (strlen(WIFI_AP_SSID));
    wifi_conf.ap.max_connection = WIFI_AP_MAX_CONN;
    wifi_conf.ap.authmode = WIFI_AUTH_OPEN;

    ret = esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_conf);

    if(ret != ESP_OK) {
		ESP_LOGE(TAG, "2");
		return ret;
	}

	strcpy((char *)wifi_conf.sta.ssid , WIFI_STA_SSID);
	strcpy((char *)wifi_conf.sta.password , WIFI_STA_PASS);

    ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_conf);

    if(ret != ESP_OK) {
		ESP_LOGE(TAG, "2");
		return ret;
	}

    ret = esp_wifi_set_mode(WIFI_MODE_APSTA);

    if(ret != ESP_OK) {
    	ESP_LOGE(TAG, "1");
    	return ret;
    }


    ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_conf);

    /* Start Wi-Fi */
    ret = esp_wifi_start();

    if(ret != ESP_OK) {
    	ESP_LOGE(TAG, "3");
    	return ret;
    }

    ret = esp_wifi_connect();

    return ret;
}

static esp_err_t mqtt_init(void) {
	esp_err_t ret;

	ESP_LOGI(TAG, "Initializing MQTT...");

	/* Fill MQTT client configuration and initialize */
	/* todo: implement certificates and keys */
	esp_mqtt_client_config_t mqtt_config = {
			.uri = MQTT_BROKER_URL,
			.client_cert_pem = NULL,
			.client_key_pem = NULL,
			.cert_pem = NULL,
#ifdef CONFIG_MQTT_LWT_ENABLE
			.lwt_topic = CONFIG_MQTT_LWT_TOPIC,
			.lwt_msg = CONFIG_MQTT_LWT_MESSAGE,
			.lwt_msg_len = CONFIG_MQTT_LWT_LENGHT,
			.lwt_qos = CONFIG_MQTT_LWT_QOS,
#endif
			.user_context = NULL
	};

	mqtt_client = esp_mqtt_client_init(&mqtt_config);

	/* Register MQTT event handler */
	if(mqtt_client != NULL) {
		ret = esp_mqtt_client_register_event(mqtt_client,
				MQTT_EVENT_ANY,
				mqtt_event_handler,
				NULL);

		if(ret != ESP_OK) {
			return ret;
		}
	}
	else {
		return ESP_FAIL;
	}

	return ret;
}

static esp_err_t i2c_master_init(void) {
	ESP_LOGI(TAG, "Initializing I2C...");

	esp_err_t ret;

    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(i2c_master_port, &conf);

    if(ret != ESP_OK) {
    	ESP_LOGE(TAG, "Failed configuring I2C parameters");

    	return ret;
    }

    ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    return ret;
}

static esp_err_t spiffs_init(void) {
	esp_err_t ret;

    ESP_LOGI(TAG, "Initializing SPIFFS...");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    ret = esp_vfs_spiffs_register(&conf);

    if(ret != ESP_OK) {
        if(ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if(ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }

        return ret ;
    }

    size_t total = 0, used = 0;

    ret = esp_spiffs_info(conf.partition_label, &total, &used);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));

        return ret;
    }
    else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

	return ret;
}

/* Event handlers */
static void wifi_event_handler(void * arg, esp_event_base_t event_base,
							   int32_t event_id, void * event_data) {
	switch(event_id) {
		case WIFI_EVENT_STA_START: {
			ESP_LOGI(TAG, "WIFI_EVENT_STA_START");

			break;
		}

		case WIFI_EVENT_STA_CONNECTED: {
			ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");

			break;
		}

		case WIFI_EVENT_STA_DISCONNECTED: {
			ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");

	        	break;
		}

		default:
			ESP_LOGI(TAG, "Other Wi-Fi event");

			break;
	}
}

static void ip_event_handler(void * arg, esp_event_base_t event_base,
						     int32_t event_id, void * event_data) {
	switch(event_id) {
		case IP_EVENT_STA_GOT_IP: {
			ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP");

			/* Start MQTT client */
			esp_mqtt_client_start(mqtt_client);

			break;
		}

		default: {
			ESP_LOGI(TAG, "Other IP event");

			break;
		}
	}
}

static void mqtt_event_handler(void * arg, esp_event_base_t event_base,
		int32_t event_id, void * event_data) {
	esp_mqtt_event_handle_t event = event_data;

	switch(event_id) {
		case MQTT_EVENT_CONNECTED: {
			ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

			/* Publish to user defined alive topic */
			if(esp_mqtt_client_publish(mqtt_client, MQTT_STATE_TOPIC, "connected", 0, 0, 0) == -1) {
				ESP_LOGE(TAG, "Failed publishing to topic");
			}
			else {
				ESP_LOGI(TAG, "Sent publish successful to %s topic", MQTT_STATE_TOPIC);
			}


			/* Subscribe to user defined valve state and ota notification topics */
			if(esp_mqtt_client_subscribe(mqtt_client, MQTT_INCOMING_DATA_TOPIC, 0) == -1) {
				ESP_LOGE(TAG, "Failed subscribing to topic");
			}
			else {
				ESP_LOGI(TAG, "Sent subscribe successful to %s topic", MQTT_INCOMING_DATA_TOPIC);
			}

			break;
		}

		case MQTT_EVENT_DISCONNECTED: {
			ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");

			break;
		}

		case MQTT_EVENT_DATA: {
			ESP_LOGI(TAG, "MQTT_EVENT_DATA");

			/* Copy to buffer and print data */
			sprintf(incoming_mqtt_data, "%.*s", event->data_len, event->data);
			ESP_LOGI(TAG, "data=%s", incoming_mqtt_data);

			/* Check if the array is a number */
			if(atoi(incoming_mqtt_data)) {
				int get_data_time = atoi(incoming_mqtt_data);

				if(get_data_time > 0) {
					if(get_data_time > 60) {
						get_data_time = 60;
					}

					ESP_LOGI(TAG, "time=%d", get_data_time);
					sprintf(incoming_mqtt_data, "%d", get_data_time);

					/* Create task to get IMU data */
					if(get_imu_data_handle == NULL) {
						if(xTaskCreate(get_imu_data_task,
								"Get IMU data Task",
								configMINIMAL_STACK_SIZE * 4,
								incoming_mqtt_data,
								tskIDLE_PRIORITY + 1,
								&get_imu_data_handle) != pdPASS) {
							ESP_LOGE(TAG, "Failed creating RTOS task");
						}
					}
				}
			}
			/* Check if the array is the stop command */
			else if(!strcmp(incoming_mqtt_data, "stop")) {
				ESP_LOGI(TAG, "stop command");
			}
			/* Other messages */
			else {
				ESP_LOGW(TAG, "Message not supported");
			}

			break;
		}

		default:
			ESP_LOGI(TAG, "Other MQTT event");
			break;
	}
}

/* RTOS tasks */
void get_imu_data_task(void * arg) {
	int get_data_time = atoi((const char *)arg);
	ESP_LOGI(TAG, "get data time in seconds is %d", get_data_time);
	TickType_t last_time_wake = xTaskGetTickCount();
	int samples_count = 0;

	mpu6050_acce_value_t acce;
	mpu6050_gyro_value_t gyro;

    ESP_LOGI(TAG, "Opening file");

    FILE * f = fopen("/spiffs/data.csv", "w");

    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

	fprintf(f, "timestamp,gyro_x,gyro_y,gyro_z\n");

	ESP_LOGI(TAG, "Getting data...");

	for(;;) {
		mpu6050_get_acce(&mpu6050, &acce);
		mpu6050_get_gyro(&mpu6050, &gyro);
		printf("gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

		fprintf(f, "%d,%.2f,%.2f,%.2f\n", xTaskGetTickCount(), gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);


		vTaskDelayUntil(&last_time_wake, pdMS_TO_TICKS((TickType_t)IMU_SAMPLING_RATE_MS));

		if(++samples_count >= (int)((float)get_data_time / (1 / IMU_SAMPLING_RATE_HZ))) {
			ESP_LOGI(TAG, "Samples quantity reached. Closing file and deleting task...");
			fclose(f);
			get_imu_data_handle = NULL;

			break;
		}
	}

	vTaskDelete(NULL);
}
