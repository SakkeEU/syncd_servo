#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "syncd_sender_conn.h"
#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define SENDER_MAIN_TAG "SENDER_MAIN"

static inline void check_error(esp_err_t err){
	if(err)
		ESP_LOGI(SENDER_MAIN_TAG,"error:%d\n", err);
}

void syncd_task(void * pvParameters){
	
	syncd_sender_wifi_init();
	syncd_sender_espnow_init();
	check_error(mpu6050_i2c_init());
	check_error(mpu6050_sync_default_init(MPU6050_ADDR0));

	check_error(mpu6050_show_config(MPU6050_ADDR0));
	
	check_error(mpu6050_offsets_init(MPU6050_ADDR0));
	
	volatile int64_t time_past = 0;
	volatile int64_t time_curr = 0;
	volatile uint16_t time_delta = 0;
	uint8_t data[16] = {0};
	
	esp_err_t err;
	for(;;){
		err = mpu6050_read_burst(MPU6050_ADDR0, RACCEL_XOUT_H, data, 14);
		if(err != 0){
			ESP_LOGI(SENDER_MAIN_TAG,"error:%d\n", err);
			continue;
		}
		time_curr = esp_timer_get_time();
		if(time_past == 0 || time_curr < time_past){
			time_past = time_curr;
			continue;
		}

		time_delta = (uint16_t)(time_curr - time_past);
		time_past = time_curr;
		data[14] = (time_delta >> 8) & 0xFF;
		data[15] = time_delta & 0xFF;

		syncd_packet_t p = {p.buf = data, .len = 16};
		syncd_packet_t * ptr_packet = &p;
		syncd_sender_send((void *) ptr_packet);
		
		vTaskDelay(40 / portTICK_RATE_MS);
	}
}

void app_main(void){
	// Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    xTaskCreate(*syncd_task, "syncd_task", 2048, NULL, 11, NULL);
    
    vTaskDelay(100000 / portTICK_RATE_MS);
	
}
