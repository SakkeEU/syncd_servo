#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "syncd_connection.h"
#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

void syncd_task(void * pvParameters){
	
	volatile int64_t time = 0;
	uint8_t data[14] = {0};
	esp_err_t err;
	syncd_wifi_init();
	syncd_espnow_init();
	mpu6050_i2c_init();
	mpu6050_sync_default_init(MPU6050_ADDR0);
	
	mpu6050_show_config(MPU6050_ADDR0);
	vTaskDelay(3000 / portTICK_RATE_MS);

	mpu6050_offsets_init(MPU6050_ADDR0);
	mpu6050_get_offsets_8bit(MPU6050_ADDR0, data);
	syncd_packet_t p = {p.buf = data, .len = 14};
	syncd_packet_t * ptr_packet = &p;
	syncd_send((void *) ptr_packet);
	
	for(;;){
		time = esp_timer_get_time();
		for(uint8_t i = 0; i < 14; i++){
			data[i] = 0;
		}
		
		err = mpu6050_read_burst(MPU6050_ADDR0, RACCEL_XOUT_H, data, 14);
		if(err != 0)
			continue;
		syncd_packet_t p = {p.buf = data, .len = 14};
		syncd_packet_t * ptr_packet = &p;
		
		syncd_send((void *) ptr_packet);
		
		time = esp_timer_get_time() - time;
		printf("time: %ld us\n", (volatile long int)time);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

void app_main(void){
	// Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    xTaskCreate(*syncd_task, "syncd_task", 2048, NULL, 11, NULL);
    
    vTaskDelay(100000 / portTICK_RATE_MS);
	
}
