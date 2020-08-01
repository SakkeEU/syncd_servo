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

#define SENDER_MAIN_TAG "SENDER_MAIN"

static inline void check_error(esp_err_t err){
	if(err)
		ESP_LOGI(SENDER_MAIN_TAG,"error:%d\n", err);
}

void syncd_task(void * pvParameters){
	
	volatile int64_t time = 0;
	uint8_t data[14] = {0};
	syncd_wifi_init();
	syncd_espnow_init();
	check_error(mpu6050_i2c_init());
	check_error(mpu6050_sync_default_init(MPU6050_ADDR0));

	check_error(mpu6050_show_config(MPU6050_ADDR0));
	vTaskDelay(3000 / portTICK_RATE_MS);
	
	//check_error(mpu6050_read_burst(MPU6050_ADDR0, RXA_OFFS_USRH, data, 6));
	//check_error(mpu6050_read_burst(MPU6050_ADDR0, RXG_OFFS_USRH, (data+6), 6));
	
	//int16_t a1,a2,a3,a4,a5,a6;
	//a1 = (data[0] << 8) | data[1];
	//a2 = (data[2] << 8) | data[3];
	//a3 = (data[4] << 8) | data[5];
	//a4 = (data[6] << 8) | data[7];
	//a5 = (data[8] << 8) | data[9];
	//a6 = (data[10] << 8) | data[11];
	//printf("%d\n", a1);
	//printf("%d\n", a2);
	//printf("%d\n", a3);
	//printf("%d\n", a4);
	//printf("%d\n", a5);
	//printf("%d\n", a6);
	
	check_error(mpu6050_offsets_init(MPU6050_ADDR0));

	//check_error(mpu6050_read_burst(MPU6050_ADDR0, RXA_OFFS_USRH, data, 6));
	//check_error(mpu6050_read_burst(MPU6050_ADDR0, RXG_OFFS_USRH, (data+6), 6));
	
	//a1 = (data[0] << 8) | data[1];
	//a2 = (data[2] << 8) | data[3];
	//a3 = (data[4] << 8) | data[5];
	//a4 = (data[6] << 8) | data[7];
	//a5 = (data[8] << 8) | data[9];
	//a6 = (data[10] << 8) | data[11];
	//printf("%d\n", a1);
	//printf("%d\n", a2);
	//printf("%d\n", a3);
	//printf("%d\n", a4);
	//printf("%d\n", a5);
	//printf("%d\n", a6);
	vTaskDelay(5000 / portTICK_RATE_MS);
	
	esp_err_t err;
	for(;;){
		time = esp_timer_get_time();
		for(uint8_t i = 0; i < 14; i++){
			data[i] = 0;
		}
		
		err = mpu6050_read_burst(MPU6050_ADDR0, RACCEL_XOUT_H, data, 14);
		if(err != 0){
			ESP_LOGI(SENDER_MAIN_TAG,"error:%d\n", err);
			continue;
		}
		syncd_packet_t p = {p.buf = data, .len = 14};
		syncd_packet_t * ptr_packet = &p;
		
		syncd_send((void *) ptr_packet);
		
		//printf("%d\n", (int16_t)((data[0] << 8) | data[1]));
		//printf("%d\n", (int16_t)((data[2] << 8) | data[3]));
		//printf("%d\n", (int16_t)((data[4] << 8) | data[5]));
		//printf("%d\n", (int16_t)((data[8] << 8) | data[9]));
		//printf("%d\n", (int16_t)((data[10] << 8) | data[11]));
		//printf("%d\n", (int16_t)((data[12] << 8) | data[13]));
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
