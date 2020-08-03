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
static inline void float2array(float f, uint8_t * array){
	
	uint asint = * ((uint *)&f);
	
	uint8_t i;
	for(i = 0; i < 4; i++)
		array[i] = (asint >> (8 * i)) & 0xFF;
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
	volatile int64_t time_delta = 0;
	float dt = 0;
	int16_t data[14] = {0};
	float angle[3] = {0};
	
	float bias = 0.96;
	//float accel_sens = 16384.0;
	float gyro_sens = 131.0;
	int s = 0;
	esp_err_t err;
	for(;;){
		//err = mpu6050_read_burst(MPU6050_ADDR0, RACCEL_XOUT_H, data, 14);
		err = mpu6050_read_sensors(MPU6050_ADDR0, data);
		if(err != 0){
			ESP_LOGI(SENDER_MAIN_TAG,"error:%d\n", err);
			continue;
		}
		time_curr = esp_timer_get_time();
		if(time_past == 0 || time_curr < time_past){
			time_past = time_curr;
			continue;
		}
		
		time_delta = (time_curr - time_past)/2;
		dt = (time_delta / 1000000.0)*2;
		angle[0] += bias * ((data[4] * dt) / gyro_sens);
		angle[1] += bias * ((data[5] * dt) / gyro_sens);
		angle[2] += bias * ((data[6] * dt) / gyro_sens);
		
		//if(!(s%30)){
			//printf("\ndt: %f\n", dt);
			//printf("time_delta: %ld us\n", (volatile long int)time_delta);
			//printf("time: %ld us\n", (volatile long int)(time_curr - time_past));
			//printf("\n****\n");
			//printf("gx%.2f\n", angle[0]);
			//printf("gy%.2f\n", angle[1]);
			//printf("gz%.2f\n", angle[2]);
		//}
		time_past = time_curr;
		s++;
		
		uint8_t send[12] = {0};
		float2array(angle[0], send);
		float2array(angle[1], send+4);
		float2array(angle[2], send+8);
		syncd_packet_t p = {p.buf = send, .len = 12};
		syncd_packet_t * ptr_packet = &p;
		syncd_send((void *) ptr_packet);
		
		vTaskDelay(20 / portTICK_RATE_MS);
	}
}

void app_main(void){
	// Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    xTaskCreate(*syncd_task, "syncd_task", 2048, NULL, 11, NULL);
    
    vTaskDelay(100000 / portTICK_RATE_MS);
	
}
