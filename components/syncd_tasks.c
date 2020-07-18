#include <stdint.h>
#include <stdio.h>
#include "syncd_tasks.h"
#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"

void mpu6050_task(void * pvParameters){
	
	mpu6050_i2c_init();
	mpu6050_sync_default_init(MPU6050_ADDR0);
	
	mpu6050_show_config(MPU6050_ADDR0);
	vTaskDelay(3000 / portTICK_RATE_MS);
	
	for(;;){
		uint8_t len = 12;
		uint8_t data_temp[len];
		int16_t data_raw[len/2];
		//volatile int64_t time;
		
		//time = (volatile int64_t)esp_timer_get_time();
		mpu6050_read(MPU6050_ADDR0, RACCEL_XOUT_H, data_temp, len/2);
		mpu6050_read(MPU6050_ADDR0, RGYRO_XOUT_H, (data_temp + len/2), len/2);
		
		//printf("time for reads: %ld us\n", (volatile long) (esp_timer_get_time() - time));
		
		for(uint8_t i = 0; i < len - 1; i += 2)
			data_raw[i/2] = (data_temp[i] << 8) | data_temp[i + 1];
			
		printf("AccelRaw: X%d Y%d Z%d\n", data_raw[0], data_raw[1], data_raw[2]);
		printf("GyroRaw: X%d Y%d Z%d\n", data_raw[3], data_raw[4], data_raw[5]);
		
		//vTaskDelay(200 / portTICK_RATE_MS);
	}
}

