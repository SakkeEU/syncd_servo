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
	
	int16_t offset[7];
	mpu6050_offsets_init(MPU6050_ADDR0);
	mpu6050_get_offsets(MPU6050_ADDR0, offset);
	printf("AccelOffs(X Y Z): %d %d %d\n", offset[0], offset[1], offset[2]);
	printf("GyroOffs(X Y Z): %d %d %d\n", offset[4], offset[5], offset[6]);
	printf("tempOffs: %d\n", offset[3]);
	
	for(;;){
		int16_t data[7];
		
		mpu6050_read_sensors(MPU6050_ADDR0, data, 1);
			
		printf("Accel(X Y Z): %d %d %d\n", data[0], data[1], data[2]);
		printf("Gyro(X Y Z): %d %d %d\n", data[4], data[5], data[6]);
		printf("tempRaws: %d\n", data[3]);
		
		
		
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

