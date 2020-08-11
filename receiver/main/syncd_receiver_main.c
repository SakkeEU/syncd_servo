#include <math.h>
#include "syncd_receiver_conn.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/pwm.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define PERIOD 20000
#define INIT_DUTY 1500
#define CHANNEL_NUM 1
#define GPIO_PIN 2

SemaphoreHandle_t sem;

//TODO: Due to the semaphore syncing receive() and task() some packets are lost
//on each iteration, as a result the angles computated by the receiver are 
//shorter than the angles measured by the mpu. Solutions:
//1. Faster receiver routine (cant be done for now due to hardware limitations).
//2. Slower sender.
//3. Make receiver update the servo position less often.
void syncd_receiver_task(void * pvParam){
	
	syncd_receiver_wifi_init();
	syncd_receiver_espnow_init();
	
	//start-stop the servo to stop it from shaking. This is a hack-ish
	//solution, a better circuit might solve the problem.
	uint32_t period = PERIOD;	
	uint32_t init_duty[1] = {INIT_DUTY};
	uint8_t channel_num = CHANNEL_NUM;
	const uint32_t pin_num[1] = {GPIO_PIN};
	pwm_init(period, init_duty, channel_num, pin_num);
	pwm_set_phase(0, 0);
	pwm_start();
	vTaskDelay(50 / portTICK_RATE_MS);
	pwm_stop(0);
	
	//take the sempaphore so it's already empty on the first iteration
	sem = xSemaphoreCreateBinary();
	xSemaphoreTake(sem, 1);
	
	uint32_t s = 0;
	float gyro_angle[3] = {0};
	syncd_packet_t packet;
	for(;;){
		while(xSemaphoreTake(sem, 2000 / portTICK_RATE_MS) == pdFALSE){
			printf("waiting packet\n");
			continue;
		}
		
		packet = syncd_receiver_receive();
		if(packet.buf == NULL)
			continue;
		
		int16_t gyro_sens = 131;
		int16_t accel_sens = 32768/2;
		
		uint16_t delta_t = 0;
		int16_t gyro[3] = {0};
		int16_t accel[3] = {0};
		accel[0] = packet.buf[0] << 8 | packet.buf[1];
		accel[1] = packet.buf[2] << 8 | packet.buf[3];
		accel[2] = packet.buf[4] << 8 | packet.buf[5];
		gyro[0] = packet.buf[8] << 8 | packet.buf[9];
		gyro[1] = packet.buf[10] << 8 | packet.buf[11];
		gyro[2] = packet.buf[12] << 8 | packet.buf[13];
		delta_t = packet.buf[14] << 8 | packet.buf[15];
		free(packet.buf);
		
		float accel_f[3] = {
			(float)accel[0]/(float)accel_sens,
			(float)accel[1]/(float)accel_sens,
			(float)accel[2]/(float)accel_sens
		};
		float accel_abs_sum = fabsf(accel_f[0]) + fabsf(accel_f[1]) + fabsf(accel_f[2]);
		float accel_norm = sqrtf(powf(accel_f[0], 2) + powf(accel_f[1], 2) + powf(accel_f[2], 2));
		float accel_angle[3] = {0};
		
		float abs_sum_min = 1.2;
		float abs_sum_max = 1.6;
		if(accel_abs_sum > abs_sum_min && accel_abs_sum < abs_sum_max){
			accel_angle[0] = atan2f(-accel_f[0], sqrtf(powf(accel_f[1], 2) + powf(accel_f[2], 2))) * 57.3; //180/3.14159 
			accel_angle[1] = atan2f(accel_f[1], accel_f[2]) * 57.3;
		}
		
		float dt = ((float)(delta_t))/1000000;
		float bias = 0.96;
		
		//Two of these are useless for now
		gyro_angle[0] += (((float)gyro[0] * dt) / (float)gyro_sens);
		gyro_angle[1] += (((float)gyro[1] * dt) / (float)gyro_sens);
		gyro_angle[2] += (((float)gyro[2] * dt) / (float)gyro_sens);
		
		//if the angle is not close to 0 or 90° and we are not moving
		//accel_angle is more precise than gyro_angle
		float angle = 0;
		if(accel_abs_sum > abs_sum_min && accel_abs_sum < abs_sum_max)
			angle = (1.0 - bias) * gyro_angle[1] + bias * accel_angle[0];
		else
			angle = bias * gyro_angle[1] + (1.0 - bias) * accel_angle[0];
			
		//very simple anti drift solution
		if(accel_abs_sum - accel_f[2] < 0.1){
			gyro_angle[0] = bias * gyro_angle[0];
			gyro_angle[1] = bias * gyro_angle[1];
			gyro_angle[2] = bias * gyro_angle[2];
		}
		
		//experimental value to offset the loss of angle precision
		float angle2duty = 1000/90;
		int16_t duty = 1500 + (int16_t)(angle * angle2duty);
		
		pwm_set_duty(0, duty);
		pwm_start();
		vTaskDelay(30 / portTICK_RATE_MS);
		pwm_stop(0);
		
		if(!(s%10)){
			printf("accel_abs_sum:%.2f\n", accel_abs_sum);
			printf("accel_norm:%.2f\n", accel_norm);
			printf("accel_angle[0]:%.2f\n", accel_angle[0]);
			printf("gyro_angle[1]:%.2f\n", gyro_angle[1]);
			printf("angle:%.2f\n", angle);
			printf("***\n");
		}
		s++;
	}
}

void app_main(void){
	
	ESP_ERROR_CHECK(nvs_flash_init());
	
	xTaskCreate(*syncd_receiver_task, "syncd_receiver_task", 2048, NULL, 11, NULL);
	vTaskDelay(100000 / portTICK_RATE_MS);
}
