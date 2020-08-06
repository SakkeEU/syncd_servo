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

void syncd_receiver_task(void * pvParam){
	
	syncd_receiver_wifi_init();
	syncd_receiver_espnow_init();
	
	uint32_t period = PERIOD;	
	uint32_t init_duty[1] = {INIT_DUTY};
	uint8_t channel_num = CHANNEL_NUM;
	const uint32_t pin_num[1] = {GPIO_PIN};
	pwm_init(period, init_duty, channel_num, pin_num);
	pwm_set_phase(0, 0);
	pwm_start();
	//vTaskDelay(2000 / portTICK_RATE_MS);
	//pwm_stop(0);
	//pwm_set_duty(0, 1000);
	//printf("1000\n");
	//pwm_start();
	//vTaskDelay(2000 / portTICK_RATE_MS);
	//pwm_stop(0);
	//pwm_set_duty(0, 1200);
	//printf("1200\n");
	//pwm_start();
	//vTaskDelay(2000 / portTICK_RATE_MS);
	//pwm_stop(0);
	//pwm_set_duty(0, 1400);
	//printf("1400\n");
	//pwm_start();
	//vTaskDelay(2000 / portTICK_RATE_MS);
	//pwm_stop(0);
	//pwm_set_duty(0, 1600);
	//printf("1600\n");
	//pwm_start();
	//vTaskDelay(2000 / portTICK_RATE_MS);
	//pwm_stop(0);
	//pwm_set_duty(0, 1800);
	//printf("1800\n");
	//pwm_start();
	//vTaskDelay(2000 / portTICK_RATE_MS);
	//pwm_stop(0);
	//pwm_set_duty(0, 2000);
	//printf("2000\n");
	//pwm_start();
	//vTaskDelay(2000 / portTICK_RATE_MS);
	//pwm_stop(0);
	//pwm_set_duty(0, 1500);
	//printf("1500\n");
	//pwm_start();
	
	sem = xSemaphoreCreateBinary();
	xSemaphoreTake(sem, 1);
	syncd_packet_t packet;
	uint32_t s = 0;
	for(;;){
		while(xSemaphoreTake(sem, 2000 / portTICK_RATE_MS) == pdFALSE){
			printf("waiting packet\n");
			continue;
		}
		
		packet = syncd_receiver_receive();
		if(packet.buf == NULL)
			continue;
		
		int16_t gyro_sens = 131;
		
		int16_t gyro[3] = {0};
		//int16_t accel[3] = {0};
		uint16_t delta_t = 0;
		//accel[0] = packet.buf[0] << 8 | packet.buf[1];
		//accel[1] = packet.buf[2] << 8 | packet.buf[3];
		//accel[2] = packet.buf[4] << 8 | packet.buf[5];
		gyro[0] = packet.buf[8] << 8 | packet.buf[9];
		gyro[1] = packet.buf[10] << 8 | packet.buf[11];
		gyro[2] = packet.buf[12] << 8 | packet.buf[13];
		delta_t = packet.buf[14] << 8 | packet.buf[15];
		free(packet.buf);
		
		float dt = ((float)(delta_t))/1000000;
		float bias = 0.96;
		float angle[3] = {0};
		
		
		angle[0] += bias * (((float)gyro[0] * dt) / (float)gyro_sens);
		angle[1] += bias * (((float)gyro[1] * dt) / (float)gyro_sens);
		angle[2] += bias * (((float)gyro[2] * dt) / (float)gyro_sens);
		
		float angle2duty = 5.5555;
		int16_t duty = 1500 + (int16_t)(angle[0] * angle2duty);
		pwm_stop(0x01);
		pwm_set_duty(0, duty);
		pwm_start();
		if(!(s%5)){
			printf("***\n");
			printf("gx:%d\n", gyro[0]);
			printf("gy:%d\n", gyro[1]);
			printf("gz:%d\n", gyro[2]);
			printf("delta_t:%d\n", delta_t);
			printf("dt:%.2f\n", dt);
			printf("*\n");
			printf("duty:%d\n", duty);
			printf("alpha:%.2f\n", angle[0]);
			printf("beta:%.2f\n", angle[1]);
			printf("gamma:%.2f\n", angle[2]);
			printf("***\n");
		}
		//vTaskDelay(20 / portTICK_RATE_MS);
		s++;
	}
}

void app_main(void){
	
	ESP_ERROR_CHECK(nvs_flash_init());
	
	xTaskCreate(*syncd_receiver_task, "syncd_receiver_task", 2048, NULL, 11, NULL);
	vTaskDelay(100000 / portTICK_RATE_MS);
}
