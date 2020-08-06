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
	float angle[3] = {0};
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
		
		int16_t gyro[3] = {0};
		uint16_t delta_t = 0;
		//int16_t accel[3] = {0};
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
		
		angle[0] += bias * (((float)gyro[0] * dt) / (float)gyro_sens);
		angle[1] += bias * (((float)gyro[1] * dt) / (float)gyro_sens);
		angle[2] += bias * (((float)gyro[2] * dt) / (float)gyro_sens);
		
		//experimental value to offset the loss of angle precision
		float angle2duty = 1500/90;
		int16_t duty = 1500 + (int16_t)(angle[2] * angle2duty);
		
		pwm_set_duty(0, duty);
		pwm_start();
		vTaskDelay(40 / portTICK_RATE_MS);
		pwm_stop(0);
		if(!(s%3)){
			printf("duty:%d\n", duty);
			printf("alpha:%.2f\n", angle[0]);
			printf("beta:%.2f\n", angle[1]);
			printf("gamma:%.2f\n", angle[2]);
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
