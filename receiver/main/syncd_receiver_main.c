#include "syncd_receiver_conn.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "driver/pwm.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define PERIOD 20000
#define INIT_DUTY 1500
#define CHANNEL_NUM 1
#define GPIO_PIN 2

void syncd_receiver_task(void * pvParam){
	
	syncd_receiver_wifi_init();
	syncd_receiver_espnow_init();
	
	uint32_t init_duty = INIT_DUTY;
	const uint32_t pin_num = GPIO_PIN;
	pwm_init(PERIOD, &init_duty, CHANNEL_NUM, &pin_num);
	pwm_start();
	
	syncd_packet_t packet;
	
	for(;;){
		packet = syncd_receiver_receive();
		if(packet.buf == NULL)
			continue;
		
		int16_t gyro_sens = 131;
		
		int16_t gyro[3] = {0};
		int16_t accel[3] = {0};
		uint16_t delta_t = 0;
		accel[0] = packet.buf[0] << 8 | packet.buf[1];
		accel[1] = packet.buf[2] << 8 | packet.buf[3];
		accel[2] = packet.buf[4] << 8 | packet.buf[5];
		gyro[0] = packet.buf[8] << 8 | packet.buf[9];
		gyro[1] = packet.buf[10] << 8 | packet.buf[11];
		gyro[2] = packet.buf[12] << 8 | packet.buf[13];
		delta_t = packet.buf[14] << 8 | packet.buf[15];
		free(packet.buf);
		
		float dt = delta_t/1000000;
		float bias = 0.96;
		float angle[3] = {0};
		
		
		angle[0] += bias * ((gyro[0] * dt) / (float)gyro_sens);
		angle[1] += bias * ((gyro[1] * dt) / (float)gyro_sens);
		angle[2] += bias * ((gyro[2] * dt) / (float)gyro_sens);
		
		float angle2duty = 5.5555;
		int16_t duty = 1500 + (int16_t)(angle[0] * angle2duty);
		pwm_stop(0x01);
		pwm_set_duty(0, duty);
		pwm_start();
		printf("***\n");
		printf("***\n");
		vTaskDelay(10 / portTICK_RATE_MS);

	}
}

void app_main(void){
	
	ESP_ERROR_CHECK(nvs_flash_init());
	
	xTaskCreate(*syncd_receiver_task, "syncd_receiver_task", 2048, NULL, 6, NULL);
	vTaskDelay(100000 / portTICK_RATE_MS);
}
