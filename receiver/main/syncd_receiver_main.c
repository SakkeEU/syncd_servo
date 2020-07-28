#include "syncd_receiver_conn.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"


void syncd_receiver_task(void * pvParam){
	
	syncd_receiver_wifi_init();
	syncd_receiver_espnow_init();
	
	syncd_packet_t packet;
	
	for(;;){
		packet = syncd_receiver_receive();
		if(packet.buf == NULL)
			continue;
		
		uint8_t data_len = packet.len/2;	
		int16_t data[data_len];
		for(uint8_t i = 0; i < data_len; i++)
			data[i] = 0;
		
		for(uint8_t i = 0; i < packet.len - 1; i+=2)
			data[i/2] = (packet.buf[i] << 8) | packet.buf[i+1];
		free(packet.buf);
		
		printf("***\n");
		for(uint8_t i = 0; i < data_len; i++){
			printf("%d\n", data[i]);
		}
		printf("***\n");

	}
}

void app_main(void){
	
	ESP_ERROR_CHECK(nvs_flash_init());
	
	xTaskCreate(*syncd_receiver_task, "syncd_receiver_task", 2048, NULL, 6, NULL);
	vTaskDelay(100000 / portTICK_RATE_MS);
}
