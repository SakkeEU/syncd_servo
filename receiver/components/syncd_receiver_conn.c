#include <string.h>
#include "syncd_receiver_conn.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"

static syncd_packet_t packet = {.buf = NULL, .len = 0};

static esp_err_t wifi_handler(void *ctx, system_event_t *event){
	
	switch(event->event_id){
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG_WIFI, " WIFI STARTED");
            break;
		default:
			break;
	}	
	return ESP_OK;
}

static void send_cb(const uint8_t * mac_addr, esp_now_send_status_t status){}
static void receive_cb(const uint8_t * mac_addr, const uint8_t * data, int data_len){
	
	if(mac_addr == NULL){
		ESP_LOGE(TAG_ESPNOW, "NULL mac address in receive callback");
		return;
	}
	
	packet.buf = malloc(data_len);
	if (packet.buf == NULL) {
        ESP_LOGE(TAG_ESPNOW, "Malloc receive data fail");
        return;
    }
    memcpy(packet.buf, data, data_len);
    packet.len = data_len;
  
	ESP_LOGD(TAG_ESPNOW, "new packet arrived\n");
	xSemaphoreGive(sem);
}

syncd_packet_t syncd_receiver_receive(void){
		
	syncd_packet_t ret;
	ret.buf = malloc(packet.len);
	if (ret.buf == NULL) {
        ESP_LOGE(TAG_ESPNOW, "Malloc receive data fail");
        return ret;
    }
	memcpy(ret.buf, packet.buf, packet.len);
	ret.len = packet.len;

	free(packet.buf);
	packet.len = 0;
	return ret;
}


void syncd_receiver_espnow_init(void){
	
	ESP_ERROR_CHECK(esp_now_init());
	ESP_ERROR_CHECK(esp_now_register_recv_cb(receive_cb));
	ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));
}

void syncd_receiver_espnow_deinit(void){
	esp_now_deinit();
}

void syncd_receiver_wifi_init(void){
	
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_handler, NULL));
	
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH)); //TODO: test ram storage for better performance
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, 0));
}

void syncd_receiver_wifi_deinit(void){
	esp_wifi_stop();
	esp_wifi_deinit();
}
