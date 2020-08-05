#include <string.h>
#include "syncd_sender_conn.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"

//TODO: find better solution
static uint8_t syncd_peer_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static volatile syncd_send_result_t send_result = SEND_NOT_FINISHED;

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
//receive_cb is not needed on the sender for now
static void receive_cb(const uint8_t * mac_addr, const uint8_t * data, int data_len){}
static void send_cb(const uint8_t * mac_addr, esp_now_send_status_t status){
	
	if(mac_addr == NULL){
		ESP_LOGE(TAG_ESPNOW, "NULL mac address in send callback");
		return;
	}
	if(status == ESP_NOW_SEND_FAIL)
		send_result = SEND_FAIL;
	else
		send_result = SEND_SUCCESS;
}

void syncd_sender_send(void * param){

	syncd_packet_t * packet = (syncd_packet_t *) param;
	
	ESP_ERROR_CHECK(esp_now_send(syncd_peer_mac, packet->buf, packet->len));
	while(send_result == SEND_NOT_FINISHED)
		continue;
	if(send_result == SEND_FAIL)
		ESP_LOGI(TAG_ESPNOW, "send fail");
	else
		ESP_LOGI(TAG_ESPNOW, "send success");
		
	send_result = SEND_NOT_FINISHED;
}

void syncd_sender_espnow_init(void){
	
	ESP_ERROR_CHECK(esp_now_init());
	ESP_ERROR_CHECK(esp_now_register_recv_cb(receive_cb));
	ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));
	
	esp_now_peer_info_t peer = {
		.lmk			= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.channel		= ESPNOW_CHANNEL,
		.ifidx			= ESP_IF_WIFI_STA,
		.encrypt		= 0,
		.priv			= NULL
	};
	memcpy(peer.peer_addr, syncd_peer_mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void syncd_sender_espnow_deinit(void){
	esp_now_deinit();
}

void syncd_sender_wifi_init(void){
	
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_handler, NULL));
	
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH)); //TODO: test ram storage for better performance
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, 0));
}

void syncd_sender_wifi_deinit(void){
	esp_wifi_stop();
	esp_wifi_deinit();
}
