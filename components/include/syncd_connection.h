#ifndef _SYNCD_CONNECTION_
#define _SYNCD_CONNECTION_

#include <stdint.h>

#define TAG_WIFI "SYNCD_WIFI"
#define TAG_ESPNOW "SYNCD_ESPNOW"

#define ESPNOW_CHANNEL 1

typedef enum{
	SEND_FAIL,
	SEND_SUCCESS,
	SEND_NOT_FINISHED
}syncd_send_result_t;

typedef struct{
	uint8_t * buf;
	uint8_t len;
}syncd_packet_t;

void syncd_send(void * param);

void syncd_espnow_init();
void syncd_espnow_deinit();
void syncd_wifi_init();
void syncd_wifi_deinit();

#endif
