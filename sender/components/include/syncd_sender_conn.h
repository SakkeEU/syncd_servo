#ifndef _SYNCD_SENDER_CONN_
#define _SYNCD_SENDER_CONN_

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

void syncd_sender_send(void * param);

void syncd_sender_espnow_init(void);
void syncd_sender_espnow_deinit(void);
void syncd_sender_wifi_init(void);
void syncd_sender_wifi_deinit(void);

#endif
