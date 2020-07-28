#ifndef _SYNCD_RECEIVER_CONN_
#define _SYNCD_RECEIVER_CONN_

#include <stdint.h>

#define TAG_WIFI "SYNCD_WIFI"
#define TAG_ESPNOW "SYNCD_ESPNOW"

#define ESPNOW_CHANNEL 1

typedef enum{
	NO_NEW_PACKET,
	NEW_PACKET
}syncd_receiver_receive_result_t;

typedef struct{
	uint8_t * buf;
	uint8_t len;
}syncd_packet_t;

syncd_packet_t syncd_receiver_receive(void);

void syncd_receiver_espnow_init(void);
void syncd_receiver_espnow_deinit(void);
void syncd_receiver_wifi_init(void);
void syncd_receiver_wifi_deinit(void);


#endif
