#ifndef _SYNCD_CONNECTION_
#define _SYNCD_CONNECTION_

#include <stdint.h>

#define TAG_WIFI "SYNCD_WIFI"

#define ESPNOW_CHANNEL 1

void syncd_wifi_init();
void syncd_espnow_init();
void syncd_espnow_deinit();

#endif
