#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int WIFI_Init(void);
int WIFI_SendHttpRequest(int event_code);

#ifdef __cplusplus
}
#endif

#endif
