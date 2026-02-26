#ifndef WIFI_SERVICE1_H
#define WIFI_SERVICE1_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int WIFI1_Init(void);
int WIFI1_SendProxyEvent(int event_code);
int WIFI1_SendProxyRawText(const char *text_body);
void WIFI1_GetLastError(int *stage, int *es_status);

#ifdef __cplusplus
}
#endif

#endif
