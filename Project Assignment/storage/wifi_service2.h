#ifndef WIFI_SERVICE2_H
#define WIFI_SERVICE2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int WIFI2_Init(void);
int WIFI2_SendTcp(const char *message);

#ifdef __cplusplus
}
#endif

#endif
