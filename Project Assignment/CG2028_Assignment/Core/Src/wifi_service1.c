#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "main.h"
#include "wifi_secrets.h"
#include "../../Drivers/BSP/Components/es_wifi/es_wifi.h"

extern UART_HandleTypeDef huart1;

#define WIFI1_IO_TIMEOUT_MS          4500U
#define WIFI1_HTTP_BUFFER_SIZE       480U
#define WIFI1_CONNECT_SETTLE_MS      300U
#define WIFI1_SOCKET_SETTLE_MS       500U
#define WIFI1_SEND_TIMEOUT_MS        7000U
#define WIFI1_RECV_TIMEOUT_MS        2500U
#define WIFI1_MAX_INIT_RETRY         3U
#define WIFI1_RETRY_DELAY_MS         120U
#define WIFI1_CONNECT_ATTEMPTS       3U
#define WIFI1_RESET_PULSE_MS         10U
#define WIFI1_RESET_SETTLE_MS        500U

#define WIFI1_ERR_BAD_ARG            -100
#define WIFI1_ERR_BUS_REGISTER       -110
#define WIFI1_ERR_HW_RESET           -120
#define WIFI1_ERR_INIT               -130
#define WIFI1_ERR_NOT_CONNECTED      -140
#define WIFI1_ERR_RESOLVE            -210
#define WIFI1_ERR_TCP_OPEN           -220
#define WIFI1_ERR_HTTP_BUILD         -230
#define WIFI1_ERR_HTTP_SEND          -240

typedef struct {
    int stage;
    int es_status;
} wifi1_error_info_t;

static SPI_HandleTypeDef hspi3;
static ES_WIFIObject_t es;
static uint8_t wifi1_ready = 0;
static uint8_t wifi1_connected = 0;
static wifi1_error_info_t wifi1_last_error = {0, 0};

static void WIFI1_SetError(int stage, int es_status)
{
    wifi1_last_error.stage = stage;
    wifi1_last_error.es_status = es_status;
}

void WIFI1_GetLastError(int *stage, int *es_status)
{
    if (stage != NULL) {
        *stage = wifi1_last_error.stage;
    }
    if (es_status != NULL) {
        *es_status = wifi1_last_error.es_status;
    }
}

int WIFI1_SendProxyRawText(const char *text_body);

static void WIFI1_Log(const char *fmt, ...)
{
    char line[220];
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (written <= 0) {
        return;
    }

    size_t line_len = strnlen(line, sizeof(line));
    if ((line_len + 2U) < sizeof(line)) {
        line[line_len] = '\r';
        line[line_len + 1U] = '\n';
        line[line_len + 2U] = '\0';
        line_len += 2U;
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)line_len, 200U);
}

static uint32_t WIFI1_ClampTimeout(uint32_t timeout)
{
    if ((timeout == 0U) || (timeout > WIFI1_IO_TIMEOUT_MS)) {
        return WIFI1_IO_TIMEOUT_MS;
    }
    return timeout;
}

static int16_t WIFI1_IO_Send(uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint32_t t = WIFI1_ClampTimeout(timeout);
    uint8_t odd_tail[2] = {'\n', '\n'};

    if ((data == NULL) || (len == 0U)) {
        return 0;
    }

    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) != GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) >= t) {
            return -1;
        }
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);
    for (volatile uint32_t i = 0; i < 200U; ++i) {
        __NOP();
    }

    if (len > 1U) {
        if (HAL_SPI_Transmit(&hspi3, data, len / 2U, t) != HAL_OK) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }
    }

    if ((len & 1U) != 0U) {
        odd_tail[0] = data[len - 1U];
        if (HAL_SPI_Transmit(&hspi3, odd_tail, 1, t) != HAL_OK) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    return (int16_t)len;
}

static int16_t WIFI1_IO_Receive(uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint32_t t = WIFI1_ClampTimeout(timeout);
    uint16_t total = 0;

    if (data == NULL) {
        return -1;
    }

    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - start) >= t) {
            return -1;
        }
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);
    for (volatile uint32_t i = 0; i < 200U; ++i) {
        __NOP();
    }

    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) == GPIO_PIN_SET) {
        if (HAL_SPI_Receive(&hspi3, &data[total], 1, t) != HAL_OK) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }
        total += 2;
        if ((len != 0U) && (total >= len)) {
            break;
        }
        if (total >= ES_WIFI_DATA_SIZE) {
            break;
        }
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    return (int16_t)total;
}

static void WIFI1_DelayUs(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us / 5U;
    while (cycles-- > 0U) {
        __NOP();
    }
}

static int8_t WIFI1_PerformHardwareReset(void)
{
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(WIFI1_RESET_PULSE_MS);
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(WIFI1_RESET_SETTLE_MS);
    return 0;
}

static int8_t WIFI1_ResetModulePrompt(void)
{
    uint8_t prompt[6] = {0};
    uint8_t count = 0;
    uint32_t tickstart = HAL_GetTick();

    HAL_GPIO_WritePin(ISM43362_WAKEUP_GPIO_Port, ISM43362_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

    if (WIFI1_PerformHardwareReset() != 0) {
        return -1;
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);
    WIFI1_DelayUs(15);

    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) == GPIO_PIN_SET) {
        if (count > 4U) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }

        if (HAL_SPI_Receive(&hspi3, &prompt[count], 1, WIFI1_IO_TIMEOUT_MS) != HAL_OK) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }

        count += 2;

        if ((HAL_GetTick() - tickstart) > WIFI1_IO_TIMEOUT_MS) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

    if ((prompt[0] != 0x15U) || (prompt[1] != 0x15U) ||
        (prompt[2] != '\r')  || (prompt[3] != '\n') ||
        (prompt[4] != '>')   || (prompt[5] != ' ')) {
        return -1;
    }

    return 0;
}

static int8_t WIFI1_IO_Init(uint16_t mode)
{
    if ((mode != ES_WIFI_INIT) && (mode != ES_WIFI_RESET)) {
        return 0;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = ISM43362_BOOT0_Pin | ISM43362_WAKEUP_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = ISM43362_RST_Pin | ISM43362_SPI3_CSN_Pin;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pin = ISM43362_DRDY_EXTI1_Pin;
    HAL_GPIO_Init(ISM43362_DRDY_EXTI1_GPIO_Port, &gpio);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_SPI3;
    gpio.Pin = INTERNAL_SPI3_SCK_Pin | INTERNAL_SPI3_MISO_Pin | INTERNAL_SPI3_MOSI_Pin;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = LED3_WIFI__LED4_BLE_Pin;
    HAL_GPIO_Init(LED3_WIFI__LED4_BLE_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_SET);

    if ((HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_RESET) || (hspi3.Instance != SPI3)) {
        hspi3.Instance = SPI3;
        hspi3.Init.Mode = SPI_MODE_MASTER;
        hspi3.Init.Direction = SPI_DIRECTION_2LINES;
        hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
        hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi3.Init.NSS = SPI_NSS_SOFT;
        hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hspi3.Init.CRCPolynomial = 7;
        hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
        hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

        if (HAL_SPI_Init(&hspi3) != HAL_OK) {
            WIFI1_SetError(WIFI1_ERR_HW_RESET, -1);
            return -1;
        }
    }

    HAL_GPIO_WritePin(ISM43362_BOOT0_GPIO_Port, ISM43362_BOOT0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_WAKEUP_GPIO_Port, ISM43362_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_Delay(30);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    return WIFI1_ResetModulePrompt();
}

static int8_t WIFI1_IO_DeInit(void)
{
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    return 0;
}

static void WIFI1_IO_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

int WIFI1_Init(void)
{
    int8_t status;
    static const ES_WIFI_SecurityType_t sec_try[] = {
        ES_WIFI_SEC_WPA2,
        ES_WIFI_SEC_WPA_WPA2,
        ES_WIFI_SEC_WPA
    };

    if (wifi1_connected && ES_WIFI_IsConnected(&es)) {
        return 0;
    }
    wifi1_connected = 0;

    if (!wifi1_ready) {
        status = ES_WIFI_RegisterBusIO(&es, WIFI1_IO_Init, WIFI1_IO_DeInit, WIFI1_IO_Delay, WIFI1_IO_Send, WIFI1_IO_Receive);
        if (status != ES_WIFI_STATUS_OK) {
            WIFI1_SetError(WIFI1_ERR_BUS_REGISTER, status);
            WIFI1_Log("WIFI1 ERR %d es=%d (RegisterBus)", WIFI1_ERR_BUS_REGISTER, (int)status);
            return WIFI1_ERR_BUS_REGISTER;
        }

        for (uint8_t attempt = 0; attempt < WIFI1_MAX_INIT_RETRY; ++attempt) {
            status = ES_WIFI_Init(&es);
            if (status == ES_WIFI_STATUS_OK) {
                wifi1_ready = 1;
                break;
            }

            (void)WIFI1_IO_Init(ES_WIFI_RESET);
            HAL_Delay(WIFI1_RETRY_DELAY_MS);
        }

        if (!wifi1_ready) {
            WIFI1_SetError(WIFI1_ERR_INIT, status);
            WIFI1_Log("WIFI1 ERR %d es=%d (Init)", WIFI1_ERR_INIT, (int)status);
            return WIFI1_ERR_INIT;
        }
    }

    if (!ES_WIFI_IsConnected(&es)) {
        for (uint8_t attempt = 0; attempt < WIFI1_CONNECT_ATTEMPTS; ++attempt) {
            (void)ES_WIFI_Disconnect(&es);
            HAL_Delay(80);

            for (uint8_t sec_idx = 0; sec_idx < (uint8_t)(sizeof(sec_try) / sizeof(sec_try[0])); ++sec_idx) {
                status = ES_WIFI_Connect(&es, WIFI_SECRET_SSID, WIFI_SECRET_PASSWORD, sec_try[sec_idx]);
                if (status == ES_WIFI_STATUS_OK) {
                    HAL_Delay(WIFI1_CONNECT_SETTLE_MS);
                    if (ES_WIFI_IsConnected(&es)) {
                        wifi1_connected = 1;
                        WIFI1_SetError(0, 0);
                        WIFI1_Log("WIFI1 OK connected");
                        return 0;
                    }
                }
            }

            HAL_Delay(WIFI1_RETRY_DELAY_MS);
        }

        WIFI1_SetError(WIFI1_ERR_NOT_CONNECTED, status);
        WIFI1_Log("WIFI1 ERR %d es=%d (Connect)", WIFI1_ERR_NOT_CONNECTED, (int)status);
        return WIFI1_ERR_NOT_CONNECTED;
    }
}

int WIFI1_SendHttpCustom(const char *host,
                         uint16_t port,
                         const char *method,
                         const char *path,
                         const char *content_type,
                         const char *body,
                         uint8_t *resp_buf,
                         uint16_t resp_buf_len,
                         uint16_t *resp_len)
{
    if ((host == NULL) || (method == NULL) || (path == NULL) || (content_type == NULL) || (body == NULL)) {
        WIFI1_SetError(WIFI1_ERR_BAD_ARG, -1);
        return WIFI1_ERR_BAD_ARG;
    }

    if (WIFI1_Init() != 0) {
        return WIFI1_ERR_NOT_CONNECTED;
    }

    uint8_t ip[4] = {0};
    unsigned int a = 0;
    unsigned int b = 0;
    unsigned int c = 0;
    unsigned int d = 0;
    char tail = '\0';

    if (sscanf(host, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) == 4) {
        if ((a > 255U) || (b > 255U) || (c > 255U) || (d > 255U)) {
            WIFI1_SetError(WIFI1_ERR_RESOLVE, -3);
            return WIFI1_ERR_RESOLVE;
        }
        ip[0] = (uint8_t)a;
        ip[1] = (uint8_t)b;
        ip[2] = (uint8_t)c;
        ip[3] = (uint8_t)d;
    } else {
        int8_t dns_status = ES_WIFI_DNS_LookUp(&es, host, ip);
        if (dns_status != ES_WIFI_STATUS_OK) {
            WIFI1_SetError(WIFI1_ERR_RESOLVE, dns_status);
            return WIFI1_ERR_RESOLVE;
        }
    }

    char request[WIFI1_HTTP_BUFFER_SIZE];
    uint16_t body_len = (uint16_t)strlen(body);
    int req_len = snprintf(request,
                           sizeof(request),
                           "%s %s HTTP/1.1\r\n"
                           "Host: %s\r\n"
                           "Content-Type: %s\r\n"
                           "Content-Length: %u\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           method,
                           path,
                           host,
                           content_type,
                           (unsigned int)body_len,
                           body);

    if ((req_len <= 0) || (req_len >= (int)sizeof(request)) || (req_len >= 65535)) {
        WIFI1_SetError(WIFI1_ERR_HTTP_BUILD, req_len);
        return WIFI1_ERR_HTTP_BUILD;
    }

    WIFI1_Log("WIFI1 HTTP REQ:\n%s", request);

    ES_WIFI_Conn_t conn;
    memset(&conn, 0, sizeof(conn));
    conn.Number = 1;
    conn.Type = ES_WIFI_TCP_CONNECTION;
    conn.LocalPort = 0;
    conn.RemotePort = port;
    conn.RemoteIP[0] = ip[0];
    conn.RemoteIP[1] = ip[1];
    conn.RemoteIP[2] = ip[2];
    conn.RemoteIP[3] = ip[3];

    (void)ES_WIFI_StopClientConnection(&es, &conn);
    int8_t open_status = ES_WIFI_StartClientConnection(&es, &conn);
    if (open_status != ES_WIFI_STATUS_OK) {
        WIFI1_SetError(WIFI1_ERR_TCP_OPEN, open_status);
        return WIFI1_ERR_TCP_OPEN;
    }

    HAL_Delay(WIFI1_SOCKET_SETTLE_MS);

    uint16_t sent_len = 0;
    int8_t send_status = ES_WIFI_SendData(&es,
                                          conn.Number,
                                          (uint8_t *)request,
                                          (uint16_t)req_len,
                                          &sent_len,
                                          WIFI1_SEND_TIMEOUT_MS);

    if ((send_status != ES_WIFI_STATUS_OK) && (sent_len < (uint16_t)req_len)) {
        uint16_t remaining = (uint16_t)req_len - sent_len;
        uint16_t sent_retry = 0;

        HAL_Delay(120);
        send_status = ES_WIFI_SendData(&es,
                                       conn.Number,
                                       (uint8_t *)&request[sent_len],
                                       remaining,
                                       &sent_retry,
                                       WIFI1_SEND_TIMEOUT_MS);
        sent_len = (uint16_t)(sent_len + sent_retry);

        if ((send_status != ES_WIFI_STATUS_OK) && (sent_len < (uint16_t)req_len)) {
            (void)ES_WIFI_StopClientConnection(&es, &conn);
            WIFI1_SetError(WIFI1_ERR_HTTP_SEND, send_status);
            WIFI1_Log("WIFI1 ERR %d es=%d (HTTP send, sent=%u/%u)",
                      WIFI1_ERR_HTTP_SEND,
                      (int)send_status,
                      (unsigned int)sent_len,
                      (unsigned int)req_len);
            return WIFI1_ERR_HTTP_SEND;
        }
    }

    if (send_status != ES_WIFI_STATUS_OK) {
        WIFI1_Log("WIFI1 WARN send status=%d but bytes sent=%u/%u",
                  (int)send_status,
                  (unsigned int)sent_len,
                  (unsigned int)req_len);
    }

    uint16_t got = 0;
    if ((resp_buf != NULL) && (resp_buf_len > 0U)) {
        (void)ES_WIFI_ReceiveData(&es,
                                  conn.Number,
                                  resp_buf,
                                  resp_buf_len,
                                  &got,
                                  WIFI1_RECV_TIMEOUT_MS);
    }

    (void)ES_WIFI_StopClientConnection(&es, &conn);

    if (resp_len != NULL) {
        *resp_len = got;
    }

    WIFI1_SetError(0, 0);
    WIFI1_Log("WIFI1 OK HTTP sent bytes=%u recv=%u", (unsigned int)sent_len, (unsigned int)got);
    return 0;
}

int WIFI1_SendProxyEvent(int event_code)
{
    char body[64];
    int body_len = snprintf(body, sizeof(body), "Fall detected, event code = %d", event_code);
    if ((body_len <= 0) || (body_len >= (int)sizeof(body))) {
        WIFI1_SetError(WIFI1_ERR_HTTP_BUILD, body_len);
        return WIFI1_ERR_HTTP_BUILD;
    }

    return WIFI1_SendProxyRawText(body);
}

int WIFI1_SendProxyRawText(const char *text_body)
{
    if (text_body == NULL) {
        WIFI1_SetError(WIFI1_ERR_BAD_ARG, -1);
        return WIFI1_ERR_BAD_ARG;
    }

    return WIFI1_SendHttpCustom(ESP32_PROXY_HOST,
                                ESP32_PROXY_PORT,
                                "POST",
                                ESP32_PROXY_PATH,
                                "text/plain",
                                text_body,
                                NULL,
                                0,
                                NULL);
}
