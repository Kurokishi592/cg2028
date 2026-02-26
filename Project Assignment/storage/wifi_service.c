#include "wifi_service.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "main.h"
#include "wifi_secrets.h"
#include "../../Drivers/BSP/Components/es_wifi/es_wifi.h"

#define WIFI_MAX_INIT_RETRY       2
#define WIFI_IO_TIMEOUT_MS        4500U
#define WIFI_RETRY_DELAY_MS       120U
#define WIFI_CONNECT_ATTEMPTS     2
#define WIFI_CONNECT_SETTLE_MS    220U
#define WIFI_HTTP_REQ_BUFFER_SIZE 320U
#define WIFI_SOCKET_SETTLE_MS     80U
#define WIFI_SEND_TIMEOUT_MS      5000U
#define WIFI_RECV_TIMEOUT_MS      2500U
#define WIFI_RESET_PULSE_MS       10U
#define WIFI_RESET_SETTLE_MS      500U
#define WIFI_DRDY_STARTUP_TO_MS   2500U
#define WIFI_TCP_OPEN_SOCKET_TRIES 2U
#define WIFI_TCP_OPEN_RETRIES     2U
#define WIFI_TCP_RETRY_DELAY_MS   80U
#define WIFI_DEBUG_UART           1U

#define WIFI_ERR_BUS_REGISTER     -11
#define WIFI_ERR_INIT_FAIL        -12
#define WIFI_ERR_CONNECT_FAIL     -22
#define WIFI_ERR_SEND_EMPTY       -41
#define WIFI_ERR_SEND_OPEN_FAIL   -42
#define WIFI_ERR_SEND_FAIL        -43

static uint32_t WIFI_ClampTimeout(uint32_t timeout)
{
    if ((timeout == 0U) || (timeout > WIFI_IO_TIMEOUT_MS)) {
        return WIFI_IO_TIMEOUT_MS;
    }
    return timeout;
}

static SPI_HandleTypeDef hspi3;

static ES_WIFIObject_t esWifiObj;

static uint8_t wifi_initialized = 0;
static uint8_t wifi_connected = 0;

static int8_t WIFI_EnsureInit(void);
static int8_t WIFI_EnsureConnected(void);

static int8_t WIFI_IO_Init(uint16_t mode);
static int8_t WIFI_IO_DeInit(void);
static void WIFI_IO_Delay(uint32_t ms);
static int16_t WIFI_IO_Send(uint8_t *data, uint16_t len, uint32_t timeout);
static int16_t WIFI_IO_Receive(uint8_t *data, uint16_t len, uint32_t timeout);

static int8_t WIFI_RegisterBus(void);
static int8_t WIFI_ModulePowerUp(void);
static void WIFI_ModuleGpioInit(void);
static void WIFI_IndicatorInit(void);
static void WIFI_IndicatorOn(void);
static int8_t WIFI_SPI3_Init(void);
static int8_t WIFI_PerformHardwareReset(void);
static int8_t WIFI_ResetModulePrompt(void);
static int8_t WIFI_WaitReadyState(GPIO_PinState expected, uint32_t timeout_ms);
static int8_t WIFI_WaitReadyRising(uint32_t timeout_ms);
static void WIFI_DelayUs(uint32_t us);
static int WIFI_FormatEsp32ProxyHttpRequest(uint8_t *request, size_t request_size, int event_code);
static int8_t WIFI_ParseIpv4Literal(const char *host, uint8_t out_ip[4]);
static int8_t WIFI_ResolveHostIpv4(const char *host, uint8_t out_ip[4]);
static int8_t WIFI_OpenTcpConnection(const uint8_t ip_addr[4], uint16_t remote_port, ES_WIFI_Conn_t *out_conn, uint8_t *out_socket);
static int8_t WIFI_TryConnectSecurity(ES_WIFI_SecurityType_t sec);
static void WIFI_Log(const char *fmt, ...);

#if WIFI_DEBUG_UART
extern UART_HandleTypeDef huart1;
#endif

static void WIFI_Log(const char *fmt, ...)
{
#if WIFI_DEBUG_UART
    char line[180];
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
#else
    (void)fmt;
#endif
}

static int8_t WIFI_RegisterBus(void)
{
    static uint8_t registered = 0;

    if (registered) {
        return 0;
    }

    if (ES_WIFI_RegisterBusIO(&esWifiObj,
                              WIFI_IO_Init,
                              WIFI_IO_DeInit,
                              WIFI_IO_Delay,
                              WIFI_IO_Send,
                              WIFI_IO_Receive) != ES_WIFI_STATUS_OK) {
        return -1;
    }

    registered = 1;
    return 0;
}

static int8_t WIFI_PerformHardwareReset(void)
{
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(WIFI_RESET_PULSE_MS);
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(WIFI_RESET_SETTLE_MS);
    return 0;
}

static int8_t WIFI_ModulePowerUp(void)
{
    WIFI_ModuleGpioInit();
    WIFI_IndicatorInit();
    WIFI_IndicatorOn();

    HAL_GPIO_WritePin(ISM43362_BOOT0_GPIO_Port, ISM43362_BOOT0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ISM43362_WAKEUP_GPIO_Port, ISM43362_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_Delay(30);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    return 0;
}

static void WIFI_ModuleGpioInit(void)
{
    static uint8_t initialized = 0;
    GPIO_InitTypeDef gpio = {0};

    if (initialized) {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

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

    HAL_GPIO_WritePin(ISM43362_BOOT0_GPIO_Port, ISM43362_BOOT0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_WAKEUP_GPIO_Port, ISM43362_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_SET);

    initialized = 1;
}

static void WIFI_IndicatorInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = LED3_WIFI__LED4_BLE_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED3_WIFI__LED4_BLE_GPIO_Port, &gpio);
}

static void WIFI_IndicatorOn(void)
{
    HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_SET);
}

static int8_t WIFI_SPI3_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = INTERNAL_SPI3_SCK_Pin | INTERNAL_SPI3_MISO_Pin | INTERNAL_SPI3_MOSI_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio);

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
        return -1;
    }

    return 0;
}

static int8_t WIFI_WaitReadyState(GPIO_PinState expected, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) != expected) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return -1;
        }
    }

    return 0;
}

static int8_t WIFI_WaitReadyRising(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return -1;
        }
    }

    return 0;
}

static void WIFI_DelayUs(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us / 5U;
    while (cycles-- > 0U) {
        __NOP();
    }
}

static int8_t WIFI_ResetModulePrompt(void)
{
    uint8_t prompt[6] = {0};
    uint8_t count = 0;
    uint32_t tickstart = HAL_GetTick();

    HAL_GPIO_WritePin(ISM43362_WAKEUP_GPIO_Port, ISM43362_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

    if (WIFI_PerformHardwareReset() != 0) {
        return -1;
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);
    WIFI_DelayUs(15);

    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) == GPIO_PIN_SET) {
        if (count > 4U) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }

        if (HAL_SPI_Receive(&hspi3, &prompt[count], 1, WIFI_IO_TIMEOUT_MS) != HAL_OK) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }

        count += 2;

        if ((HAL_GetTick() - tickstart) > WIFI_IO_TIMEOUT_MS) {
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

static int8_t WIFI_IO_Init(uint16_t mode)
{
    if ((mode == ES_WIFI_INIT) || (mode == ES_WIFI_RESET)) {
        if (WIFI_ModulePowerUp() != 0) {
            return -1;
        }

        if ((HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_RESET) ||
            (hspi3.Instance != SPI3)) {
            if (WIFI_SPI3_Init() != 0) {
                return -1;
            }
        }

        return WIFI_ResetModulePrompt();
    }

    return 0;
}

static int8_t WIFI_IO_DeInit(void)
{
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    return 0;
}

static void WIFI_IO_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

static int16_t WIFI_IO_Send(uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint32_t effective_timeout = WIFI_ClampTimeout(timeout);
    uint8_t odd_tail[2] = {'\n', '\n'};

    if ((data == NULL) || (len == 0U)) {
        return 0;
    }

    if (WIFI_WaitReadyState(GPIO_PIN_SET, effective_timeout) != 0) {
        return -1;
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);
    WIFI_DelayUs(15);

    if (len > 1U) {
        if (HAL_SPI_Transmit(&hspi3, data, len / 2U, effective_timeout) != HAL_OK) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }
    }

    if ((len & 1U) != 0U) {
        odd_tail[0] = data[len - 1U];
        if (HAL_SPI_Transmit(&hspi3, odd_tail, 1, effective_timeout) != HAL_OK) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    return (int16_t)len;
}

static int16_t WIFI_IO_Receive(uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint32_t effective_timeout = WIFI_ClampTimeout(timeout);
    uint16_t total = 0;

    if (data == NULL) {
        return -1;
    }

    if (WIFI_WaitReadyRising(effective_timeout) != 0) {
        return -1;
    }

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);
    WIFI_DelayUs(15);

    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) == GPIO_PIN_SET) {
        if (HAL_SPI_Receive(&hspi3, &data[total], 1, effective_timeout) != HAL_OK) {
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

static int8_t WIFI_EnsureInit(void)
{
    int8_t status = -1;

    if (wifi_initialized) {
        return 0;
    }

    if (WIFI_RegisterBus() != 0) {
        return WIFI_ERR_BUS_REGISTER;
    }

    for (uint8_t attempt = 0; attempt < WIFI_MAX_INIT_RETRY; ++attempt) {
        status = ES_WIFI_Init(&esWifiObj);
        if (status == ES_WIFI_STATUS_OK) {
            wifi_initialized = 1;
            return 0;
        }

        (void)WIFI_IO_Init(ES_WIFI_RESET);
        HAL_Delay(WIFI_RETRY_DELAY_MS);
    }

    return WIFI_ERR_INIT_FAIL;
}

static int8_t WIFI_EnsureConnected(void)
{
    static const ES_WIFI_SecurityType_t sec_fallback[] = {
        ES_WIFI_SEC_WPA_WPA2,
        ES_WIFI_SEC_WPA
    };

    if (wifi_connected) {
        return 0;
    }

    int8_t init_status = WIFI_EnsureInit();
    if (init_status != 0) {
        return init_status;
    }

    if (ES_WIFI_IsConnected(&esWifiObj)) {
        wifi_connected = 1;
        return 0;
    }

    for (uint8_t attempt = 0; attempt < WIFI_CONNECT_ATTEMPTS; ++attempt) {
        (void)ES_WIFI_Disconnect(&esWifiObj);
        HAL_Delay(80);

        if (WIFI_TryConnectSecurity(ES_WIFI_SEC_WPA2) == 0) {
            wifi_connected = 1;
            return 0;
        }

        for (uint8_t sec_idx = 0; sec_idx < (uint8_t)(sizeof(sec_fallback) / sizeof(sec_fallback[0])); ++sec_idx) {
            if (WIFI_TryConnectSecurity(sec_fallback[sec_idx]) == 0) {
                wifi_connected = 1;
                return 0;
            }
        }

        HAL_Delay(WIFI_RETRY_DELAY_MS);
    }

    return WIFI_ERR_CONNECT_FAIL;
}

static int8_t WIFI_TryConnectSecurity(ES_WIFI_SecurityType_t sec)
{
    int8_t connect_status = ES_WIFI_Connect(&esWifiObj,
                                            WIFI_SECRET_SSID,
                                            WIFI_SECRET_PASSWORD,
                                            sec);
    if (connect_status != ES_WIFI_STATUS_OK) {
        WIFI_Log("WIFI connect fail sec=%d status=%d", (int)sec, (int)connect_status);
        return -1;
    }

    HAL_Delay(WIFI_CONNECT_SETTLE_MS);
    if (!ES_WIFI_IsConnected(&esWifiObj)) {
        WIFI_Log("WIFI connect not associated sec=%d", (int)sec);
        return -1;
    }

    WIFI_Log("WIFI connected sec=%d", (int)sec);
    return 0;
}

int WIFI_Init(void)
{
    return WIFI_EnsureConnected();
}

static int WIFI_FormatEsp32ProxyHttpRequest(uint8_t *request, size_t request_size, int event_code)
{
    char body[128];
    const char *event_name = "none";

    if ((request == NULL) || (request_size == 0U)) {
        return -1;
    }

    if (event_code == 0) {
        event_name = "near_fall";
    } else if (event_code == 1) {
        event_name = "real_fall";
    } else if (event_code == 2) {
        event_name = "manual_alert";
    }

    int body_len = snprintf(body,
                            sizeof(body),
                            "{\"event_code\":%d,\"event\":\"%s\"}",
                            event_code,
                            event_name);
    if ((body_len <= 0) || (body_len >= (int)sizeof(body))) {
        return -1;
    }

    int req_len = snprintf((char *)request,
                           request_size,
                           "POST %s HTTP/1.1\r\n"
                           "Host: %s\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n\r\n"
                           "%s",
                           ESP32_PROXY_PATH,
                           ESP32_PROXY_HOST,
                           body_len,
                           body);

    if ((req_len <= 0) || (req_len >= (int)request_size)) {
        return -1;
    }

    return req_len;
}

static int8_t WIFI_ParseIpv4Literal(const char *host, uint8_t out_ip[4])
{
    unsigned int a = 0;
    unsigned int b = 0;
    unsigned int c = 0;
    unsigned int d = 0;
    char tail = '\0';

    if ((host == NULL) || (out_ip == NULL)) {
        return -1;
    }

    if (sscanf(host, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4) {
        return -1;
    }

    if ((a > 255U) || (b > 255U) || (c > 255U) || (d > 255U)) {
        return -1;
    }

    out_ip[0] = (uint8_t)a;
    out_ip[1] = (uint8_t)b;
    out_ip[2] = (uint8_t)c;
    out_ip[3] = (uint8_t)d;
    return 0;
}

static int8_t WIFI_ResolveHostIpv4(const char *host, uint8_t out_ip[4])
{
    if ((host == NULL) || (out_ip == NULL)) {
        return -1;
    }

    if (WIFI_ParseIpv4Literal(host, out_ip) == 0) {
        return 0;
    }

    if (ES_WIFI_DNS_LookUp(&esWifiObj, host, out_ip) != ES_WIFI_STATUS_OK) {
        return -1;
    }

    return 0;
}

static int8_t WIFI_OpenTcpConnection(const uint8_t ip_addr[4], uint16_t remote_port, ES_WIFI_Conn_t *out_conn, uint8_t *out_socket)
{
    ES_WIFI_Conn_t conn;

    if ((ip_addr == NULL) || (out_conn == NULL) || (out_socket == NULL)) {
        return -1;
    }

    memset(&conn, 0, sizeof(conn));
    conn.RemotePort = remote_port;
    conn.LocalPort = 0;
    conn.RemoteIP[0] = ip_addr[0];
    conn.RemoteIP[1] = ip_addr[1];
    conn.RemoteIP[2] = ip_addr[2];
    conn.RemoteIP[3] = ip_addr[3];
    conn.Type = ES_WIFI_TCP_CONNECTION;

    for (uint8_t attempt = 0; attempt < WIFI_TCP_OPEN_RETRIES; ++attempt) {
        for (uint8_t socket_try = 0; socket_try < WIFI_TCP_OPEN_SOCKET_TRIES; ++socket_try) {
            conn.Number = socket_try;
            (void)ES_WIFI_StopClientConnection(&esWifiObj, &conn);
            HAL_Delay(20);

            if (ES_WIFI_StartClientConnection(&esWifiObj, &conn) == ES_WIFI_STATUS_OK) {
                *out_conn = conn;
                *out_socket = socket_try;
                return 0;
            }
        }

        if ((attempt + 1U) < WIFI_TCP_OPEN_RETRIES) {
            HAL_Delay(WIFI_TCP_RETRY_DELAY_MS);
        }
    }

    return -1;
}

int WIFI_SendHttpRequest(int fall_count)
{
    uint8_t ip_addr[4] = {0};
    static uint8_t payload[WIFI_HTTP_REQ_BUFFER_SIZE];
    uint16_t payload_len = 0;
    uint16_t sent_len = 0;
    uint16_t recv_len = 0;
    uint8_t socket = 0;
    ES_WIFI_Conn_t conn;

    memset(payload, 0, sizeof(payload));
    memset(&conn, 0, sizeof(conn));

    if (WIFI_EnsureConnected() != 0) {
        WIFI_Log("HTTP abort: WiFi not connected");
        return -1;
    }

    if (WIFI_ResolveHostIpv4(ESP32_PROXY_HOST, ip_addr) != 0) {
        WIFI_Log("HTTP resolve fail host=%s", ESP32_PROXY_HOST);
        return -2;
    }

    WIFI_Log("HTTP target %u.%u.%u.%u:%u event=%d",
             ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3],
             (unsigned int)ESP32_PROXY_PORT,
             fall_count);

    if (WIFI_OpenTcpConnection(ip_addr, ESP32_PROXY_PORT, &conn, &socket) != 0) {
        WIFI_Log("HTTP open fail all sockets");
        return WIFI_ERR_SEND_OPEN_FAIL;
    }

    WIFI_Log("HTTP socket open ok socket=%u", (unsigned int)socket);

    HAL_Delay(WIFI_SOCKET_SETTLE_MS);

    int req_len = WIFI_FormatEsp32ProxyHttpRequest(payload, sizeof(payload), fall_count);
    if (req_len <= 0) {
        (void)ES_WIFI_StopClientConnection(&esWifiObj, &conn);
        WIFI_Log("HTTP request format fail");
        return WIFI_ERR_SEND_EMPTY;
    }

    payload_len = (uint16_t)req_len;

    if (ES_WIFI_SendData(&esWifiObj,
                         socket,
                         payload,
                         payload_len,
                         &sent_len,
                         WIFI_SEND_TIMEOUT_MS) != ES_WIFI_STATUS_OK) {
        HAL_Delay(120);
        WIFI_Log("HTTP send retry socket=%u", (unsigned int)socket);
        if (ES_WIFI_SendData(&esWifiObj,
                             socket,
                             payload,
                             payload_len,
                             &sent_len,
                             WIFI_SEND_TIMEOUT_MS) != ES_WIFI_STATUS_OK) {
            (void)ES_WIFI_StopClientConnection(&esWifiObj, &conn);
            WIFI_Log("HTTP send fail socket=%u", (unsigned int)socket);
            return WIFI_ERR_SEND_FAIL;
        }
    }

    WIFI_Log("HTTP send ok bytes=%u", (unsigned int)sent_len);

    (void)ES_WIFI_ReceiveData(&esWifiObj,
                              socket,
                              payload,
                              sizeof(payload) - 1,
                              &recv_len,
                              WIFI_RECV_TIMEOUT_MS);

    (void)ES_WIFI_StopClientConnection(&esWifiObj, &conn);
    return 0;
}
