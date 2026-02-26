#include "wifi_service2.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "wifi_secrets.h"
#include "../../Drivers/BSP/Components/es_wifi/es_wifi.h"

#define WIFI2_IO_TIMEOUT_MS      4000U
#define WIFI2_SEND_TIMEOUT_MS    9000U
#define WIFI2_INIT_DELAY_MS      300U
#define WIFI2_CONNECT_DELAY_MS   300U
#define WIFI2_SOCKET_SETTLE_MS   1000U
#define WIFI2_POST_SEND_DELAY_MS 80U
#define WIFI2_RESET_PULSE_MS     10U
#define WIFI2_RESET_SETTLE_MS    500U

#define WIFI2_SERVER_HOST        ESP32_PROXY_HOST
#define WIFI2_SERVER_PORT        ESP32_PROXY_PORT

#define WIFI2_ERR_BAD_ARG        -1
#define WIFI2_ERR_BUS            -2
#define WIFI2_ERR_INIT           -3
#define WIFI2_ERR_CONNECT_AP     -4
#define WIFI2_ERR_TCP_OPEN       -5
#define WIFI2_ERR_TCP_SEND       -6

static SPI_HandleTypeDef hspi3;
static ES_WIFIObject_t es;
static uint8_t wifi2_bus_registered = 0;
static uint8_t wifi2_ready = 0;
static uint8_t wifi2_connected = 0;

extern UART_HandleTypeDef huart1;

static void WIFI2_Log(const char *fmt, ...)
{
    char line[160];
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

static uint32_t WIFI2_ClampTimeout(uint32_t timeout)
{
    if ((timeout == 0U) || (timeout > WIFI2_IO_TIMEOUT_MS)) {
        return WIFI2_IO_TIMEOUT_MS;
    }
    return timeout;
}

static int16_t WIFI2_IO_Send(uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint32_t t = WIFI2_ClampTimeout(timeout);
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

static int16_t WIFI2_IO_Receive(uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint32_t t = WIFI2_ClampTimeout(timeout);
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

static void WIFI2_DelayUs(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us / 5U;
    while (cycles-- > 0U) {
        __NOP();
    }
}

static int8_t WIFI2_ResetModulePrompt(void)
{
    uint8_t prompt[6] = {0};
    uint8_t count = 0;
    uint32_t tickstart = HAL_GetTick();

    HAL_GPIO_WritePin(ISM43362_WAKEUP_GPIO_Port, ISM43362_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(WIFI2_RESET_PULSE_MS);
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(WIFI2_RESET_SETTLE_MS);

    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);
    WIFI2_DelayUs(15);

    while (HAL_GPIO_ReadPin(ISM43362_DRDY_EXTI1_GPIO_Port, ISM43362_DRDY_EXTI1_Pin) == GPIO_PIN_SET) {
        if (count > 4U) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }

        if (HAL_SPI_Receive(&hspi3, &prompt[count], 1, WIFI2_IO_TIMEOUT_MS) != HAL_OK) {
            HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
            return -1;
        }

        count += 2;

        if ((HAL_GetTick() - tickstart) > WIFI2_IO_TIMEOUT_MS) {
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

static int8_t WIFI2_IO_Init(uint16_t mode)
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

    HAL_GPIO_WritePin(ISM43362_BOOT0_GPIO_Port, ISM43362_BOOT0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_WAKEUP_GPIO_Port, ISM43362_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ISM43362_RST_GPIO_Port, ISM43362_RST_Pin, GPIO_PIN_SET);

    return WIFI2_ResetModulePrompt();
}

static int8_t WIFI2_IO_DeInit(void)
{
    HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
    return 0;
}

static void WIFI2_IO_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

int WIFI2_Init(void)
{
    int8_t status;
    static const ES_WIFI_SecurityType_t sec_try[] = {
        ES_WIFI_SEC_WPA2,
        ES_WIFI_SEC_WPA_WPA2,
        ES_WIFI_SEC_WPA
    };

    if (!wifi2_bus_registered) {
        status = ES_WIFI_RegisterBusIO(&es,
                                       WIFI2_IO_Init,
                                       WIFI2_IO_DeInit,
                                       WIFI2_IO_Delay,
                                       WIFI2_IO_Send,
                                       WIFI2_IO_Receive);
        if (status != ES_WIFI_STATUS_OK) {
            return WIFI2_ERR_BUS;
        }
        wifi2_bus_registered = 1;
    }

    if (!wifi2_ready) {
        status = ES_WIFI_Init(&es);
        if (status != ES_WIFI_STATUS_OK) {
            return WIFI2_ERR_INIT;
        }
        wifi2_ready = 1;
        HAL_Delay(WIFI2_INIT_DELAY_MS);
    }

    wifi2_connected = ES_WIFI_IsConnected(&es) ? 1U : 0U;

    if (!wifi2_connected) {
        for (uint8_t sec_idx = 0; sec_idx < (uint8_t)(sizeof(sec_try) / sizeof(sec_try[0])); ++sec_idx) {
            status = ES_WIFI_Connect(&es, WIFI_SSID, WIFI_PASSWORD, sec_try[sec_idx]);
            if (status == ES_WIFI_STATUS_OK) {
                HAL_Delay(WIFI2_CONNECT_DELAY_MS);
                if (ES_WIFI_IsConnected(&es)) {
                    wifi2_connected = 1U;
                    break;
                }
            }
        }
        if (!wifi2_connected) {
            return WIFI2_ERR_CONNECT_AP;
        }
    }

    return 0;
}

int WIFI2_SendTcp(const char *message)
{
    if (message == NULL) {
        return WIFI2_ERR_BAD_ARG;
    }

    int init_status = WIFI2_Init();
    if (init_status != 0) {
        return init_status;
    }

    uint16_t message_len = (uint16_t)strlen(message);
    if (message_len == 0U) {
        return 0;
    }

    uint8_t payload[128];
    if (message_len >= (uint16_t)(sizeof(payload) - 1U)) {
        return WIFI2_ERR_BAD_ARG;
    }
    memcpy(payload, message, message_len);
    payload[message_len] = '\n';
    uint16_t msg_len = (uint16_t)(message_len + 1U);

    uint8_t remote_ip[4] = {0};
    unsigned int a = 0;
    unsigned int b = 0;
    unsigned int c = 0;
    unsigned int d = 0;
    char tail = '\0';
    if (sscanf(WIFI2_SERVER_HOST, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) == 4) {
        if ((a > 255U) || (b > 255U) || (c > 255U) || (d > 255U)) {
            return WIFI2_ERR_BAD_ARG;
        }
        remote_ip[0] = (uint8_t)a;
        remote_ip[1] = (uint8_t)b;
        remote_ip[2] = (uint8_t)c;
        remote_ip[3] = (uint8_t)d;
    } else {
        int8_t dns_status = ES_WIFI_DNS_LookUp(&es, WIFI2_SERVER_HOST, remote_ip);
        if (dns_status != ES_WIFI_STATUS_OK) {
            WIFI2_Log("WIFI2 DNS fail host=%s es=%d", WIFI2_SERVER_HOST, (int)dns_status);
            return WIFI2_ERR_TCP_OPEN;
        }
    }

    WIFI2_Log("WIFI2 TCP dst=%u.%u.%u.%u:%u",
              (unsigned int)remote_ip[0],
              (unsigned int)remote_ip[1],
              (unsigned int)remote_ip[2],
              (unsigned int)remote_ip[3],
              (unsigned int)WIFI2_SERVER_PORT);

    static const uint8_t socket_try[] = {0U, 1U};

    for (uint8_t k = 0; k < (uint8_t)(sizeof(socket_try) / sizeof(socket_try[0])); ++k) {
        ES_WIFI_Conn_t conn;
        memset(&conn, 0, sizeof(conn));
        conn.Number = socket_try[k];
        conn.Type = ES_WIFI_TCP_CONNECTION;
        conn.LocalPort = 0;
        conn.RemotePort = WIFI2_SERVER_PORT;
        conn.RemoteIP[0] = remote_ip[0];
        conn.RemoteIP[1] = remote_ip[1];
        conn.RemoteIP[2] = remote_ip[2];
        conn.RemoteIP[3] = remote_ip[3];

        (void)ES_WIFI_StopClientConnection(&es, &conn);
        int8_t open_status = ES_WIFI_StartClientConnection(&es, &conn);
        if (open_status != ES_WIFI_STATUS_OK) {
            WIFI2_Log("WIFI2 TCP open fail s=%u es=%d", (unsigned int)conn.Number, (int)open_status);
            if (k == 0U) {
                wifi2_connected = 0U;
                init_status = WIFI2_Init();
                if (init_status != 0) {
                    WIFI2_Log("WIFI2 TCP open fail (reinit=%d)", init_status);
                    return WIFI2_ERR_TCP_OPEN;
                }
            }
            continue;
        }

        WIFI2_Log("WIFI2 TCP open ok s=%u", (unsigned int)conn.Number);

        HAL_Delay(WIFI2_SOCKET_SETTLE_MS);

        uint16_t sent_len = 0;
        int8_t send_status = ES_WIFI_SendData(&es,
                                              conn.Number,
                                              payload,
                                              msg_len,
                                              &sent_len,
                                              WIFI2_SEND_TIMEOUT_MS);

        if ((send_status != ES_WIFI_STATUS_OK) && (sent_len < msg_len)) {
            uint16_t remaining = (uint16_t)(msg_len - sent_len);
            uint16_t sent_retry = 0;

            HAL_Delay(150);
            send_status = ES_WIFI_SendData(&es,
                                           conn.Number,
                                           &payload[sent_len],
                                           remaining,
                                           &sent_retry,
                                           WIFI2_SEND_TIMEOUT_MS);
            sent_len = (uint16_t)(sent_len + sent_retry);
        }

        if ((send_status != ES_WIFI_STATUS_OK) && (sent_len < msg_len)) {
            uint16_t remaining = (uint16_t)(msg_len - sent_len);
            uint16_t sent_retry2 = 0;

            HAL_Delay(200);
            send_status = ES_WIFI_SendData(&es,
                                           conn.Number,
                                           &payload[sent_len],
                                           remaining,
                                           &sent_retry2,
                                           WIFI2_SEND_TIMEOUT_MS);
            sent_len = (uint16_t)(sent_len + sent_retry2);
        }

        HAL_Delay(WIFI2_POST_SEND_DELAY_MS);
        (void)ES_WIFI_StopClientConnection(&es, &conn);

        if (sent_len >= msg_len) {
            WIFI2_Log("WIFI2 TCP ok s=%u es=%d sent=%u/%u",
                      (unsigned int)conn.Number,
                      (int)send_status,
                      (unsigned int)sent_len,
                      (unsigned int)msg_len);
            return 0;
        }

        WIFI2_Log("WIFI2 TCP fail s=%u es=%d sent=%u/%u",
                  (unsigned int)conn.Number,
                  (int)send_status,
                  (unsigned int)sent_len,
                  (unsigned int)msg_len);

        if ((send_status == ES_WIFI_STATUS_ERROR) && (sent_len == 0U)) {
            (void)ES_WIFI_Disconnect(&es);
            HAL_Delay(250);
            wifi2_connected = 0U;
            init_status = WIFI2_Init();
            if (init_status == 0) {
                open_status = ES_WIFI_StartClientConnection(&es, &conn);
                if (open_status == ES_WIFI_STATUS_OK) {
                    HAL_Delay(WIFI2_SOCKET_SETTLE_MS);
                    sent_len = 0;
                    send_status = ES_WIFI_SendData(&es,
                                                   conn.Number,
                                                   payload,
                                                   msg_len,
                                                   &sent_len,
                                                   WIFI2_SEND_TIMEOUT_MS);
                    HAL_Delay(WIFI2_POST_SEND_DELAY_MS);
                    (void)ES_WIFI_StopClientConnection(&es, &conn);
                    if (sent_len >= msg_len) {
                        WIFI2_Log("WIFI2 TCP recover ok s=%u es=%d sent=%u/%u",
                                  (unsigned int)conn.Number,
                                  (int)send_status,
                                  (unsigned int)sent_len,
                                  (unsigned int)msg_len);
                        return 0;
                    }
                    WIFI2_Log("WIFI2 TCP recover fail s=%u es=%d sent=%u/%u",
                              (unsigned int)conn.Number,
                              (int)send_status,
                              (unsigned int)sent_len,
                              (unsigned int)msg_len);
                }
            }
        }
    }

    uint16_t sent_to_len = 0;
    int8_t send_to_status = ES_WIFI_SendDataTo(&es,
                                               0,
                                               payload,
                                               msg_len,
                                               &sent_to_len,
                                               WIFI2_SEND_TIMEOUT_MS,
                                               remote_ip,
                                               WIFI2_SERVER_PORT);
    if (sent_to_len >= msg_len) {
        WIFI2_Log("WIFI2 TCP alt ok es=%d sent=%u/%u",
                  (int)send_to_status,
                  (unsigned int)sent_to_len,
                  (unsigned int)msg_len);
        return 0;
    }

    WIFI2_Log("WIFI2 TCP alt fail es=%d sent=%u/%u",
              (int)send_to_status,
              (unsigned int)sent_to_len,
              (unsigned int)msg_len);

    wifi2_connected = 0U;
    return WIFI2_ERR_TCP_SEND;
}