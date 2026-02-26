#include "wifi_service2.h"

#include <stdint.h>
#include <string.h>

#include "main.h"
#include "wifi_secrets.h"
#include "../../Drivers/BSP/Components/es_wifi/es_wifi.h"

#define WIFI2_IO_TIMEOUT_MS      4000U
#define WIFI2_SEND_TIMEOUT_MS    5000U
#define WIFI2_INIT_DELAY_MS      300U
#define WIFI2_CONNECT_DELAY_MS   300U
#define WIFI2_RESET_PULSE_MS     10U
#define WIFI2_RESET_SETTLE_MS    500U

#define WIFI2_SERVER_IP0         192U
#define WIFI2_SERVER_IP1         168U
#define WIFI2_SERVER_IP2         1U
#define WIFI2_SERVER_IP3         103U
#define WIFI2_SERVER_PORT        5000U

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

    if (!wifi2_connected) {
        status = ES_WIFI_Connect(&es, WIFI_SECRET_SSID, WIFI_SECRET_PASSWORD, ES_WIFI_SEC_WPA2);
        if (status != ES_WIFI_STATUS_OK) {
            return WIFI2_ERR_CONNECT_AP;
        }
        wifi2_connected = 1;
        HAL_Delay(WIFI2_CONNECT_DELAY_MS);
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

    uint16_t msg_len = (uint16_t)strlen(message);
    if (msg_len == 0U) {
        return 0;
    }

    ES_WIFI_Conn_t conn;
    memset(&conn, 0, sizeof(conn));
    conn.Number = 1;
    conn.Type = ES_WIFI_TCP_CONNECTION;
    conn.LocalPort = 0;
    conn.RemotePort = WIFI2_SERVER_PORT;
    conn.RemoteIP[0] = WIFI2_SERVER_IP0;
    conn.RemoteIP[1] = WIFI2_SERVER_IP1;
    conn.RemoteIP[2] = WIFI2_SERVER_IP2;
    conn.RemoteIP[3] = WIFI2_SERVER_IP3;

    (void)ES_WIFI_StopClientConnection(&es, &conn);
    if (ES_WIFI_StartClientConnection(&es, &conn) != ES_WIFI_STATUS_OK) {
        return WIFI2_ERR_TCP_OPEN;
    }

    uint16_t sent_len = 0;
    int8_t send_status = ES_WIFI_SendData(&es,
                                          conn.Number,
                                          (uint8_t *)message,
                                          msg_len,
                                          &sent_len,
                                          WIFI2_SEND_TIMEOUT_MS);

    (void)ES_WIFI_StopClientConnection(&es, &conn);

    if ((send_status != ES_WIFI_STATUS_OK) || (sent_len < msg_len)) {
        return WIFI2_ERR_TCP_SEND;
    }

    return 0;
}