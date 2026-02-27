  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/

/* ---------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------- Includes ----------------------------------------------------- */

/*
 * This file contains the main loop for sensor reading and preprocessing, fall detection, alerts and escalation, and communications
*/

// Standard library headers
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <sys/stat.h>
#include <stdbool.h>

// Drivers and BSP headers for peripherals
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/Components/spirit1/SPIRIT1_Library/Inc/SPIRIT_Types.h"
#include "../../Drivers/BSP/Components/spirit1/SPIRIT1_Library/Inc/SPIRIT_PktBasic.h"
#include "../../Drivers/BSP/Components/spirit1/SPIRIT1_Library/Inc/SPIRIT_Irq.h"

// Private headers
// #include "wifi_service2.h"
#include "wifi.h"
#include "wifi_secrets.h"
#include "lcd.h"
#include "tones.h"
#include "fall_detection.h"

#ifdef DEBUG
#define FALL_DEBUG 1
#endif

#define hypotf(x,y) sqrtf((x)*(x) + (y)*(y))
#define sqr(x) ((x)*(x))

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void init(void);
void blink_LED2(int delay_ms);
static void UART1_Init(void);
static int WIFI_AppSendText(const char *text);
extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART
extern int mov_avg(int N, int* accel_buff); // asm implementation
int mov_avg_C(int N, int* accel_buff); // Reference C implementation
void SPI_WIFI_ISR(void);
void SystemClock_Config(void);


static void Buzzer_Init(void);
static void Buzzer_On(void);
static void Buzzer_Off(void);
static void update_alert_outputs(fall_event_t new_event); // Handles LED2 and Buzzer based on g_latched_event
static void telebot_task(uint32_t now, fall_event_t event); // Handles Wifi and telebot messages every new NEARFALL/REALFALL
static void oled_task(uint32_t now, fall_event_t event); // Handles OLED texts
static void button_task(uint32_t now); // Handles user button to clear REAL_FALL latch


UART_HandleTypeDef huart1;

static void fd_update_windows(float acc_mag, float gyro_mag);

static void fd_get_ranges(float *acc_range, float *gyro_range);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ISM43362_DRDY_EXTI1_Pin) {
		SPI_WIFI_ISR();
	}
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	blink_LED2(100);
  }
}

/* --------------------------- Globals and constants --------------------------- */
const int N=4; // size of buffer
const float G = 9.8f; // Gravitational acceleration constant

// MPU raw data (for potential debugging/extension)
float ax = 0.0f, ay = 0.0f, az = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;
float mx = 0.0f, my = 0.0f, mz = 0.0f;

// Circular buffer to store most recent 4 accel readings
int accel_buff_x[4]={0};
int accel_buff_y[4]={0};
int accel_buff_z[4]={0};
int i=0;												// Counter to keep track of how many readings have been taken

float roll_pitch_yaw[3] = {0.0f};

// Sliding windows for recent accel/gyro magnitudes (for MAX-MIN range)
// Shorter window so gyro range "memory" decays faster after a spike
#define FD_WINDOW_SIZE 10
static float acc_mag_window[FD_WINDOW_SIZE] = {0.0f};
static float gyro_mag_window[FD_WINDOW_SIZE] = {0.0f};
static int fd_win_idx = 0;
static int fd_win_filled = 0;

// Pressure sensor (barometer) for optional "going up" detection
static float last_pressure = 0.0f;
static int pressure_ready = 0;

// Latched alert state for non-blocking LED/buzzer behavior
static fall_event_t g_latched_event = FALL_EVENT_NONE;
static uint32_t g_latched_timestamp = 0;

// Current classified event and when it was set (shared across tasks)
static fall_event_t g_current_event = FALL_EVENT_NONE;
static uint32_t g_current_event_timestamp = 0U;

// Whether the fall-detection state machine is allowed to run.
// After a REAL_FALL is detected, we pause it until the user
// acknowledges via the button.
static uint8_t g_detector_enabled = 1U;

// Sensor sampling period (controls how often we read sensors and run fall detection)
#define SENSOR_SAMPLE_PERIOD_MS 20U
static uint32_t g_last_sample_tick = 0U;

const char* NEAR_FALL_STR = "Near fall detected!";
const char* REAL_FALL_STR = "Real fall detected!";

static uint8_t g_wifi_server_ip[4] = {0};
static uint8_t g_wifi_ready = 0;
static const uint8_t g_wifi_socket = 1;

/* ---------------------------------------------------------------------------------------------------------- */

/*
 * This is the state machine of the fall detection algorithm.
 * Init -> Filter sensor accelerometer readings -> Calculate ||AT||, ||GT|| and angle <-> Check ||AT||>t_m -> (delta_AT>t_at && delta_GT>t_ga) -> check angle>t_th -> classify as real fall else near fall 
 */
int main(void)
{
	//-------------------------------------- Initialise Device --------------------------------------//
	init(); // initialize peripherals and UART for transmission

	static int csv_header_printed = 0;

	while (1)
	{
		uint32_t now = HAL_GetTick();
		fall_event_t fall_event = g_current_event; // snapshot for this loop

		// -------------------------------------- SENSOR SAMPLING + FALL DETECTION (TIMED) -------------------------------------- //
		if ((now - g_last_sample_tick) >= SENSOR_SAMPLE_PERIOD_MS)
		{
			g_last_sample_tick = now;
			
		/* -------------------------------------- READ ACCELEROMETER VALUES AND PRE-PROCESS -------------------------------------- */
		int16_t accel_data_i16[3] = { 0 };						// Array to store the x, y and z readings of accelerometer
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);					// Function to read accelerometer values
		
		// Copy the values over to a circular style buffer
		accel_buff_x[i%4]=accel_data_i16[0]; 					// Acceleration along X-Axis
		accel_buff_y[i%4]=accel_data_i16[1]; 					// Acceleration along Y-Axis
		accel_buff_z[i%4]=accel_data_i16[2]; 					// Acceleration along Z-Axis
		// Preprocessing the filtered outputs using moving average filter.
		float accel_filt_asm[3]={0}; 							// final value of filtered acceleration values
		accel_filt_asm[0]= (float)mov_avg(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_asm[1]= (float)mov_avg(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_asm[2]= (float)mov_avg(N,accel_buff_z) * (9.8/1000.0f);

		// Calculate magnitude of filtered accelerometer vector (AT_t)
		float accel_magnitude_asm = sqrtf(sqr(accel_filt_asm[0]) + sqr(accel_filt_asm[1]) + sqr(accel_filt_asm[2]));

		/* -------------------------------------- READ GYROSCOPE VALUES AND PRE-PROCESS -------------------------------------- */
		float gyro_data[3]={0.0};
		BSP_GYRO_GetXYZ(gyro_data);								// Function to read gyroscope values
		
		// The output of gyro has been made to display in dps(degree per second)
		float gyro_velocity[3]={0.0};
		gyro_velocity[0]=(gyro_data[0]*9.8/(1000));
		gyro_velocity[1]=(gyro_data[1]*9.8/(1000));
		gyro_velocity[2]=(gyro_data[2]*9.8/(1000));
		
		// Calculate magnitude of gyro vector (GT_t)
		float gyro_magnitude_asm = sqrtf(sqr(gyro_velocity[0]) + sqr(gyro_velocity[1]) + sqr(gyro_velocity[2]));
		
		// Update sliding windows and derive recent instability ranges for accel and gyro
		fd_update_windows(accel_magnitude_asm, gyro_magnitude_asm);
		float accel_recent_range = 0.0f;
		float gyro_recent_range  = 0.0f;
		fd_get_ranges(&accel_recent_range, &gyro_recent_range);
		

		/* -------------------------------------- READ MAGNETOMETER VALUES AND PRE-PROCESS -------------------------------------- */
		int16_t mag_raw[3] = {0};
		BSP_MAGNETO_GetXYZ(mag_raw);              // BSP expects int16_t* buffer
		float mag_xyz[3] = {0.0f};
		mag_xyz[0] = (float)mag_raw[0];
		mag_xyz[1] = (float)mag_raw[1];
		mag_xyz[2] = (float)mag_raw[2];
		
		/* -------------------------------------- COMPUTE ROLL & PITCH FROM ACCEL -------------------------------------- */
		float roll_rad  = atan2f(accel_filt_asm[1], hypotf(accel_filt_asm[0], accel_filt_asm[2]));
		float pitch_rad = atan2f(-accel_filt_asm[0], hypotf(accel_filt_asm[1], accel_filt_asm[2]));
		roll_pitch_yaw[0] = roll_rad  * 180.0f / (float)M_PI;
		roll_pitch_yaw[1] = pitch_rad * 180.0f / (float)M_PI;
		
		/* -------------------------------------- COMPUTE YAW WITH COUPLING EFFECT MITIGATION -------------------------------------- */
		float original_magX = mag_xyz[0];
		float original_magY = mag_xyz[1];
		// float original_magZ = mag_xyz[2];
		
		// compensate for roll
		float polar_r_xz = sqrt(mag_xyz[0]*mag_xyz[0] + mag_xyz[2]*mag_xyz[2]);
		// float theta_xz_init = atan2(mag_xyz[0],mag_xyz[2]);
		// float polar_angle_after_roll = theta_xz_init - roll_rad;
		mag_xyz[0] = polar_r_xz * cos(roll_rad);
		mag_xyz[2] = polar_r_xz * sin(roll_rad);
		
		// compensate for pitch
		float polar_r_yz = sqrt(mag_xyz[1]*mag_xyz[1] + mag_xyz[2]*mag_xyz[2]);
		// float theta_yz_init = atan2(mag_xyz[1],mag_xyz[2]);
		// float polar_angle_after_pitch = theta_yz_init - pitch_rad;
		mag_xyz[1] = polar_r_yz * cos(pitch_rad);
		mag_xyz[2] = polar_r_yz * sin(pitch_rad);
		
		float deltaX = mag_xyz[0] - original_magX;
		float deltaY = mag_xyz[1] - original_magY;

		float yaw = atan2(mag_xyz[0] - deltaX, mag_xyz[1] - deltaY);
		roll_pitch_yaw[2] = yaw  * 180.0f/M_PI;
		
		/* -------------------------------------- BAROMETER (PRESSURE SENSOR) -------------------------------------- */
		float pressure_hpa = BSP_PSENSOR_ReadPressure();
		float dp_hpa = 0.0f;
		if (pressure_ready)
		{
			dp_hpa = pressure_hpa - last_pressure;
		}
		else
		{
			pressure_ready = 1;
		}
		last_pressure = pressure_hpa;

		/* -------------------------------------- FALL DETECTION -------------------------------------- */
		fall_event_t new_event = FALL_EVENT_NONE;
		if (i >= 3 && g_detector_enabled)
		{
			new_event = detect_fall(accel_magnitude_asm, gyro_magnitude_asm,
								   accel_recent_range, gyro_recent_range,
								   roll_pitch_yaw, dp_hpa);

			if (new_event != g_current_event)
			{
				g_current_event = new_event;
				g_current_event_timestamp = now;
				fall_event = new_event;

				// Once we see a REAL_FALL, pause the detector until
				// the user acknowledges via the button.
				if (new_event == FALL_EVENT_REAL_FALL)
				{
					g_detector_enabled = 0U;
				}
			}
			
			if (fall_event == FALL_EVENT_NEAR_FALL)
			{
				int wifi_status = WIFI_AppSendText(NEAR_FALL_STR);
				(void)wifi_status;
			}
			else if (fall_event == FALL_EVENT_REAL_FALL)
			{
				int wifi_status = WIFI_AppSendText(REAL_FALL_STR);
				(void)wifi_status;
			}
		}
		
		/* -------------------------------------- OLED / BUZZER  -------------------------------------- */
		update_alert_outputs(fall_event);
		
		/* -------------------------------------- CSV DATA LOGGING OVER UART -------------------------------------- */
		#ifdef FALL_DEBUG
		char buffer[200];
		if(fall_event == FALL_EVENT_NEAR_FALL) {
			sprintf(buffer, "Classification: NEAR-FALL\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			int wifi_status = WIFI_AppSendText(NEAR_FALL_STR);
			if (wifi_status != 0) {
				sprintf(buffer, "WIFI send failed: %d\r\n", wifi_status);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			}
		}
		else if(fall_event == FALL_EVENT_REAL_FALL) {
			sprintf(buffer, "Classification: REAL FALL\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			int wifi_status = WIFI_AppSendText(REAL_FALL_STR);
			if (wifi_status != 0) {
				sprintf(buffer, "WIFI send failed: %d\r\n", wifi_status);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			}
		}
		if (!csv_header_printed) {
			// Header row
			sprintf(buffer,
					"time_ms,acc,accR,gyro,gyroR,roll,pitch,yaw,pressure,dp_hpa,state,event\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			csv_header_printed = 1;
		}

		// Data row
		sprintf(buffer,
				"%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.3f,%d,%d\r\n",
				(unsigned long)HAL_GetTick(),
				accel_magnitude_asm,
				accel_recent_range,
				gyro_magnitude_asm,
				gyro_recent_range,
				roll_pitch_yaw[0],
				roll_pitch_yaw[1],
				roll_pitch_yaw[2],
				pressure_hpa,
				dp_hpa,
				get_fall_state(),
				(int)fall_event);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		#endif
		i++;
		}

		// -------------------------------------- NON-BLOCKING TASKS (LED/BUZZER, TELEBOT, OLED, BUTTON) -------------------------------------- //
		update_alert_outputs(fall_event);
		telebot_task(now, fall_event);
		oled_task(now, fall_event);
		button_task(now);
	}
}

static void init (void)
{
	HAL_Init();													// Reset all peripherals, initialize flash interface and systick
	SystemClock_Config();
	UART1_Init();												// Initialize UART1 for serial communication
	fall_detection_init();										// Initialize fall detection state machine
	lcd_start();
	lcd_draw_text(80, 120, "Device", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(20, 160, "Initialization...", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);

	// Peripheral initializations using BSP functions
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_PSENSOR_Init();
	BSP_GYRO_Init();
	Buzzer_Init();
	BSP_MAGNETO_Init();
	BSP_LED_Off(LED2);											// Set the initial LED state to off
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);					// Initialize user button
	
	// Initialize Wi-Fi module, connect to ESP32 
	char wifi_status_buf[100];
	
	int status_len_1 = sprintf(wifi_status_buf, "Device initialized\r\n");
	if (status_len_1 > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*)wifi_status_buf, (uint16_t)status_len_1, HAL_MAX_DELAY);
	}

	WIFI_Status_t wifi_status = WIFI_Init();

	wifi_status = WIFI_Connect(WIFI_SSID, WIFI_PASSWORD, WIFI_ECN_WPA2_PSK);
	lcd_clear(LCD_COLOR_WHITE);
	lcd_draw_text(65, 120, "Connecting", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(50, 160, "to Wi-Fi...", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	if (wifi_status != WIFI_STATUS_OK) {
		g_wifi_ready = 0;
		return;
	}
	unsigned int a = 0;
	unsigned int b = 0;
	unsigned int c = 0;
	unsigned int d = 0;
	char tail = '\0';
	if (sscanf(ESP32_PROXY_HOST, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) == 4) {
		g_wifi_server_ip[0] = (uint8_t)a;
		g_wifi_server_ip[1] = (uint8_t)b;
		g_wifi_server_ip[2] = (uint8_t)c;
		g_wifi_server_ip[3] = (uint8_t)d;
	}
	
	wifi_status = WIFI_OpenClientConnection(g_wifi_socket, WIFI_TCP_PROTOCOL, "conn", g_wifi_server_ip, ESP32_PROXY_PORT, 0);
	if (wifi_status != WIFI_STATUS_OK) {
		g_wifi_ready = 0;
		return;
	} else {
		g_wifi_ready = 1;
	}
	lcd_clear(LCD_COLOR_GREEN);
	lcd_draw_text(65, 130, "Wi-Fi", LCD_COLOR_BLACK, LCD_COLOR_GREEN, 3);
	lcd_draw_text(30, 170, "Connected!", LCD_COLOR_BLACK, LCD_COLOR_GREEN, 3);
	HAL_Delay(500);
	lcd_clear(LCD_COLOR_WHITE);
	lcd_draw_text(75, 20, "Fall", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);
	lcd_draw_text(15, 60, "Detection", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);
	lcd_draw_text(50, 100, "Device", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);
	
	lcd_draw_text(35, 150, "Accel", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(30, 175, "00.00", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(150, 150, "Gyro", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(135, 175, "0000.00", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(25, 215, "Roll", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(25, 240, "00.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(90, 215, "Pitch", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(95, 240, "00.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(175, 215, "Yaw", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(170, 240, "00.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);

	// TEMP, RMB TO DELETE
	HAL_Delay(4000);
	lcd_clear(LCD_COLOR_WHITE);
	lcd_draw_text(70, 20, "Haha", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);
	lcd_draw_text(80, 60, "You", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);
	lcd_draw_text(70, 100, "Fell!", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);

	lcd_draw_text(35, 150, "Accel", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(30, 175, "98.76", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(150, 150, "Gyro", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(135, 175, "1234.56", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(25, 215, "Roll", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(25, 240, "12.3", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(90, 215, "Pitch", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(95, 240, "45.6", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(175, 215, "Yaw", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(170, 240, "78.9", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);

	lcd_draw_text(50, 280, "Press button", LCD_COLOR_RED, LCD_COLOR_WHITE, 2);
	lcd_draw_text(65, 300, "to revive", LCD_COLOR_RED, LCD_COLOR_WHITE, 2);

}

static int WIFI_AppSendText(const char *text)
{
	if (text == NULL) {
		return -10;
	}

	uint8_t payload[128];
	int n = snprintf((char *)payload, sizeof(payload), "%s\n", text);
	if ((n <= 0) || (n >= (int)sizeof(payload))) {
		return -11;
	}

	uint16_t sent_len = 0;
	WIFI_Status_t status = WIFI_SendData(g_wifi_socket, payload, (uint16_t)n, &sent_len, 7000U);

	if ((status == WIFI_STATUS_OK) && (sent_len == (uint16_t)n)) {
		return 0;
	}

	(void)WIFI_CloseClientConnection(g_wifi_socket);
	status = WIFI_OpenClientConnection(g_wifi_socket, WIFI_TCP_PROTOCOL, "conn", g_wifi_server_ip, ESP32_PROXY_PORT, 0);
	if (status != WIFI_STATUS_OK) {
		g_wifi_ready = 0;
		return -12;
	}

	sent_len = 0;
	status = WIFI_SendData(g_wifi_socket, payload, (uint16_t)n, &sent_len, 7000U);

	if ((status == WIFI_STATUS_OK) && (sent_len == (uint16_t)n)) {
		return 0;
	}
	return -13;
}



static void Buzzer_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Use ARD_D5 as a generic buzzer output pin (active-high)
	GPIO_InitStruct.Pin = ARD_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ARD_D5_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_RESET);
}

static void Buzzer_On(void)
{
	HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_SET);
}

static void Buzzer_Off(void)
{
	HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_RESET);
}

static void update_alert_outputs(fall_event_t new_event)
{
	uint32_t now = HAL_GetTick();

	// Latch events to avoid flicker; REAL_FALL stays latched
	// until cleared by the user button, NEAR_FALL auto-clears.
	// const uint32_t REAL_FALL_LATCH_MS = 0U;   // unused now
	const uint32_t NEAR_FALL_LATCH_MS = 2000U;

	if (new_event == FALL_EVENT_REAL_FALL)
	{
		g_latched_event = FALL_EVENT_REAL_FALL;
		g_latched_timestamp = now;
	}
	else if (new_event == FALL_EVENT_NEAR_FALL && g_latched_event == FALL_EVENT_NONE)
	{
		g_latched_event = FALL_EVENT_NEAR_FALL;
		g_latched_timestamp = now;
	}

	// Auto-clear only NEAR_FALL latches after timeout; REAL_FALL
	// is cleared by button_task() when the user acknowledges.
	if (g_latched_event == FALL_EVENT_NEAR_FALL && (now - g_latched_timestamp) > NEAR_FALL_LATCH_MS)
	{
		g_latched_event = FALL_EVENT_NONE;
	}

	// Non-blocking patterns for LED and buzzer
	static uint32_t last_buzz_toggle = 0U;
	static uint8_t  buzz_on = 0U;
	static uint32_t last_led_toggle = 0U;
	static uint8_t  led_on = 0U;

	const uint32_t BUZZ_DELAY_MS     = 5000U; // buzzer starts 5s after REAL_FALL
	const uint32_t BUZZ_ON_MS        = 120U;  // buzzer on duration
	const uint32_t BUZZ_OFF_MS       = 180U;  // buzzer off duration
	const uint32_t LED_REAL_PERIODMS = 200U;  // fast blink for REAL_FALL
	const uint32_t LED_NEAR_PERIODMS = 600U;  // slower blink for NEAR_FALL

	switch (g_latched_event)
	{
	case FALL_EVENT_REAL_FALL:
		// LED: fast blink
		if ((now - last_led_toggle) >= LED_REAL_PERIODMS)
		{
			if (led_on) { BSP_LED_Off(LED2); led_on = 0U; }
			else       { BSP_LED_On(LED2);  led_on = 1U; }
			last_led_toggle = now;
		}

		// Buzzer: start beeping only after delay from event
		if ((now - g_latched_timestamp) < BUZZ_DELAY_MS)
		{
			Buzzer_Off();
			buzz_on = 0U;
			break;
		}
		if (!buzz_on)
		{
			if ((now - last_buzz_toggle) >= BUZZ_OFF_MS)
			{
				Buzzer_On();
				buzz_on = 1U;
				last_buzz_toggle = now;
			}
		}
		else
		{
			if ((now - last_buzz_toggle) >= BUZZ_ON_MS)
			{
				Buzzer_Off();
				buzz_on = 0U;
				last_buzz_toggle = now;
			}
		}
		break;

	case FALL_EVENT_NEAR_FALL:
		// NEAR FALL: slow LED blink, no buzzer
		Buzzer_Off();
		buzz_on = 0U;
		if ((now - last_led_toggle) >= LED_NEAR_PERIODMS)
		{
			if (led_on) { BSP_LED_Off(LED2); led_on = 0U; }
			else       { BSP_LED_On(LED2);  led_on = 1U; }
			last_led_toggle = now;
		}
		break;

	case FALL_EVENT_NONE:
	default:
		// NO FALL: everything off
		Buzzer_Off();
		buzz_on = 0U;
		BSP_LED_Off(LED2);
		led_on = 0U;
		break;
	}
}

// User button task: clears latched REAL_FALL on a *press*,
// not a long hold, but still debounces using the original
// "release to arm, then a few consecutive pressed samples"
// pattern. Non-blocking.
static void button_task(uint32_t now)
{
	(void)now; // not used, we debounce by counts instead of time
	static uint8_t  button_was_down = 0U;
	static uint8_t  button_reset_armed = 0U;
	static uint8_t  button_press_ticks = 0U;
	const uint8_t BUTTON_RESET_TICKS_REQUIRED = 2U; // small debounce, no long hold

	if (g_latched_event != FALL_EVENT_REAL_FALL)
	{
		button_was_down = 0U;
		button_reset_armed = 0U;
		button_press_ticks = 0U;
		return;
	}

	GPIO_PinState state = BSP_PB_GetState(BUTTON_USER);
	if (state == GPIO_PIN_SET)
	{
		if (!button_was_down)
		{
			button_was_down = 1U;
			button_press_ticks = 1U;
		}
		else if (button_reset_armed)
		{
			if (button_press_ticks < BUTTON_RESET_TICKS_REQUIRED)
			{
				button_press_ticks++;
			}
			if (button_press_ticks >= BUTTON_RESET_TICKS_REQUIRED)
			{
			// Clear latched REAL_FALL and re-arm detector
			g_latched_event = FALL_EVENT_NONE;
			g_current_event = FALL_EVENT_NONE;
			g_current_event_timestamp = now;
			fall_detection_init();
			g_detector_enabled = 1U;
			button_was_down = 0U;
			button_reset_armed = 0U;
			button_press_ticks = 0U;
			}
		}
	}
	else
	{
		button_was_down = 0U;
		button_press_ticks = 0U;
		// A clean release "arms" the reset; the next
		// short press that is stable for a couple of
		// samples will be treated as an
		// acknowledgement to clear REAL_FALL.
		button_reset_armed = 1U;
	}
}

// Telebot / Wi-Fi reporting task: send once per event change
static void telebot_task(uint32_t now, fall_event_t event)
{
	// (void)now; // reserved for future backoff logic
	// static fall_event_t last_sent_event = FALL_EVENT_NONE;

	// if (!g_wifi_ready)
	// {
	// 	return;
	// }
	// if (event == FALL_EVENT_NONE || event == last_sent_event)
	// {
	// 	return;
	// }

	// const char *msg = NULL;
	// if (event == FALL_EVENT_NEAR_FALL)
	// {
	// 	msg = NEAR_FALL_STR;
	// }
	// else if (event == FALL_EVENT_REAL_FALL)
	// {
	// 	msg = REAL_FALL_STR;
	// }
	// else
	// {
	// 	return;
	// }

	// (void)WIFI_AppSendText(msg);
	// last_sent_event = event;
}

// OLED update task: update display only when classification changes
static void oled_task(uint32_t now, fall_event_t event)
{
	// (void)now; // not currently used
	// static fall_event_t last_drawn_event = FALL_EVENT_NONE;

	// if (event == last_drawn_event)
	// {
	// 	return;
	// }
	// last_drawn_event = event;

	// switch (event)
	// {
	// case FALL_EVENT_REAL_FALL:
	// 	lcd_draw_text(40, 130, "REAL FALL", LCD_COLOR_RED, LCD_COLOR_WHITE, 3);
	// 	break;
	// case FALL_EVENT_NEAR_FALL:
	// 	lcd_draw_text(30, 130, "NEAR FALL", LCD_COLOR_ORANGE, LCD_COLOR_WHITE, 3);
	// 	break;
	// case FALL_EVENT_NONE:
	// default:
	// 	lcd_draw_text(40, 130, "STABLE   ", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 3);
	// 	break;
	// }
}



// Window used to determine instability based on recent accel/gyro magnitudes. Updated every iteration and used to derive recent range for fall detection. Uses a simple circular buffer approach.
static void fd_update_windows(float acc_mag, float gyro_mag)
{
	acc_mag_window[fd_win_idx]  = acc_mag;
	gyro_mag_window[fd_win_idx] = gyro_mag;
	fd_win_idx = (fd_win_idx + 1) % FD_WINDOW_SIZE;
	if (fd_win_idx == 0)
	{
		fd_win_filled = 1;
	}
}

static void fd_get_ranges(float *acc_range, float *gyro_range)
{
	int count = fd_win_filled ? FD_WINDOW_SIZE : fd_win_idx;
	if (count <= 0)
	{
		*acc_range = 0.0f;
		*gyro_range = 0.0f;
		return;
	}

	float acc_min = acc_mag_window[0];
	float acc_max = acc_mag_window[0];
	float gyro_min = gyro_mag_window[0];
	float gyro_max = gyro_mag_window[0];

	for (int k = 1; k < count; ++k)
	{
		if (acc_mag_window[k] < acc_min) acc_min = acc_mag_window[k];
		if (acc_mag_window[k] > acc_max) acc_max = acc_mag_window[k];
		if (gyro_mag_window[k] < gyro_min) gyro_min = gyro_mag_window[k];
		if (gyro_mag_window[k] > gyro_max) gyro_max = gyro_mag_window[k];
	}

	*acc_range = acc_max - acc_min;
	*gyro_range = gyro_max - gyro_min;
}

/**
 * @brief  C implementation of the moving average filter. This is a reference implementation and is not optimized for performance.
 * @param  N: Number of samples to average (window size)
 * @param  accel_buff: Pointer to the buffer containing the accelerometer readings
 * @return The average of the last N samples in the buffer
 */
int mov_avg_C(int N, int* accel_buff)
{ 	// The implementation below is inefficient and meant only for verifying your results.
	int result = 0;
	for(int i = 0; i < N; i ++)
	{
		result += accel_buff[i];
	}
	result = result / 4;
	return result;
}

/**
 * @brief  UART1 Initialization Function
 * Pin configuration for UART. BSP_COM_Init() can do this automatically.
 */
static void UART1_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while(1);
	}

}

void SystemClock_Config(void)
{
  /* oscillator and clocks configs */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();

  /* 80 Mhz from PLL with MSI 8Mhz as source clock */
  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;   /* 8 Mhz */
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * Helper function for LED blinking
 * @param delay_ms: Delay in milliseconds between toggles
 */
void blink_LED2(int delay_ms)
{
	BSP_LED_Toggle(LED2);
	HAL_Delay(delay_ms);
}


// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
