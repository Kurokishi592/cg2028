  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/

/* ---------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------- Includes ----------------------------------------------------- */
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

#include "stdio.h"
#include "string.h"
#include "math.h"
#include <sys/stat.h>

static void UART1_Init(void);

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART

extern int mov_avg(int N, int* accel_buff); // asm implementation

int mov_avg_C(int N, int* accel_buff); // Reference C implementation

typedef enum
{
	FALL_EVENT_NONE = 0,
	FALL_EVENT_NEAR_FALL,
	FALL_EVENT_REAL_FALL
} fall_event_t;

fall_event_t detect_fall_minimal(const float accel_filt_asm[3], const float gyro_velocity[3]);

UART_HandleTypeDef huart1;


int main(void)
{
	const int N=4;

	HAL_Init();													// Reset all peripherals, initialize flash interface and systick

	UART1_Init();												// Initialize UART1 for serial communication

	// Peripheral initializations using BSP functions
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();

	BSP_LED_Off(LED2);											// Set the initial LED state to off

	int accel_buff_x[4]={0};
	int accel_buff_y[4]={0};
	int accel_buff_z[4]={0};
	int i=0;													// Counter to keep track of how many readings have been taken
	int delay_ms=100; 											// Change delay time to suit your code

	while (1)
	{
		BSP_LED_Toggle(LED2);									// This function helps to toggle the current LED state

		int16_t accel_data_i16[3] = { 0 };						// Array to store the x, y and z readings of accelerometer
		
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);					// Function to read accelerometer values

		// Copy the values over to a circular style buffer
		accel_buff_x[i%4]=accel_data_i16[0]; 					// Acceleration along X-Axis
		accel_buff_y[i%4]=accel_data_i16[1]; 					// Acceleration along Y-Axis
		accel_buff_z[i%4]=accel_data_i16[2]; 					// Acceleration along Z-Axis


		float gyro_data[3]={0.0};
		float* ptr_gyro=gyro_data;
		BSP_GYRO_GetXYZ(ptr_gyro);								// Function to read gyroscope values

		// The output of gyro has been made to display in dps(degree per second)
		float gyro_velocity[3]={0.0};
		gyro_velocity[0]=(gyro_data[0]*9.8/(1000));
		gyro_velocity[1]=(gyro_data[1]*9.8/(1000));
		gyro_velocity[2]=(gyro_data[2]*9.8/(1000));


		// Preprocessing the filtered outputs  The same needs to be done for the output from the C program as well
		float accel_filt_asm[3]={0}; 							// final value of filtered acceleration values

		accel_filt_asm[0]= (float)mov_avg(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_asm[1]= (float)mov_avg(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_asm[2]= (float)mov_avg(N,accel_buff_z) * (9.8/1000.0f);


		// Preprocessing the filtered outputs  The same needs to be done for the output from the assembly program as well
		float accel_filt_c[3]={0};

		accel_filt_c[0]=(float)mov_avg_C(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_c[1]=(float)mov_avg_C(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_c[2]=(float)mov_avg_C(N,accel_buff_z) * (9.8/1000.0f);

		/* ---------------------------------------------------------------------------------------------------------- */
		// UART transmission of the results

		char buffer[150]; 									// Create a buffer large enough to hold the text

		// Transmitting results of C execution over UART
		if(i>=3)
		{
			// // 1. First printf() Equivalent
			// sprintf(buffer, "Results of C execution for filtered accelerometer readings:\r\n");
			// HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// // 2. Second printf() (with Floats) Equivalent
			// // Note: Requires -u _printf_float to be enabled in Linker settings
			// sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
			// 		accel_filt_c[0], accel_filt_c[1], accel_filt_c[2]);
			// HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// // Transmitting results of asm execution over UART

			// // 1. First printf() Equivalent
			// sprintf(buffer, "Results of assembly execution for filtered accelerometer readings:\r\n");
			// HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// // 2. Second printf() (with Floats) Equivalent
			// // Note: Requires -u _printf_float to be enabled in Linker settings
			// sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
			// 		accel_filt_asm[0], accel_filt_asm[1], accel_filt_asm[2]);
			// HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// // Transmitting Gyroscope readings over UART

			// // 1. First printf() Equivalent
			// sprintf(buffer, "Gyroscope sensor readings:\r\n");
			// HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// // 2. Second printf() (with Floats) Equivalent
			// // Note: Requires -u _printf_float to be enabled in Linker settings
			// sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n\n",
			// 		gyro_velocity[0], gyro_velocity[1], gyro_velocity[2]);
			// HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		}

		HAL_Delay(delay_ms);	// 1 second delay

		/* ---------------------------------------------------------------------------------------------------------- */
		// Fall detection
		if(i>=3) {
			// fall_event_t fall_event = detect_fall_minimal(accel_filt_asm, gyro_velocity);
			fall_event_t fall_event = detect_fall_minimal(accel_filt_c, gyro_velocity);
			if(fall_event == FALL_EVENT_NEAR_FALL)
			{
				sprintf(buffer, "Classification: NEAR-FALL\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			}
			else if(fall_event == FALL_EVENT_REAL_FALL)
			{
				sprintf(buffer, "Classification: REAL FALL\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			}
		}

		i++;
	}
}

/**
 * @brief  Minimal fall classifier using filtered acceleration and gyro vectors.
 *         Reads directly from accel_filt_asm and gyro_velocity.
 * @param  accel_filt_asm: Filtered acceleration vector in m/s^2
 * @param  gyro_velocity: Gyro vector (scaled output)
 * @return FALL_EVENT_NONE, FALL_EVENT_NEAR_FALL, or FALL_EVENT_REAL_FALL
 */
fall_event_t detect_fall_minimal(const float accel_filt_asm[3], const float gyro_velocity[3])
{
	const float G_NOMINAL = 9.8f;
	const float STILL_ACCEL_BAND = 1.2f;
	const float REST_ACCEL_BAND = 0.35f;
	const float REST_GYRO_TH = 2.2f;
	const float NEAR_GYRO_TH = 5.5f;
	const float REAL_GYRO_TH = 8.0f;
	const float IMPACT_ACCEL_TH = 14.0f;
	const float FREE_FALL_ACCEL_TH = 4.5f;
	const float JERK_IMPACT_TH = 5.0f;
	const float NEAR_JERK_TH = 0.8f;

	typedef enum
	{
		DETECT_IDLE = 0,
		DETECT_WAIT_IMPACT
	} detect_state_t;

	static detect_state_t state = DETECT_IDLE;
	static int state_ticks = 0;
	static int near_fall_score = 0;
	static int cooldown_ticks = 0;

	static float gyro_bias_x = 0.0f;
	static float gyro_bias_y = 0.0f;
	static float gyro_bias_z = 0.0f;
	static int bias_ready = 0;
	static int bias_samples = 0;

	static float prev_accel_mag = 9.8f;
	static float gyro_mag_lpf = 0.0f;
	static int moving_ticks = 0;

	float ax = accel_filt_asm[0];
	float ay = accel_filt_asm[1];
	float az = accel_filt_asm[2];

	float gx_raw = gyro_velocity[0];
	float gy_raw = gyro_velocity[1];
	float gz_raw = gyro_velocity[2];

	float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
	float accel_jerk = fabsf(accel_mag - prev_accel_mag);
	prev_accel_mag = accel_mag;

	if(!bias_ready)
	{
		gyro_bias_x = (gyro_bias_x * bias_samples + gx_raw) / (bias_samples + 1);
		gyro_bias_y = (gyro_bias_y * bias_samples + gy_raw) / (bias_samples + 1);
		gyro_bias_z = (gyro_bias_z * bias_samples + gz_raw) / (bias_samples + 1);
		bias_samples++;
		if(bias_samples >= 20)
		{
			bias_ready = 1;
		}
		return FALL_EVENT_NONE;
	}

	if(fabsf(accel_mag - G_NOMINAL) < STILL_ACCEL_BAND)
	{
		const float alpha = 0.01f;
		gyro_bias_x = (1.0f - alpha) * gyro_bias_x + alpha * gx_raw;
		gyro_bias_y = (1.0f - alpha) * gyro_bias_y + alpha * gy_raw;
		gyro_bias_z = (1.0f - alpha) * gyro_bias_z + alpha * gz_raw;
	}

	float gx = gx_raw - gyro_bias_x;
	float gy = gy_raw - gyro_bias_y;
	float gz = gz_raw - gyro_bias_z;
	float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
	gyro_mag_lpf = 0.8f * gyro_mag_lpf + 0.2f * gyro_mag;

	if(cooldown_ticks > 0)
	{
		cooldown_ticks--;
		return FALL_EVENT_NONE;
	}

	switch(state)
	{
		case DETECT_IDLE:
			state_ticks = 0;

			if((fabsf(accel_mag - G_NOMINAL) < REST_ACCEL_BAND) && (gyro_mag_lpf < REST_GYRO_TH))
			{
				near_fall_score = 0;
				moving_ticks = 0;
				return FALL_EVENT_NONE;
			}

			if(moving_ticks < 20)
			{
				moving_ticks++;
			}

			if(((accel_mag < FREE_FALL_ACCEL_TH) && (gyro_mag_lpf > NEAR_GYRO_TH)) ||
			   ((accel_mag > IMPACT_ACCEL_TH) && (gyro_mag_lpf > REAL_GYRO_TH)) ||
			   ((accel_jerk > JERK_IMPACT_TH) && (gyro_mag_lpf > REAL_GYRO_TH)))
			{
				state = DETECT_WAIT_IMPACT;
				state_ticks = 0;
				near_fall_score = 0;
				return FALL_EVENT_NONE;
			}

			if((moving_ticks >= 3) &&
			   (gyro_mag_lpf > NEAR_GYRO_TH) && (gyro_mag_lpf < REAL_GYRO_TH) &&
			   (accel_mag > (G_NOMINAL - 2.5f)) && (accel_mag < IMPACT_ACCEL_TH) &&
			   (accel_jerk > NEAR_JERK_TH))
			{
				if(near_fall_score < 8)
				{
					near_fall_score++;
				}
			}
			else if(near_fall_score > 1)
			{
				near_fall_score -= 2;
			}
			else
			{
				near_fall_score = 0;
			}

			if(near_fall_score >= 8)
			{
				near_fall_score = 0;
				cooldown_ticks = 20;
				return FALL_EVENT_NEAR_FALL;
			}

			return FALL_EVENT_NONE;

		case DETECT_WAIT_IMPACT:
			state_ticks++;

			if(((accel_mag > IMPACT_ACCEL_TH) && (gyro_mag_lpf > REAL_GYRO_TH)) ||
			   ((accel_jerk > JERK_IMPACT_TH) && (gyro_mag_lpf > REAL_GYRO_TH)))
			{
				state = DETECT_IDLE;
				state_ticks = 0;
				cooldown_ticks = 30;
				return FALL_EVENT_REAL_FALL;
			}

			if(state_ticks > 10)
			{
				state = DETECT_IDLE;
				cooldown_ticks = 20;
				return FALL_EVENT_NEAR_FALL;
			}

			return FALL_EVENT_NONE;

		default:
			state = DETECT_IDLE;
			state_ticks = 0;
			near_fall_score = 0;
			cooldown_ticks = 0;
			return FALL_EVENT_NONE;
	}
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
