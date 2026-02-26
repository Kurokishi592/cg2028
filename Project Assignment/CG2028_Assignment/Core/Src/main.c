  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/

/* ---------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------- Includes ----------------------------------------------------- */
#include "main.h"
#include "tones.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"


#include "stdio.h"
#include "string.h"
#include "math.h"
#include <sys/stat.h>
#include <stdbool.h>

#ifdef DEBUG
#define FALL_DEBUG 1
#endif

#define hypotf(x,y) sqrtf((x)*(x) + (y)*(y))
#define sqr(x) ((x)*(x))

typedef enum
{
	FALL_EVENT_NONE = 0,
	FALL_EVENT_NEAR_FALL = 1,
	FALL_EVENT_REAL_FALL = 2
} fall_event_t;

static void init(void);
static void UART1_Init(void);
static void Buzzer_Init(void);
static void Buzzer_On(void);
static void Buzzer_Off(void);
static void update_alert_outputs(fall_event_t new_event);
extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART
extern int mov_avg(int N, int* accel_buff); // asm implementation
int mov_avg_C(int N, int* accel_buff); // Reference C implementation
int fall_get_state(void); // for debugging, returns current phase of the fall detection state machine


// Internal fall-detection state machine phases (for debugging/inspection)
typedef enum { PHASE_UPRIGHT = 0, PHASE_FALLING, PHASE_POSTFALL } fall_phase_t;

static fall_phase_t g_fall_phase = PHASE_UPRIGHT;

fall_event_t detect_fall_minimal(const float accel_filt_asm[3], const float gyro_velocity[3]);
fall_event_t detect_fall(float acc_magnitude, float gyro_magnitude, float acc_range, float gyro_range, float* roll_pitch_yaw, float dp_hpa);

UART_HandleTypeDef huart1;

static void fd_update_windows(float acc_mag, float gyro_mag);

static void fd_get_ranges(float *acc_range, float *gyro_range);

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
		fall_event_t fall_event = FALL_EVENT_NONE;
		if (i >= 3)
		{
			fall_event = detect_fall(accel_magnitude_asm, gyro_magnitude_asm,
									accel_recent_range, gyro_recent_range, roll_pitch_yaw, dp_hpa);
		}

		/* -------------------------------------- OLED / BUZZER  -------------------------------------- */
		update_alert_outputs(fall_event);

		/* -------------------------------------- CSV DATA LOGGING OVER UART -------------------------------------- */
		#ifdef FALL_DEBUG
		char buffer[200];
		if (!csv_header_printed)
		{
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
				fall_get_state(),
				(int)fall_event);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		#endif

		i++;
	}

}

static void init (void)
{
	HAL_Init();													// Reset all peripherals, initialize flash interface and systick
	UART1_Init();												// Initialize UART1 for serial communication

	// Peripheral initializations using BSP functions
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_PSENSOR_Init();
	BSP_GYRO_Init();
	Buzzer_Init();
	BSP_MAGNETO_Init();
	BSP_LED_Off(LED2);											// Set the initial LED state to off
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);					// Initialize the user button
}

int fall_get_state(void)
{
	return (int)g_fall_phase;
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

	// Latch events for a short duration to avoid flicker
	const uint32_t REAL_FALL_LATCH_MS = 5000U;
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

	// Auto-clear latches after timeout
	if (g_latched_event == FALL_EVENT_REAL_FALL && (now - g_latched_timestamp) > REAL_FALL_LATCH_MS)
	{
		g_latched_event = FALL_EVENT_NONE;
	}
	else if (g_latched_event == FALL_EVENT_NEAR_FALL && (now - g_latched_timestamp) > NEAR_FALL_LATCH_MS)
	{
		g_latched_event = FALL_EVENT_NONE;
	}


	switch (g_latched_event)
	{
	case FALL_EVENT_REAL_FALL:
		// REAL FALL: buzzer alarm + intermittent red LED flash
		Buzzer_On();
		break;

	case FALL_EVENT_NEAR_FALL:
		// NEAR FALL: steady red LED only, no buzzer
		Buzzer_Off();
		break;

	case FALL_EVENT_NONE:
	default:
		// NO FALL: everything off
		Buzzer_Off();
		break;
	}
}

fall_event_t detect_fall(float acc_magnitude, float gyro_magnitude, float acc_range, float gyro_range, float* roll_pitch_yaw, float dp_hpa)
{
	/* Parameters to be tuned */
	// const float G_NOMINAL          = G;
	const float UPRIGHT_PITCH_MIN  = 60.0f;   // |pitch| above this -> upright-ish
	const float LYING_PITCH_MAX    = 45.0f;   // |pitch| below this -> lying-ish
	// const float MOVE_GYRO_TH       = 150.0f;  // basic "moving" threshold
	const float BIG_GYRO_TH        = 2000.0f; // strong rotation -> potential fall
	const float FREEFALL_LOW_TH    = 6.0f;    // accel magnitude much lower than 1g
	const float ACC_DELTA_FALL_TH  = 3.0f;    // big change between samples
	const int   FALLING_MAX_TICKS  = 5;       // max samples to stay in FALLING to avoid slow tilt as real fall
	const int   LYING_DOWN_TICKS_MIN = 4;     // required lying+still samples (shorter to catch real falls)
	const int   POSTFALL_MAX_TICKS = 10;      // max samples to stay in POSTFALL

	// These are to identify the orientation and movements of the board.
	float pitch_deg = roll_pitch_yaw[1];
	bool upright_now = (fabsf(pitch_deg) > UPRIGHT_PITCH_MIN);
	bool lying_now   = (fabsf(pitch_deg) < LYING_PITCH_MAX);
	// bool moving_now  = (gyro_magnitude > MOVE_GYRO_TH);
	bool big_gyro_now = (gyro_magnitude > BIG_GYRO_TH);
	bool going_up = (dp_hpa < -0.2f); // negative pressure delta suggests upward motion

	// To differentiate between normal activity, use strong change:
	static float prev_acc_mag = 9.8f;
	static float prev_pitch_deg = 0.0f;
	float acc_delta   = fabsf(acc_magnitude - prev_acc_mag);
	float pitch_delta = fabsf(pitch_deg - prev_pitch_deg);
	bool freefall_like = (acc_magnitude < FREEFALL_LOW_TH);
	bool strong_change = big_gyro_now || freefall_like || (acc_delta > ACC_DELTA_FALL_TH) || (pitch_delta > 15.0f);
	static bool trigger_by_gyro = 0;
	static bool trigger_by_accel = 0;

	// To keep track of how long we've been in each phase, use counters that reset on phase change
	static int phase_ticks = 0;
	static int lying_down_ticks = 0;

	fall_event_t result = FALL_EVENT_NONE;

	// Use state machine to differentiate between near fall and real fall
	switch (g_fall_phase)
	{
	case PHASE_UPRIGHT:
		// Normal upright/sitting: small motion allowed, no impact
		// Start falling phase only from upright posture
		if (upright_now)
		{
			phase_ticks++; // we are only interested in how long we are upright
			if (strong_change && phase_ticks > 2 && !going_up)
			{
				trigger_by_gyro = big_gyro_now;
				trigger_by_accel = freefall_like || (acc_delta > ACC_DELTA_FALL_TH);
				g_fall_phase = PHASE_FALLING;
				phase_ticks = 0;
			}

		}
		break;

	case PHASE_FALLING:
		phase_ticks++;
		// Capture accel and gyro activity during fall window, as both are conditions for real fall but occurs over an episode, not a single sample
		if(gyro_magnitude > BIG_GYRO_TH) {
			trigger_by_gyro = true;
		}
		if(freefall_like || (acc_delta > ACC_DELTA_FALL_TH)) {
			trigger_by_accel = true;
		}

		// Transition to post-fall when we reach lying orientation and it was due to a huge rotation
		if (lying_now && trigger_by_accel)
		{
			g_fall_phase = PHASE_POSTFALL;
			phase_ticks = 0;
			lying_down_ticks = 1;
			break;
		}
		// Recovered without lying -> near-fall
		// If falling phase lasts too long without lying, treat as none and reset
		if (phase_ticks >= FALLING_MAX_TICKS)
		{
			g_fall_phase = PHASE_UPRIGHT;
			result = FALL_EVENT_NEAR_FALL;
			phase_ticks = 0;
			trigger_by_gyro = false;
			trigger_by_accel = false;
		}
		break;

	case PHASE_POSTFALL:
		phase_ticks++;
		if (lying_now)
		{
			lying_down_ticks++;
			if (lying_down_ticks >= LYING_DOWN_TICKS_MIN)
			{
				// Sequence: upright -> falling -> lying still for a while => real fall
				// g_fall_phase = PHASE_UPRIGHT; // will not re-trigger until back upright
				if (trigger_by_accel && trigger_by_gyro) 
				{ 
					result = FALL_EVENT_REAL_FALL;
				}
				else
				{
					result = FALL_EVENT_NEAR_FALL;
				}
				
				phase_ticks = 0;
				trigger_by_gyro = false;
				trigger_by_accel = false;

				// Button press to manually reset state after a real fall
				bool reset = false;
				int button_press_ticks = 0, button_reset_ticks_required = 2, button_reset_armed = false;
				while (!reset) {
					if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_SET) {
						if(button_press_ticks < button_reset_ticks_required) {
							button_press_ticks++;
						}
						if(button_reset_armed && button_press_ticks >= button_reset_ticks_required) {
							g_fall_phase = PHASE_UPRIGHT;
							reset = true;
							button_reset_armed = false;
						}
					} else {
						button_press_ticks = 0;
						button_reset_armed = true;
					}
				}
				break;
			}
		}
		else
		{
			// If we move out of lying quickly (stand up or roll), classify as near-fall
			if (phase_ticks > 2 || upright_now)
			{
				g_fall_phase = PHASE_UPRIGHT;
				result = FALL_EVENT_NEAR_FALL;
				phase_ticks = 0;
				break;
			}
		}
		// If we stay too long in ambiguous post-fall without settling, reset
		if (phase_ticks >= POSTFALL_MAX_TICKS)
		{
			g_fall_phase = PHASE_UPRIGHT;
			result = FALL_EVENT_NEAR_FALL;
			phase_ticks = 0;
		}
		break;

	default:
		g_fall_phase = PHASE_UPRIGHT;
		break;
	}

	// Update previous-sample features
	prev_acc_mag = acc_magnitude;
	prev_pitch_deg = pitch_deg;

	return result;
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


// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
