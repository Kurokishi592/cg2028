#pragma once

#include <stdint.h>
#include <stdbool.h>

// public fall event classification struct
typedef enum
{
	FALL_EVENT_NONE = 0,
	FALL_EVENT_NEAR_FALL = 1,
	FALL_EVENT_REAL_FALL = 2
} fall_event_t;

/**
 * @brief Initialise fall-detection state. Called once at startup.
 */
void fall_detection_init(void);

/**
 * @brief Main fall detection algorithm using current features and return event.
 * @param acc_magnitude   Filtered accel magnitude (||a|| in m/s^2)
 * @param gyro_magnitude  Filtered gyro magnitude (||g|| in scaled units)
 * @param acc_range       Recent accel magnitude range (to measure instability)
 * @param gyro_range      Recent gyro magnitude range (to measure instability)
 * @param roll_pitch_yaw  Array of 3 floats: roll [0], pitch [1], yaw [2] in degrees
 * @param dp_hpa          Pressure delta (current - previous) in hPa
 * @return FALL_EVENT_NONE / FALL_EVENT_NEAR_FALL / FALL_EVENT_REAL_FALL
 */
fall_event_t detect_fall(float acc_magnitude, float gyro_magnitude, float acc_range, float gyro_range, float* roll_pitch_yaw, float dp_hpa);

/**
 * @brief Get internal state machine phase.
 * @return 0 = PHASE_UPRIGHT, 1 = PHASE_FALLING, 2 = PHASE_POSTFALL.
 */
int get_fall_state(void);