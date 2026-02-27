#include "fall_detection.h"
#include <math.h>

// Internal fall-detection state machine phases
typedef enum { 
    PHASE_UPRIGHT = 0, 
    PHASE_FALLING = 1, 
    PHASE_POSTFALL = 2
} fall_phase_t;

// Initialise state to default
static fall_phase_t g_fall_phase = PHASE_UPRIGHT;

void fall_detection_init(void)
{
    g_fall_phase = PHASE_UPRIGHT;
}

int get_fall_state(void)
{
	return (int)g_fall_phase;
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

		// Transition to post-fall when we reach lying orientation and it was due to a huge rotation + a free fall
		if (lying_now && trigger_by_accel && trigger_by_gyro)
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
        // Sequence: upright -> falling -> lying still for a while => real fall
		if (lying_now)
		{
			lying_down_ticks++;
			if (lying_down_ticks >= LYING_DOWN_TICKS_MIN)
			{
				result = FALL_EVENT_REAL_FALL;

				// Reset internal episode state; latching and button handling
				// are managed in main.c so we return immediately here.
				g_fall_phase = PHASE_UPRIGHT;
				phase_ticks = 0;
				trigger_by_gyro = false;
				trigger_by_accel = false;
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