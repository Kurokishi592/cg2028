#ifndef _SENSORS_H_
#define _SENSORS_H_
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef struct {
    float accel_magnitude_asm;
    float gyro_magnitude_asm;
    float accel_recent_range;
    float gyro_recent_range;
    float roll_pitch_yaw[3];
    float pressure_hpa;
    float dp_hpa;
} sensors_t;

#define hypotf(x,y) sqrtf((x)*(x) + (y)*(y))
#define sqr(x) ((x)*(x))
#define FD_WINDOW_SIZE 10

void sensors_init(void);
void get_IMU_reading(int i, sensors_t *sensors);
void get_baro_reading(sensors_t *sensors);

void fd_update_windows(float acc_mag, float gyro_mag);
void fd_get_ranges(float *acc_range, float *gyro_range);

extern int mov_avg(int N, int* accel_buff); // asm implementation
int mov_avg_C(int N, int* accel_buff); // Reference C implementation

#endif