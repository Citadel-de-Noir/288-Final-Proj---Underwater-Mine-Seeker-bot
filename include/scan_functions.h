#ifndef SCAN_FUNCTIONS_H_
#define SCAN_FUNCTIONS_H_

#include <stdint.h>

typedef struct {
    int right;
    int left;
} servo_cal_t;

int right_calibration_value;
int left_calibration_value;

void scan_init(void);
servo_cal_t scan_cal(void);

typedef struct {
    uint16_t angle_deg; // angle
    float distance_cm;  // ping dist
    uint16_t ir_raw;    // raw ir
    uint16_t ir_dist;   // ir dist converted into cm
} scan_data_t;

void scan_init(void);

float scan_get_distance_at(uint16_t angle_deg);

void scan_get_data(uint16_t angle_deg, scan_data_t *data);

#endif /* SCAN_FUNCTIONS_H_ */
