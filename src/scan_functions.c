#include "./include/scan_functions.h"
#include "./include/servo.h"
#include "./include/adc.h"
#include "./include/ping.h"

void scan_init(void)
{
	servo_init();
    adc_init();
    ping_init();
}

/*
servo_cal_t scan_cal(void)
{
	servo_cal_t cal;

	if (!(g_scan_features & CYBOT_SCAN_FEATURE_SERVO))
	{
		cal.right = -1;
		cal.left = -1;
		return cal;
	}

	// Existing servo_calibrate() is interactive and blocking.
	// Return configured calibration globals here.
	cal.right = right_calibration_value;
	cal.left = left_calibration_value;
	return cal;
}
*/

void scan_get_data(uint16_t angle_deg, scan_data_t *data)
{
	if (data == 0)
	{
		return;
	}

	data->angle_deg = angle_deg;
	servo_move(angle_deg);
	data->ir_raw = adc_read();
	data->distance_cm = ping_getDistance();
	data->ir_dist = rawToDistance(adc_read());
}

float scan_get_distance_at(uint16_t angle_deg)
{
	servo_move(angle_deg);
	return ping_getDistance();
}

void scan_get_ir_data(uint16_t angle_deg, scan_data_t *data)
{
	if (data == 0)
	{
		return;
	}

	data->angle_deg = angle_deg;
	servo_move(angle_deg);
	data->ir_raw = adc_read();
	data->ir_dist = rawToDistance(data->ir_raw);
	data->distance_cm = 0;
}
