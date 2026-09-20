/*
 * @date 04/30/2026
 * Updated-4 for Naval Seeker Final Project
 * @author sayonsn
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "./include/Timer.h"
#include "./include/lcd.h"
#include "./include/ping.h"
#include "./include/servo.h"
#include "./include/open_interface.h"
#include "./include/scan_functions.h"
#include "./include/uart_interrupt.h"

// Approximation of pi used to convert servo/object angles from degrees to radians.
#define pi 3.1416

// Number of mines the autonomous routine is expected to find before ending the run.
#define MIN_MINES 6

// Approximate robot width in centimeters; used to decide if a gap is passable.
#define BOT_WIDTH_CM 18.0f

// Largest object width, in centimeters, that should still be treated as a mine.
#define MINE_WIDTH_CM 7.0f

// Smallest object width, in centimeters, that should still be treated as a mine.
#define MIN_MINE_WIDTH_CM 2.0f

// Assumed clear distance used to convert an empty angular span into a gap width.
#define CLEAR_DIST_CM 22.0f

// Left edge of the servo scan sweep, in degrees.
#define SCAN_START_ANGLE 0

// Right edge of the servo scan sweep, in degrees.
#define SCAN_END_ANGLE 180

// Degrees moved between each scan sample.
#define SCAN_STEP_DEG 1

// Angle margin ignored near each scan edge so cut-off objects are not trusted as mines.
#define EDGE_IGNORE_MARGIN_DEG 20

// Minimum width for a partial edge object to still count as an obstacle.
#define EDGE_OBJECT_MIN_WIDTH_CM 5.0f

// Maximum clear angular gap allowed inside one object before splitting it.
#define OBJECT_BRIDGE_GAP_DEG 4

// Maximum midpoint-angle difference allowed when matching mines across two scans.
#define MINE_CONFIRM_ANGLE_DEG 8

// Maximum distance difference allowed when matching mines across two scans.
#define MINE_CONFIRM_DIST_CM 5.0f

// Maximum angular gap between two narrow objects that might be parts of one wide object.
#define EDGE_MERGE_GAP_DEG 34

// Maximum distance difference between split segments of the same wide object.
#define EDGE_MERGE_DIST_CM 8.0f

// Maximum width of a segment that can be considered part of a split wide object.
#define EDGE_SEGMENT_MAX_WIDTH_CM 3.5f

// Maximum number of objects kept from a single scan.
#define MAX_SCAN_OBJECTS 16

// Forward and reverse wheel speed used for normal driving.
#define DRIVE_SPEED 150

// Wheel speed used when rotating in place.
#define TURN_SPEED 100

// Calibration offset applied to left turns to reduce overshoot/undershoot.
#define LEFT_TURN_OFFSET_DEG -10.0f

// Calibration offset applied to right turns to reduce overshoot/undershoot.
#define RIGHT_TURN_OFFSET_DEG -20.0f

// Distance driven after turning toward a safe gap.
#define STEP_FORWARD_CM 40.0f

// Maximum distance at which an IR reading counts as an object.
#define OBJECT_MAX_DIST_CM 50.0f

// Standard reverse distance used after bump or boundary detection.
#define BACKWARD_STEP_CM 10.0f

// Distance threshold for treating a mine-sized object as too close to trust.
#define CLOSE_OBJECT_DIST_CM 10.0f

// Small backup distance used before rescanning an uncertain close object.
#define CLOSE_OBJECT_BACKUP_CM 5.0f

// Minimum safe signal value for the left cliff sensor.
#define CL_L_LOW 300

// Maximum safe signal value for the left cliff sensor.
#define CL_L_HIGH 2500

// Minimum safe signal value for the front-left cliff sensor.
#define CL_FL_LOW 300

// Maximum safe signal value for the front-left cliff sensor.
#define CL_FL_HIGH 2500

// Minimum safe signal value for the front-right cliff sensor.
#define CL_FR_LOW 300

// Maximum safe signal value for the front-right cliff sensor.
#define CL_FR_HIGH 2500

// Minimum safe signal value for the right cliff sensor.
#define CL_R_LOW 300

// Maximum safe signal value for the right cliff sensor.
#define CL_R_HIGH 2500

// Maximum number of mine detections saved from one confirmed scan result.
#define MAX_MINES_PER_SCAN 8

// Time in milliseconds to wait while the demining action/song completes.
#define DEMINING_WAIT 5000

// Time in milliseconds for the servo to settle before midpoint ping measurement.
#define MIDPOINT_SERVO_SETTLE_MS 100

// Time in milliseconds to wait after midpoint ping before using the reading.
#define MIDPOINT_PING_WAIT_MS 100

// Minimum time between repeated manual safety warning messages.
#define MANUAL_SAFETY_REPORT_MS 1000

// structs
//------------------------------------------------------------------------------

// detected object struct
typedef struct
{
    bool found;

    int start_angle; // object begins
    int end_angle;   // object ends
    int mid_angle;   // center angle of object

    float distance_cm; // distance from robot
    float width_cm;    // estimated object width
    bool edge_partial; // true if object touches/extends beyond scan edge
} object_t;

// result from a scan sweeping front of robot
typedef struct
{
    int mine_count; // number of mines found in scan
    // bool mine_found;                     // true if mine detected in scan
    object_t mines[MAX_MINES_PER_SCAN]; // array of detected mines
    int object_count;                   // number of valid objects seen in scan
    bool uncertain_mine;                // true if a possible mine appeared in only one of two sweeps

    bool gap_found;      // true if gap detected in scan
    int gap_start_angle; // if gap found, angle where gap begins. if no gap found, value is invalid and should not be used
    int gap_end_angle;   // if gap found, angle where gap ends. if no gap found, value is invalid and should not be used
    int gap_mid_angle;   // if gap found, angle of center of gap. if no gap found, value is invalid and should not be used
    float gap_width_cm;  // if gap found, width of gap in cm. if no gap found, value is invalid and should not be used
} scan_result_t;

// helper functions

// Converts an angle from degrees to radians.
// This keeps the rest of the code working in servo-friendly degrees while still
// allowing math.h trig functions to be used for width and pose calculations.
static float deg_to_rad(float deg)
{
    return deg * pi / 180.0f;
}

// Checks all four cliff sensors against their calibrated safe ranges.
// A value below or above the expected range is treated as a boundary, hole, or
// unsafe floor reading that should stop normal forward movement.
static bool cliff_detected(oi_t *obj)
{
    return obj->cliffLeftSignal < CL_L_LOW || obj->cliffLeftSignal > CL_L_HIGH || obj->cliffFrontLeftSignal < CL_FL_LOW || obj->cliffFrontLeftSignal > CL_FL_HIGH || obj->cliffFrontRightSignal < CL_FR_LOW || obj->cliffFrontRightSignal > CL_FR_HIGH || obj->cliffRightSignal < CL_R_LOW || obj->cliffRightSignal > CL_R_HIGH;
}

// Checks the Create's left and right bump sensors.
// Used by both manual safety handling and autonomous obstacle avoidance.
static bool bumped(oi_t *obj)
{
    return obj->bumpLeft || obj->bumpRight;
}

// Decides whether an active scan should exit early.
// stop_flag always interrupts; manual_flag only interrupts when the caller wants
// manual mode to be able to take over during a scan.
static bool scan_interrupted(bool manual_interrupt)
{
    return stop_flag || (manual_interrupt && manual_flag);
}

// Robot pose estimate in centimeters with heading measured in degrees.
// Heading starts at 90 degrees so positive Y is treated as the initial forward direction.
static float pose_x_cm = 0.0f;
static float pose_y_cm = 0.0f;
static float pose_hdg_deg = 90.0f;

// Wraps the heading estimate into the range [0, 360).
// Keeping the heading normalized prevents long runs from accumulating angles
// outside the range expected by position reporting and trig calculations.
static void normalize_heading(void)
{
    while (pose_hdg_deg >= 360.0f)
    {
        pose_hdg_deg -= 360.0f;
    }
    while (pose_hdg_deg < 0.0f)
    {
        pose_hdg_deg += 360.0f;
    }
}

// Polls the Create sensors and updates the estimated robot position.
// The Create reports distance in millimeters and angle in degrees since the last
// update, so this function converts distance to centimeters and integrates it
// into the x/y pose estimate.
static void update_pose(oi_t *obj)
{
    float dist_cm;

    oi_update(obj);

    pose_hdg_deg += obj->angle;
    normalize_heading();

    dist_cm = obj->distance / 10.0f;
    pose_x_cm += dist_cm * cosf(deg_to_rad(pose_hdg_deg));
    pose_y_cm += dist_cm * sinf(deg_to_rad(pose_hdg_deg));
}

// Sends the current estimated position over UART.
// The GUI can use this periodic status line to track where the robot thinks it is.
static void report_position(void)
{
    char buffer[80];

    sprintf(buffer, "Pos: x %.2f cm, y %.2f cm, hdg %.2f deg\r\n", pose_x_cm,
            pose_y_cm, pose_hdg_deg);
    uart_sendStr(buffer);
}

// movement
//-----------------------------------------------------------------------------

// Stops both drive wheels immediately.
// This is called before mode changes, safety handling, scans, and shutdown.
static void stop_bot(void)
{
    oi_setWheels(0, 0);
}

// Moves the robot the requested distance in centimeters.
// Positive distances drive forward and negative distances drive backward. The
// caller chooses whether cliff and bump sensors should stop the movement, which
// lets controlled backup moves ignore sensors that may already be triggered.
static bool move_checked(oi_t *obj, float cm, bool check_cliff, bool check_bump)
{
    double traveled_mm = 0.0;
    double target_mm = cm * 10.0;

    update_pose(obj);

    while (fabs(traveled_mm) < fabs(target_mm))
    {
        update_pose(obj);
        if (stop_flag || manual_flag)
        {
            stop_bot();
            report_position();
            return false;
        }

        if ((check_cliff && cliff_detected(obj)) || (check_bump && bumped(obj)))
        {
            stop_bot();
            report_position();
            return false;
        }

        int speed = (target_mm > 0) ? DRIVE_SPEED : -DRIVE_SPEED;
        oi_setWheels(speed, speed);

        traveled_mm += obj->distance;
    }

    stop_bot();
    report_position();
    return true;
}

// Convenience wrapper for normal safe movement.
// It moves the requested distance while stopping if either a cliff/boundary or
// bumper hit is detected.
static bool move_safely(oi_t *obj, float cm)
{
    return move_checked(obj, cm, true, true);
}

// Backs the robot away by the standard reverse step.
// Sensor checks are disabled because this is usually called after the robot has
// already detected a boundary or obstacle and needs room to recover.
static bool back_away(oi_t *obj)
{
    return move_checked(obj, -BACKWARD_STEP_CM, false, false);
}

// Rotates the robot in place by the requested angle.
// Positive values turn left and negative values turn right. Calibration offsets
// are applied before turning to compensate for real-world turning error.
static bool turn_degrees(oi_t *obj, float degree)
{
    double turned = 0.0;
    float target_abs = fabsf(degree);

    if (degree > 0)
    {
        target_abs += LEFT_TURN_OFFSET_DEG;
    }
    else if (degree < 0)
    {
        target_abs += RIGHT_TURN_OFFSET_DEG;
    }

    if (target_abs < 0.0f)
    {
        target_abs = 0.0f;
    }

    update_pose(obj);

    while (fabs(turned) < target_abs)
    {
        update_pose(obj);

        if (stop_flag || manual_flag)
        {
            stop_bot();
            report_position();
            return false;
        }

        if (cliff_detected(obj) || bumped(obj))
        {
            stop_bot();
            report_position();
            return false;
        }

        int speed = (degree > 0) ? TURN_SPEED : -TURN_SPEED;
        oi_setWheels(speed, -speed); // positive degree = left turn

        turned += obj->angle;
    }
    stop_bot();
    report_position();
    return true;
}

// Handles a physical bumper collision.
// The robot backs up, turns away from the side that was hit, moves forward past
// the obstacle, then turns back toward its original route.
static void handle_bump(oi_t *obj)
{
    bool left = obj->bumpLeft;
    bool right = obj->bumpRight;

    stop_bot();

    if (!move_checked(obj, -BACKWARD_STEP_CM, true, false))
    {
        return;
    }

    if (left)
    {
        // Left bump means steer right around the obstacle.
        if (!turn_degrees(obj, -90))
        {
            return;
        }
        if (!move_safely(obj, STEP_FORWARD_CM))
        {
            return;
        }
        turn_degrees(obj, 90);
    }
    else if (right)
    {
        // Right bump means steer left around the obstacle.
        if (!turn_degrees(obj, 90))
        {
            return;
        }
        if (!move_safely(obj, STEP_FORWARD_CM))
        {
            return;
        }
        turn_degrees(obj, -90);
    }
}

// Handles cliff or boundary detection.
// The robot backs away first, then turns away from the side that detected the
// unsafe reading. If both sides are unsafe, it turns around more aggressively.
static void handle_boundary_hole(oi_t *obj)
{
    bool left_boundary = obj->cliffLeftSignal < CL_L_LOW || obj->cliffLeftSignal > CL_L_HIGH ||
                         obj->cliffFrontLeftSignal < CL_FL_LOW || obj->cliffFrontLeftSignal > CL_FL_HIGH;
    bool right_boundary = obj->cliffRightSignal < CL_R_LOW || obj->cliffRightSignal > CL_R_HIGH ||
                          obj->cliffFrontRightSignal < CL_FR_LOW || obj->cliffFrontRightSignal > CL_FR_HIGH;

    stop_bot();
    // flashDanger();
    if (!back_away(obj))
    {
        return;
    }

    if (right_boundary && !left_boundary)
    {
        turn_degrees(obj, 60);
    }
    else if (left_boundary && !right_boundary)
    {
        turn_degrees(obj, -60);
    }
    else
    {
        turn_degrees(obj, -90);
    }
}

// scan
//----------------------------------------------------------------------------------

// Estimates the physical width of an object in centimeters.
// The scan records an object's angular span, and this converts that span plus
// distance into a width using basic triangle geometry.
static float object_width_cm(int start_angle, int end_angle, float dist_cm)
{
    float theta = fabsf((float)(end_angle - start_angle));
    return 2.0f * dist_cm * tanf(deg_to_rad(theta / 2.0f));
}

// Validates an IR distance reading for object detection.
// Distances must be positive and close enough to be relevant to the robot's path.
static bool valid_obj_dist(float dist)
{
    return (dist > 0 && dist <= OBJECT_MAX_DIST_CM);
}

// Checks whether an object's estimated width falls inside the mine-size range.
// Width alone does not confirm a mine; later code also verifies repeatability
// across two scans and rejects likely split wide objects.
static bool is_mine(float width_cm)
{
    return (width_cm >= MIN_MINE_WIDTH_CM && width_cm <= MINE_WIDTH_CM);
}

// Checks whether a detected object is fully inside the trusted scan region.
// Objects near 0 or 180 degrees may be cut off by the scan edge, so they are
// treated as obstacles instead of reliable mine candidates.
static bool fully_inside_scan(object_t obj)
{
    return obj.start_angle >= (SCAN_START_ANGLE + EDGE_IGNORE_MARGIN_DEG) && obj.end_angle <= (SCAN_END_ANGLE - EDGE_IGNORE_MARGIN_DEG);
}

// Initializes a scan_result_t with safe default values.
// This prevents stale mine, gap, or uncertainty data from being reused when a
// scan exits early or finds nothing.
static void init_scan_result(scan_result_t *result)
{
    result->mine_count = 0;
    result->object_count = 0;
    result->uncertain_mine = false;
    result->gap_found = false;
    result->gap_start_angle = 90;
    result->gap_end_angle = 90;
    result->gap_mid_angle = 90;
    result->gap_width_cm = 0;
}

// Evaluates a clear angular section as a possible driving gap.
// The function converts the clear angle span into a physical width estimate and
// keeps only the widest gap found so far in the scan.
static void consider_gap(scan_result_t *result, int *best_gap_start,
                         int *best_gap_end, int clear_start, int clear_end)
{
    float gap_angle;
    float gap_width;

    if (clear_start < 0 || clear_end <= clear_start)
    {
        return;
    }

    gap_angle = fabsf((float)(clear_end - clear_start));
    gap_width = 2.0f * CLEAR_DIST_CM * tanf(deg_to_rad(gap_angle / 2.0f));

    if (gap_width > result->gap_width_cm)
    {
        result->gap_width_cm = gap_width;
        *best_gap_start = clear_start;
        *best_gap_end = clear_end;
    }
}

// Compares two distance readings using a centimeter tolerance.
// This is used when matching objects across scans or deciding if narrow segments
// are close enough to belong to the same larger object.
static bool similar_distance(float a, float b, float tolerance_cm)
{
    return fabsf(a - b) <= tolerance_cm;
}

// Detects when a mine-sized segment is probably part of a wider object.
// If two narrow objects are close in angle and distance, their combined width is
// checked so a large obstacle is not incorrectly counted as two mines.
static bool looks_like_split_wide_object(object_t *objects, int object_count,
                                         int index)
{
    int j;

    if (objects[index].width_cm > EDGE_SEGMENT_MAX_WIDTH_CM)
    {
        return false;
    }

    for (j = 0; j < object_count; j++)
    {
        if (j == index || objects[j].width_cm > EDGE_SEGMENT_MAX_WIDTH_CM)
        {
            continue;
        }

        int gap;
        int start_angle;
        int end_angle;
        float avg_dist;
        float merged_width;

        if (objects[j].start_angle > objects[index].end_angle)
        {
            gap = objects[j].start_angle - objects[index].end_angle;
            start_angle = objects[index].start_angle;
            end_angle = objects[j].end_angle;
        }
        else if (objects[index].start_angle > objects[j].end_angle)
        {
            gap = objects[index].start_angle - objects[j].end_angle;
            start_angle = objects[j].start_angle;
            end_angle = objects[index].end_angle;
        }
        else
        {
            continue;
        }

        avg_dist = (objects[index].distance_cm + objects[j].distance_cm) / 2.0f;
        merged_width = object_width_cm(start_angle, end_angle, avg_dist);

        if (gap <= EDGE_MERGE_GAP_DEG && similar_distance(objects[index].distance_cm, objects[j].distance_cm, EDGE_MERGE_DIST_CM) &&
            merged_width > MINE_WIDTH_CM)
        {
            return true;
        }
    }

    return false;
}

// Checks whether two mine candidates from separate scans describe the same mine.
// A match requires both midpoint angle and distance to be within confirmation
// tolerances so one noisy scan does not create a false detection.
static bool mine_matches(object_t a, object_t b)
{
    return (abs(a.mid_angle - b.mid_angle) <= MINE_CONFIRM_ANGLE_DEG && similar_distance(a.distance_cm, b.distance_cm,
                                                                                         MINE_CONFIRM_DIST_CM));
}

// Averages two matching mine detections into one confirmed mine result.
// Averaging smooths small scan-to-scan differences in angle, distance, and width.
static object_t average_mine(object_t a, object_t b)
{
    object_t mine = a;
    mine.start_angle = (a.start_angle + b.start_angle) / 2;
    mine.end_angle = (a.end_angle + b.end_angle) / 2;
    mine.mid_angle = (a.mid_angle + b.mid_angle) / 2;
    mine.distance_cm = (a.distance_cm + b.distance_cm) / 2.0f;
    mine.width_cm = (a.width_cm + b.width_cm) / 2.0f;
    return mine;
}

// Builds a confirmed mine list by comparing the first and second scan results.
// Only mine candidates that appear in both sweeps are copied into the confirmed
// result, which reduces false positives from one noisy reading.
static void add_confirmed_mines(scan_result_t *confirmed, scan_result_t first,
                                scan_result_t second)
{
    int i;
    int j;

    for (i = 0;
         i < first.mine_count && confirmed->mine_count < MAX_MINES_PER_SCAN;
         i++)
    {
        for (j = 0; j < second.mine_count; j++)
        {
            if (mine_matches(first.mines[i], second.mines[j]))
            {
                confirmed->mines[confirmed->mine_count++] = average_mine(
                    first.mines[i], second.mines[j]);
                break;
            }
        }
    }
}

// Completes one object after its scan edges have been found.
// The servo moves to the object's midpoint for a ping distance measurement, then
// the object width is calculated and the object is reported/saved if it is useful.
static void finish_object(object_t *objects, int *object_count,
                          object_t current, char *buffer, bool report)
{
    int angle_width;
    bool fully_visible;
    bool valid_width;
    bool valid_edge_object;

    current.mid_angle = (current.start_angle + current.end_angle) / 2;
    servo_move(current.mid_angle);
    timer_waitMillis(MIDPOINT_SERVO_SETTLE_MS);
    current.distance_cm = scan_get_distance_at(current.mid_angle);
    timer_waitMillis(MIDPOINT_PING_WAIT_MS);
    current.width_cm = object_width_cm(current.start_angle, current.end_angle,
                                       current.distance_cm);

    current.found = true;
    current.edge_partial = !fully_inside_scan(current);
    angle_width = current.end_angle - current.start_angle;
    fully_visible = !current.edge_partial;
    valid_width = angle_width >= 4 && current.width_cm >= MIN_MINE_WIDTH_CM;
    valid_edge_object = current.edge_partial && current.width_cm > EDGE_OBJECT_MIN_WIDTH_CM;

    if (report)
    {
        if (fully_visible && valid_width)
        {
            sprintf(buffer,
                    "Object angle %d-%d mid %d dist %.2f width %.2f\r\n",
                    current.start_angle, current.end_angle, current.mid_angle,
                    current.distance_cm, current.width_cm);
        }
        else if (valid_edge_object)
        {
            sprintf(buffer, "Edge object %d-%d mid %d dist %.2f width %.2f\r\n",
                    current.start_angle, current.end_angle, current.mid_angle,
                    current.distance_cm, current.width_cm);
        }
        else
        {
            sprintf(buffer,
                    "Ignored blip %d-%d mid %d dist %.2f width %.2f\r\n",
                    current.start_angle, current.end_angle, current.mid_angle,
                    current.distance_cm, current.width_cm);
        }
        uart_sendStr(buffer);
    }

    if (((fully_visible && valid_width) || valid_edge_object) && *object_count < MAX_SCAN_OBJECTS)
    {
        objects[*object_count] = current;
        (*object_count)++;
    }
}

// Performs one full servo sweep and extracts objects, mine candidates, and gaps.
// IR readings are used to find object edges; ping is only used at each object's
// midpoint for a better distance measurement before width is calculated.
static scan_result_t scan_data_once(bool MANUAL_INTERRUPT)
{
    scan_result_t result;
    char buffer[100];

    init_scan_result(&result);

    bool inObj = false;

    object_t current;
    current.found = false;
    object_t objects[MAX_SCAN_OBJECTS];
    int object_count = 0;

    int clear_start = -1;
    int best_gap_start = -1;
    int best_gap_end = -1;
    int object_gap_start = -1;

    int i;

    for (i = SCAN_START_ANGLE; i <= SCAN_END_ANGLE; i += SCAN_STEP_DEG)
    {

        scan_data_t data;

        scan_get_ir_data(i, &data);

        float ir_dist = data.ir_dist;
        float ping_dist = data.distance_cm;

        float dist = ir_dist;

        sprintf(buffer,
                "Angle: %d Deg\t IR Dist: %.2f cm\t Ping Dist: %.2f cm\t Used dist: %.2f cm\r\n",
                i, ir_dist, ping_dist, dist);
        uart_sendStr(buffer);

        // Use IR only for object edges. Ping is measured once at the midpoint
        // when the object is complete, then used for the width calculation.
        bool object_here = valid_obj_dist(dist);

        if (object_here)
        {
            /*
             sprintf(buffer, "Object detected at angle %d with distance %.2f cm\r\n",
             i, dist);
             uart_sendStr(buffer);
             */
            if (!inObj)
            {
                consider_gap(&result, &best_gap_start, &best_gap_end,
                             clear_start, i - SCAN_STEP_DEG);
                clear_start = -1;
                inObj = true;
                current.start_angle = i;
            }

            object_gap_start = -1;
            current.end_angle = i;
            current.mid_angle = (current.start_angle + current.end_angle) / 2;
        }

        else
        {
            if (inObj)
            {
                if (object_gap_start < 0)
                {
                    object_gap_start = i;
                }

                if ((i - object_gap_start) > OBJECT_BRIDGE_GAP_DEG)
                {
                    inObj = false;
                    finish_object(objects, &object_count, current, buffer,
                                  true);
                    clear_start = object_gap_start;
                    object_gap_start = -1;
                }
            }

            if (!inObj && clear_start < 0)
            {
                clear_start = i;
            }
        }

        if (scan_interrupted(MANUAL_INTERRUPT))
        {
            return result;
        }
    }

    if (inObj)
    {
        finish_object(objects, &object_count, current, buffer, true);
    }
    else
    {
        consider_gap(&result, &best_gap_start, &best_gap_end, clear_start,
                     SCAN_END_ANGLE);
    }

    result.object_count = object_count;

    for (i = 0; i < object_count; i++)
    {
        bool mine_width = is_mine(objects[i].width_cm);
        bool inside_scan = fully_inside_scan(objects[i]);
        bool split_wide_object = looks_like_split_wide_object(objects,
                                                              object_count, i);

        if (!inside_scan)
        {
            sprintf(buffer,
                    "Using edge object %d-%d mid %d dist %.2f width %.2f as obstacle\r\n",
                    objects[i].start_angle, objects[i].end_angle,
                    objects[i].mid_angle, objects[i].distance_cm,
                    objects[i].width_cm);
            uart_sendStr(buffer);
        }
        else if (mine_width && objects[i].distance_cm < CLOSE_OBJECT_DIST_CM)
        {
            result.uncertain_mine = true;
        }
        else if (mine_width && !split_wide_object && result.mine_count < MAX_MINES_PER_SCAN)
        {
            result.mines[result.mine_count] = objects[i];
            result.mine_count++;
        }
    }

    if (result.gap_width_cm >= BOT_WIDTH_CM && best_gap_start >= 0)
    {
        result.gap_found = true;
        result.gap_start_angle = best_gap_start;
        result.gap_end_angle = best_gap_end;
        result.gap_mid_angle = (best_gap_start + best_gap_end) / 2;
    }

    return result;
}

// Performs two scan sweeps and combines them into a more reliable result.
// Mines must appear in both sweeps to be confirmed. If no objects are seen, the
// scan reports a full front gap so autonomous mode can drive forward.
static scan_result_t scan_data(bool MANUAL_INTERRUPT)
{
    scan_result_t first;
    scan_result_t second;
    scan_result_t confirmed;

    first = scan_data_once(MANUAL_INTERRUPT);
    if (scan_interrupted(MANUAL_INTERRUPT))
    {
        return first;
    }

    second = scan_data_once(MANUAL_INTERRUPT);
    if (scan_interrupted(MANUAL_INTERRUPT))
    {
        return second;
    }

    init_scan_result(&confirmed);
    add_confirmed_mines(&confirmed, first, second);
    confirmed.object_count =
        (first.object_count > second.object_count) ? first.object_count : second.object_count;
    confirmed.uncertain_mine = (confirmed.mine_count == 0 && (first.mine_count > 0 || second.mine_count > 0));

    if (confirmed.mine_count == 0 && !confirmed.uncertain_mine && first.object_count == 0 && second.object_count == 0)
    {
        confirmed.gap_found = true;
        confirmed.gap_start_angle = SCAN_START_ANGLE;
        confirmed.gap_end_angle = SCAN_END_ANGLE;
        confirmed.gap_mid_angle = 90;
        confirmed.gap_width_cm = 999.0f;
    }
    else if (confirmed.mine_count == 0 && !confirmed.uncertain_mine && first.gap_found && second.gap_found)
    {
        confirmed.gap_found = true;
        confirmed.gap_start_angle = second.gap_start_angle;
        confirmed.gap_end_angle = second.gap_end_angle;
        confirmed.gap_mid_angle = second.gap_mid_angle;
        confirmed.gap_width_cm = second.gap_width_cm;
    }

    return confirmed;
}

// Displays the idle mode-selection prompt.
// This is shown when the robot is stopped and waiting for manual or auto mode.
static void show_idle_prompt(void)
{
    lcd_clear();
    lcd_printf("Press M for manual\nN for auto");
}

// Resets mode, movement, and scan flags after a stop command.
// It also clears the LCD, stops the wheels, and releases stop_flag so the main
// loop can return to the idle prompt cleanly.
static void reset_after_stop(void)
{
    stop_bot();
    manual_flag = 0;
    auto_flag = 0;
    start_flag = 0;
    initiate_scan = 0;
    scan_active = 0;
    move_byte = '\0';
    lcd_clear();
    lcd_printf("STOPPED");
    stop_flag = 0;
}

// Applies the latest manual drive command from UART.
// Commands time out after a short delay so the robot stops if the GUI stops
// sending movement updates.
static void drive_manual_command(void)
{
    if ((timer_getMillis() - move_timer) > 250)
    {
        move_byte = '\0';
        stop_bot();
        return;
    }

    switch (move_byte)
    {
    case 'w':
        oi_setWheels(DRIVE_SPEED, DRIVE_SPEED);
        break;
    case 'a':
        oi_setWheels(TURN_SPEED, -TURN_SPEED);
        break;
    case 's':
        oi_setWheels(-DRIVE_SPEED, -DRIVE_SPEED);
        break;
    case 'd':
        oi_setWheels(-TURN_SPEED, TURN_SPEED);
        break;
    default:
        stop_bot();
        break;
    }
}

// Decides if a manual safety event should stop the robot immediately.
// Forward motion and missing commands are stopped on cliff/bump events, while
// other commands can still let the driver steer away from the problem.
static bool manual_safety_stop_needed(void)
{
    return move_byte == 'w' || move_byte == '\0';
}

// Sends a human-readable scan summary for manual mode.
// It reports each confirmed mine, an uncertain mine warning, the best gap, or a
// no-result message, then sends SCAN END so the GUI knows the scan is finished.
static void send_manual_scan_result(scan_result_t result, char *buffer)
{
    int k;

    if (result.mine_count > 0)
    {
        for (k = 0; k < result.mine_count; k++)
        {
            sprintf(buffer, "Mine %d angle: %d dist: %.2f width: %.2f \r\n",
                    k + 1, result.mines[k].mid_angle,
                    result.mines[k].distance_cm, result.mines[k].width_cm);
            uart_sendStr(buffer);
        }
    }
    else if (result.uncertain_mine)
    {
        uart_sendStr(
            "Possible mine seen in only one sweep. Scan again to confirm.\r\n");
    }
    else if (result.gap_found)
    {
        sprintf(buffer, "Gap angle: %d\twidth: %.2f\r\n", result.gap_mid_angle,
                result.gap_width_cm);
        uart_sendStr(buffer);
    }
    else
    {
        uart_sendStr("No mine or gap found\r\n");
    }

    uart_sendStr("SCAN END\r\n");
}

// Reports an uncertain mine to both LCD and UART.
// This is used when only one of the two scan sweeps saw a possible mine, meaning
// the robot should rescan instead of immediately moving forward.
static void report_uncertain_mine(void)
{
    lcd_clear();
    lcd_printf("Uncertain mine\nRe-scanning...");
    uart_sendStr("Possible mine seen in only one sweep. Re-scanning...\r\n");
}

// Sends pending command-status messages produced by the UART interrupt handler.
// The interrupt stores a status byte, and the main loop prints it here so UART
// output happens from one predictable place.
static void send_pending_command_status(void)
{
    char status = command_status_byte;

    if (status == '\0')
    {
        return;
    }

    command_status_byte = '\0';

    switch (status)
    {
    case 'm':
        uart_sendStr("Manual Mode \r\n");
        break;
    case 'n':
        uart_sendStr("Auto Mode \r\n");
        break;
    case 'z':
        uart_sendStr("Auto Start \r\n");
        break;
    case 'x':
        uart_sendStr("Stop \r\n");
        break;
    case 'i':
        uart_sendStr("Scan Start \r\n");
        break;
    default:
        break;
    }
}

// Waits for the demining period to finish.
// During the wait, stop and manual commands are still honored so the robot can be
// interrupted instead of being locked in a delay.
static bool wait_for_demining(void)
{
    unsigned int wait_start = timer_getMillis();

    while ((timer_getMillis() - wait_start) < DEMINING_WAIT)
    {
        if (stop_flag || manual_flag)
        {
            stop_bot();
            return false;
        }
    }

    return true;
}

// Runs a scan while marking scan_active.
// The UART interrupt code can use scan_active to know that a scan is in progress
// and avoid conflicting command behavior.
static scan_result_t guarded_scan(bool manual_interrupt)
{
    scan_result_t result;

    scan_active = 1;
    result = scan_data(manual_interrupt);
    scan_active = 0;

    return result;
}

// LED stuff
// Flashes the Create LEDs red.
// The main loop calls this when entering manual-mode handling and it can also be
// used as a danger/attention indicator.
void flashRed()
{
    int i;
    for (i = 0; i < 3; i++)
    {
        oi_setLeds(1, 1, 255, 255); // red
        timer_waitMillis(100);
    }
}

// Flashes the Create LEDs green.
// The main loop calls this when autonomous mode is active.
void flashGreen()
{
    int i;
    for (i = 0; i < 3; i++)
    {
        oi_setLeds(1, 1, 0, 255); // green
        timer_waitMillis(100);
    }
}

// main

// Program entry point for the robot controller.
// It initializes hardware, tracks the robot pose, handles UART-driven mode
// changes, runs manual driving/scanning, and executes the autonomous mine-search
// and gap-navigation state machine.
int main(void)
{
    timer_init();
    lcd_init();

    oi_t *oi = oi_alloc();

    oi_init(oi);
    scan_init();

    uart_interrupt_init();

    int mines_found = 0;
    unsigned int last_pos_report = 0;
    unsigned int last_manual_cliff_report = 0;
    unsigned int last_manual_bump_report = 0;

    char buffer[100];

    lcd_printf("Starting...Press M/N\n");

    while (1)
    {
        update_pose(oi);
        if ((timer_getMillis() - last_pos_report) > 500)
        {
            report_position();
            last_pos_report = timer_getMillis();
        }

        send_pending_command_status();

        if (stop_flag)
        {
            reset_after_stop();
            continue;
        }

        if (manual_flag == 1)
        {
            flashRed();
            lcd_printf("MANUAL MODE");

            if (cliff_detected(oi))
            {
                if (last_manual_cliff_report == 0 ||
                    (timer_getMillis() - last_manual_cliff_report) > MANUAL_SAFETY_REPORT_MS)
                {
                    lcd_clear();
                    lcd_printf("Boundary/hole detected");

                    uart_sendStr("Boundary/hole detected\r\n");
                    last_manual_cliff_report = timer_getMillis();
                }

                if (manual_safety_stop_needed())
                {
                    stop_bot();
                    continue;
                }
            }

            if (bumped(oi))
            {
                if (last_manual_bump_report == 0 ||
                    (timer_getMillis() - last_manual_bump_report) > MANUAL_SAFETY_REPORT_MS)
                {
                    lcd_clear();
                    lcd_printf("Obstacle hit");

                    uart_sendStr("Obstacle hit\r\n");
                    last_manual_bump_report = timer_getMillis();
                }

                if (manual_safety_stop_needed())
                {
                    stop_bot();
                    continue;
                }
            }

            drive_manual_command();

            if (initiate_scan == 1)
            {
                stop_bot();
                move_byte = '\0';
                initiate_scan = 0;

                lcd_clear();
                lcd_printf("Scanning...");

                scan_result_t result = guarded_scan(false);

                send_manual_scan_result(result, buffer);
            }
            continue;
        }

        if (auto_flag == 1)
        {
            flashGreen();
            lcd_printf("AUTO MODE");

            if (!start_flag)
            {
                stop_bot();
                continue;
            }
            if (mines_found >= MIN_MINES)
            {
                stop_bot();

                lcd_clear();
                lcd_printf("All mines found!");
                uart_sendStr("All mines found!\r\n");
                auto_flag = 0;
                start_flag = 0;
                stop_flag = 1;

                break;
            }

            if (cliff_detected(oi))
            {
                uart_sendStr("Boundary/hole detected\r\n");
                // flashRed();
                handle_boundary_hole(oi);
                continue;
            }

            if (bumped(oi))
            {
                uart_sendStr("Obstacle hit\r\n");
                // flashRed();
                handle_bump(oi);
                continue;
            }

            scan_result_t result = guarded_scan(true);

            if (manual_flag)
            {
                stop_bot();
                continue;
            }

            if (stop_flag)
            {
                stop_bot();
                continue;
            }

            if (result.mine_count > 0)
            {
                int k;
                stop_bot();

                for (k = 0; k < result.mine_count; k++)
                {
                    if (stop_flag)
                    {
                        stop_bot();
                        break;
                    }

                    lcd_clear();
                    lcd_printf("Mine %d/%d\nAngle: %d", k + 1,
                               result.mine_count, result.mines[k].mid_angle);
                    sprintf(buffer,
                            "Mine %d/%d at angle %d dist %.2f width %.2f \r\n",
                            k + 1, result.mine_count, result.mines[k].mid_angle,
                            result.mines[k].distance_cm,
                            result.mines[k].width_cm);
                    uart_sendStr(buffer);

                    servo_move(result.mines[k].mid_angle);
                    unsigned char songStream[10] = {80, 70, 80, 70, 80, 70, 80,
                                                    70, 80, 70};
                    unsigned char length[10] = {24, 24, 24, 24, 24, 24, 24, 24,
                                                24, 24};
                    oi_loadSong(0, 10, songStream, length);
                    oi_play_song(0);
                    if (!wait_for_demining())
                    {
                        break;
                    }
                }

                servo_move(90);
                if (stop_flag || manual_flag)
                {
                    continue;
                }
                result = guarded_scan(true); // rescan after demining to check if mine is removed

                if (manual_flag)
                {
                    stop_bot();
                    continue;
                }

                if (result.mine_count > 0)
                {
                    uart_sendStr("Mine still exists. Re-scanning...\r\n");
                    lcd_clear();
                    lcd_printf("Mine still exists.\nRe-scanning...");
                    continue;
                }

                if (result.uncertain_mine)
                {
                    report_uncertain_mine();
                    move_checked(oi, -CLOSE_OBJECT_BACKUP_CM, false, false);
                    continue;
                }

                uart_sendStr("Mines cleared\r\n");
                mines_found += k;
            }

            if (result.uncertain_mine)
            {
                report_uncertain_mine();
                move_checked(oi, -CLOSE_OBJECT_BACKUP_CM, false, false);
                continue;
            }

            if (result.gap_found)
            {
                int turn_amount = result.gap_mid_angle - 90;

                lcd_clear();
                lcd_printf("Gap found!\nMoving...");

                if (result.object_count == 0)
                {
                    uart_sendStr(
                        "No mine or object found. Front is clear.\r\n");
                }
                else
                {
                    sprintf(buffer, "Gap found at angle %d width %.2f\r\n",
                            result.gap_mid_angle, result.gap_width_cm);
                    uart_sendStr(buffer);
                }

                sprintf(buffer, "Turning %d degrees to face gap\r\n",
                        turn_amount);
                uart_sendStr(buffer);

                if (turn_amount != 0)
                {
                    turn_degrees(oi, turn_amount);
                }

                move_safely(oi, STEP_FORWARD_CM);
            }
            else
            {
                lcd_clear();
                lcd_printf("No mine or gap\nTurning left...");

                uart_sendStr("No mine or gap found. Turning left.\r\n");

                turn_degrees(oi, 55);
            }
            continue;
        }

        stop_bot();
        show_idle_prompt();

        timer_waitMillis(100);
    }

    show_idle_prompt();

    timer_waitMillis(100);
    stop_bot();

    oi_free(oi);
    return 0;
}
