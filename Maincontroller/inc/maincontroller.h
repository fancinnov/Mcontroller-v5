/*
 * maincontroller.h
 *
 *  Created on: Aug 28, 2021
 *      Author: 25053
 */
#pragma once

#ifndef INC_MAINCONTROLLER_H_
#define INC_MAINCONTROLLER_H_

#include "ahrs/ahrs.h"
#include "ekf/ekf_baro.h"
#include "ekf/ekf_rangefinder.h"
#include "ekf/ekf_odometry.h"
#include "ekf/ekf_gnss.h"
#include "position/position.h"
#include "compass/compassCalibrator.h"
#include "accel/accelCalibrator.h"
#include "flash/flash.h"
#include "sdlog/sdlog.h"
#include "uwb/uwb.h"
#include "common.h"

extern ap_t *ap;
extern AHRS *ahrs;
extern EKF_Baro *ekf_baro;
extern EKF_Rangefinder *ekf_rangefinder;
extern EKF_Odometry *ekf_odometry;
extern EKF_GNSS *ekf_gnss;
extern Motors *motors;
extern Attitude_Multi *attitude;
extern PosControl *pos_control;
extern CompassCalibrator *compassCalibrator;
extern AccelCalibrator *accelCalibrator;
extern DataFlash *dataflash;
extern SDLog *sdlog;

const float _dt=0.0025;
void send_mavlink_mission_ack(mavlink_channel_t chan, MAV_MISSION_RESULT result);
void send_mavlink_mission_item_reached(mavlink_channel_t chan, uint16_t seq);
void send_mavlink_mission_count(mavlink_channel_t chan);
void send_mavlink_mission_list(mavlink_channel_t chan);
void send_mavlink_commond_ack(mavlink_channel_t chan, MAV_CMD mav_cmd, MAV_CMD_ACK result);
float get_non_takeoff_throttle(void);
void zero_throttle_and_relax_ac(void);
void set_land_complete(bool b);
void get_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit);
float get_pilot_desired_yaw_rate(float stick_angle);
float get_pilot_desired_throttle(float throttle_control, float thr_mid);
float get_pilot_desired_climb_rate(float throttle_control);
void set_takeoff(bool set);
bool get_takeoff(void);
bool takeoff_running(void);
bool takeoff_triggered( float target_climb_rate);
void takeoff_start(float alt_cm);
void set_throttle_takeoff(void);
void get_takeoff_climb_rates(float& pilot_climb_rate,  float& takeoff_climb_rate);
float get_surface_tracking_climb_rate(float target_rate, float current_alt_target, float dt);

#endif /* INC_MAINCONTROLLER_H_ */
