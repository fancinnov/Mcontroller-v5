/*
 * mode_stabilize.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

bool mode_stabilize_init(void){
	// if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
	if (motors->get_armed() && ap->land_complete && !has_manual_throttle() &&
			(get_pilot_desired_throttle(get_channel_throttle(), 0.0f) > get_non_takeoff_throttle())) {
		return false;
	}
	// set target altitude to zero for reporting
	pos_control->set_alt_target(0);
	return true;
}

void mode_stabilize(void){
	float target_roll, target_pitch;
	float target_yaw_rate;
	float pilot_throttle_scaled;

	// if not armed set throttle to zero and exit immediately
	if (!motors->get_armed() || ap->throttle_zero || !motors->get_interlock()) {
		zero_throttle_and_relax_ac();
		return;
	}

	// clear landing flag
	set_land_complete(false);

	motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

	// convert pilot input to lean angles
	get_pilot_desired_lean_angles(target_roll, target_pitch, param->angle_max.value, param->angle_max.value);

	// get pilot's desired yaw rate
	target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());

	// get pilot's desired throttle
	pilot_throttle_scaled = get_pilot_desired_throttle(get_channel_throttle(),0.0f);

	// call attitude controller
	attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

	// output pilot's throttle
	attitude->set_throttle_out(pilot_throttle_scaled, true, param->throttle_filt.value);

	attitude->rate_controller_run();
	motors->output();
}
