/*
 * mode_althold.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

static float x_target=0.0f,y_target=0.0f,target_yaw=0.0f;

bool mode_poshold_init(void){
	if(motors->get_armed()||motors->get_interlock()){//电机未锁定,禁止切换至该模式
		Buzzer_set_ring_type(BUZZER_ERROR);
		return false;
	}
	if(!pos_control->is_active_z()){
		// initialize position and desired velocity
		pos_control->set_alt_target_to_current_alt();
		pos_control->set_desired_velocity_z(get_vel_z());
	}
	set_manual_throttle(false);//设置为自动油门
	Buzzer_set_ring_type(BUZZER_MODE_SWITCH);
	usb_printf("switch mode poshold!\n");
	return true;
}

void mode_poshold(void){
	AltHoldModeState althold_state;
	float takeoff_climb_rate = 0.0f;
	float ch7=0.0f;
	// initialize vertical speeds and acceleration
	pos_control->set_speed_z(-param->pilot_speed_dn.value, param->pilot_speed_up.value);
	pos_control->set_accel_z(param->pilot_accel_z.value);

	// get pilot desired lean angles
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(target_roll, target_pitch, param->angle_max.value, attitude->get_althold_lean_angle_max());
//	usb_printf("r:%f,p:%f\n",target_roll, target_pitch);

	// get pilot's desired yaw rate
	float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());
//	usb_printf("y:%f\n",target_yaw_rate);

	// get pilot desired climb rate
	float target_climb_rate = get_pilot_desired_climb_rate(get_channel_throttle());
	target_climb_rate = constrain_float(target_climb_rate, -param->pilot_speed_dn.value, param->pilot_speed_up.value);

	// Alt Hold State Machine Determination
	if (!motors->get_armed()) {
		althold_state = AltHold_MotorStopped;
	} else if (takeoff_running() || takeoff_triggered(target_climb_rate) || get_takeoff()) {
		althold_state = AltHold_Takeoff;
	} else if (ap->land_complete) {
		althold_state = AltHold_Landed;
	} else {
		althold_state = AltHold_Flying;
	}

	// Alt Hold State Machine
	switch (althold_state) {

	case AltHold_MotorStopped:
		motors->set_desired_spool_state(Motors::DESIRED_SHUT_DOWN);
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		attitude->reset_rate_controller_I_terms();
		attitude->set_yaw_target_to_current_heading();
		target_yaw=ahrs_yaw_deg();
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Takeoff:
		// set motors to full range
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

		// initiate take-off
		if (!takeoff_running()) {
			takeoff_start(constrain_float(param->pilot_takeoff_alt.value,0.0f,1000.0f));
			// indicate we are taking off
			set_land_complete(false);
			// clear i terms
			set_throttle_takeoff();
		}
//		usb_printf("target_climb_rate:%f\n",target_climb_rate);
		// get take-off adjusted pilot and takeoff climb rates
		get_takeoff_climb_rates(target_climb_rate, takeoff_climb_rate);

		// call attitude controller
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		target_yaw=ahrs_yaw_deg();
		x_target=get_ned_pos_x();
		y_target=get_ned_pos_y();
		pos_control->set_xy_target(x_target, y_target);
		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, _dt, false);
		pos_control->add_takeoff_climb_rate(takeoff_climb_rate, _dt);
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Landed:
		// set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
		if (target_climb_rate < 0.0f) {
			motors->set_desired_spool_state(Motors::DESIRED_SPIN_WHEN_ARMED);
		} else {
			motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		}

		attitude->reset_rate_controller_I_terms();
		attitude->set_yaw_target_to_current_heading();
		target_yaw=ahrs_yaw_deg();
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		x_target=get_ned_pos_x();
		y_target=get_ned_pos_y();
		pos_control->set_xy_target(x_target, y_target);
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Flying:
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		// call attitude controller
//		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		ch7=get_channel_7();
//		usb_printf("%f\n",ch6);
		if(!get_gnss_location_state()){//没有机载电脑，强制手动姿态
			ch7=0.95f;
		}
		if(ch7>=0.7&&ch7<1.0){//手动姿态(上挡位)
			attitude->bf_feedforward(true);
			target_yaw+=target_yaw_rate*_dt;
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			x_target=get_ned_pos_x();
			y_target=get_ned_pos_y();
			pos_control->set_xy_target(x_target, y_target);
		}else if(ch7>0.3&&ch7<0.7){//手动俯仰滚转,锁定偏航(中挡位)
			attitude->bf_feedforward(true);
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			x_target=get_ned_pos_x();
			y_target=get_ned_pos_y();
			pos_control->set_xy_target(x_target, y_target);
		}else if(ch7>0.0&&ch7<=0.3){//自动姿态(下挡位)
			attitude->bf_feedforward(false);
			pos_control->set_xy_target(x_target, y_target);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
		}

		// adjust climb rate using rangefinder
		target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), _dt);

		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, _dt, false);

		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;
	default:
		break;
	}
	attitude->rate_controller_run();
	motors->output();
}
