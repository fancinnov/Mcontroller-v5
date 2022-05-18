/*
 * mode_autonav.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

static float target_yaw=0.0f;
static float thrust=0.0f;
static bool jump=false;
bool mode_autonav_init(void){
	if(!pos_control->is_active_z()){
		// initialize position and desired velocity
		pos_control->set_alt_target_to_current_alt();
		pos_control->set_desired_velocity_z(get_vel_z());
	}else{
		return false;
	}
	set_manual_throttle(false);//设置为自动油门
	usb_printf("switch mode autonav!\n");
	return true;
}

static float ch7=0.0f;
static float theta=0.0f;
void mode_autonav(void){
	AltHoldModeState althold_state;
	float takeoff_climb_rate = 0.0f;

	// initialize vertical speeds and acceleration
	pos_control->set_speed_z(-param->pilot_speed_dn.value, param->pilot_speed_up.value);
	pos_control->set_accel_z(param->pilot_accel_z.value);
	pos_control->set_speed_xy(25);
	pos_control->set_accel_xy(20);

	// get pilot desired lean angles
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(target_roll, target_pitch, param->angle_max.value, attitude->get_althold_lean_angle_max());

	// get pilot's desired yaw rate
	float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());

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
		robot_state=STATE_STOP;
		motors->set_desired_spool_state(Motors::DESIRED_SHUT_DOWN);
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		attitude->get_rate_roll_pid().set_integrator(param->rate_pid_integrator.value.x);
		attitude->get_rate_pitch_pid().set_integrator(param->rate_pid_integrator.value.y);
		attitude->get_rate_yaw_pid().set_integrator(param->rate_pid_integrator.value.z);
		attitude->set_yaw_target_to_current_heading();
		target_yaw=ahrs_yaw_deg();
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Takeoff:
		robot_state=STATE_TAKEOFF;
		// set motors to full range
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

		// initiate take-off
		if (!takeoff_running()) {
			takeoff_start(constrain_float(param->pilot_takeoff_alt.value,0.0f,1000.0f));
			// indicate we are taking off
			set_land_complete(false);
			// clear i terms
			set_throttle_takeoff();
			set_thr_force_decrease(false);//起飞时禁止限制油门
			jump=true;
		}

		// get take-off adjusted pilot and takeoff climb rates
		get_takeoff_climb_rates(target_climb_rate, takeoff_climb_rate);

		// call attitude controller
		ch7=get_channel_7();
		if(ch7>=0.7&&ch7<1.0){//手动姿态
			target_yaw+=target_yaw_rate*_dt;
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
		}else if(ch7>0.3&&ch7<0.7){//定点
			pos_control->set_xy_target(250.0f, -200.0f);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
		}else if(ch7>0.0&&ch7<=0.3){//定点
			pos_control->set_xy_target(250.0f, -200.0f);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
		}else{
			target_yaw+=target_yaw_rate*_dt;
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
		}
		// call position controller
		if(jump){//起飞时直接跳起30cm
			if(thrust>0.3){//0.3是阈值，只需要比悬停油门大一点, 需要根据不同的机型自行调整
				thrust=0;
				pos_control->get_accel_z_pid().set_integrator(250);//set i, 需要根据不同的机型自行调整
				pos_control->set_alt_target(get_pos_z()+30);//设置目标高度比当前高度高30cm
				jump=false;
			}else{//jump
				thrust+=0.005;
				attitude->set_throttle_out(thrust, true, 10.0f);
				break;
			}
		}
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, _dt, false);
		pos_control->add_takeoff_climb_rate(takeoff_climb_rate, _dt);
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Landed:
		robot_state=STATE_LANDED;
		// set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
		if (target_climb_rate < 0.0f) {
			motors->set_desired_spool_state(Motors::DESIRED_SPIN_WHEN_ARMED);
		} else {
			motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		}

		attitude->get_rate_roll_pid().set_integrator(param->rate_pid_integrator.value.x);
		attitude->get_rate_pitch_pid().set_integrator(param->rate_pid_integrator.value.y);
		attitude->get_rate_yaw_pid().set_integrator(param->rate_pid_integrator.value.z);
		attitude->set_yaw_target_to_current_heading();
		target_yaw=ahrs_yaw_deg();
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Flying:
		if(get_vib_value()>0.1){
			robot_state=STATE_FLYING;
		}else{
			robot_state=STATE_FLYING_VIRTUAL;
		}

		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		// call attitude controller

		ch7=get_channel_7();
		if(ch7>=0.7&&ch7<1.0){//手动姿态
			target_yaw+=target_yaw_rate*_dt;
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
		}else if(ch7>0.3&&ch7<0.7){//定点
			theta=0.0f;
			pos_control->set_xy_target(250.0f, -200.0f);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
		}else if(ch7>0.0&&ch7<=0.3){//画圆
			theta+=(M_PI/12/400);
			pos_control->set_xy_target(200+50*cosf(theta), -200.0f+50*sinf(theta));
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
		}else{
			target_yaw+=target_yaw_rate*_dt;
		    attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
		}

		// adjust climb rate using rangefinder
		target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), _dt);

		if((!rangefinder_state.alt_healthy)&&((target_climb_rate+param->pilot_speed_dn.value)<10)){//cms
			//油门拉到最低时强制油门下降 注意：该功能只在surface tracking无效时使用
			set_thr_force_decrease(true);
		}else{
			set_thr_force_decrease(false);
		}

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
