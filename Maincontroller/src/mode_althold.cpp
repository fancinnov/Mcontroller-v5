/*
 * mode_althold.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

static float target_yaw=0.0f;
bool mode_althold_init(void){
	if(motors->get_armed()){//电机未锁定,禁止切换至该模式
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
	usb_printf("switch mode althold!\n");
	return true;
}

void mode_althold(void){
	AltHoldModeState althold_state;
	float takeoff_climb_rate = 0.0f;
	// initialize vertical speeds and acceleration
	pos_control->set_speed_z(-param->pilot_speed_dn.value, param->pilot_speed_up.value);//设置z轴最大最小速度
	pos_control->set_accel_z(param->pilot_accel_z.value);//设置z轴最大加速度

	// get pilot desired lean angles
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(target_roll, target_pitch, param->angle_max.value, attitude->get_althold_lean_angle_max());//获取摇杆的目标俯仰滚转角

	// get pilot's desired yaw rate
	float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());//获取摇杆设置的目标偏航角速度

	// get pilot desired climb rate
	float target_climb_rate = get_pilot_desired_climb_rate(get_channel_throttle());//获取摇杆设置的目标爬升速率
	target_climb_rate = constrain_float(target_climb_rate, -param->pilot_speed_dn.value, param->pilot_speed_up.value);

	// Alt Hold State Machine Determination
	//更新飞行状态
	if (!motors->get_armed()) {
		althold_state = AltHold_MotorStopped;//电机锁定状态
	} else if (takeoff_running() || takeoff_triggered(target_climb_rate) || get_takeoff()) {
		althold_state = AltHold_Takeoff;//起飞状态
	} else if (ap->land_complete) {
		althold_state = AltHold_Landed;//降落状态
	} else {
		althold_state = AltHold_Flying;//飞行中状态
	}

	// Alt Hold State Machine
	switch (althold_state) {

	case AltHold_MotorStopped:
		robot_state=STATE_STOP;
		if(robot_state_desired==STATE_FLYING||robot_state_desired==STATE_TAKEOFF){
			if(target_climb_rate<=0){//油门在低位保证安全
				arm_motors();
			}
		}
		motors->set_desired_spool_state(Motors::DESIRED_SHUT_DOWN);
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		attitude->reset_rate_controller_I_terms();
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
		}

		// get take-off adjusted pilot and takeoff climb rates
		get_takeoff_climb_rates(target_climb_rate, takeoff_climb_rate);

		// call attitude controller
		attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);

		// call position controller
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
		if(robot_state_desired==STATE_DRIVE||robot_state_desired==STATE_LANDED){
			disarm_motors();
		}
		attitude->reset_rate_controller_I_terms();
		attitude->set_yaw_target_to_current_heading();
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		target_yaw=ahrs_yaw_deg();
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Flying:
		robot_state=STATE_FLYING;
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		// call attitude controller
		target_yaw+=target_yaw_rate*_dt;
		attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);//姿态控制外环
		if((!rangefinder_state.alt_healthy)&&((target_climb_rate+param->pilot_speed_dn.value)<10)){//cms
			//油门拉到最低时强制油门下降 注意：该功能只在surface tracking无效时使用
			set_thr_force_decrease(true);
		}else{
			set_thr_force_decrease(false);
		}

		if(robot_state_desired==STATE_DRIVE||robot_state_desired==STATE_LANDED){//自动降落
			target_climb_rate=-constrain_float(param->auto_land_speed.value, 0.0f, param->pilot_speed_dn.value);//设置降落速度cm/s
		}

		// surface tracking that adjust climb rate using rangefinder
		target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), _dt);//地形跟随

		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, _dt, false);//从目标爬升速率设置目标高度

		pos_control->update_z_controller(get_pos_z(), get_vel_z());//高度控制串级PID
		break;
	default:
		break;
	}
	attitude->rate_controller_run();//角速率PID控制（姿态控制内环）
	motors->output();//电机输出
}
