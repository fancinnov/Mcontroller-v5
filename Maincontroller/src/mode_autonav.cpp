/*
 * mode_autonav.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

static float target_yaw=0.0f;
static bool jump=true;
static bool use_gcs=false, use_rc=false;
static bool execute_land=false;
static bool reach_target_point=false;
static Vector3f ned_target_pos;
static Vector2f ned_target_dis_2d, ned_dis_2d, vel_desired;
static bool get_first_pos=false;
static uint32_t takeoff_time=0, lock_time=0, safe_time=0;
static float yaw_delta=0.0f;
static float jump_alt=30.0f;
static uint16_t land_detect=0;
static float takeoff_alt=0.0f;
static bool hit_target_takeoff_alt=true;
bool mode_autonav_init(void){
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
	usb_printf("switch mode autonav!\n");
	return true;
}

void mode_autonav(void){
	AltHoldModeState althold_state;
	float takeoff_climb_rate = 0.0f;
	float ch7=get_channel_7();
	// initialize vertical speeds and acceleration
	pos_control->set_speed_z(-param->pilot_speed_dn.value, param->pilot_speed_up.value);
	pos_control->set_accel_z(param->pilot_accel_z.value);
	pos_control->set_speed_xy(param->poshold_vel_max.value);
	pos_control->set_accel_xy(param->poshold_accel_max.value);

	rangefinder_state.enabled=true;
	// get pilot desired lean angles
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(target_roll, target_pitch, param->angle_max.value, attitude->get_althold_lean_angle_max());

	// get pilot's desired yaw rate
	float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());
	// get pilot desired climb rate
	float target_climb_rate = get_pilot_desired_climb_rate(get_channel_throttle());
	target_climb_rate = constrain_float(target_climb_rate, -param->pilot_speed_dn.value, param->pilot_speed_up.value);

	if(get_force_autonav()&&!rc_channels_healthy()){//强制飞控进入自主模式
		ch7=-1.0f;
	}

	if(ch7<0.0f){//遥控器未连接
		target_roll=0.0f;
		target_pitch=0.0f;
		target_yaw_rate=0.0f;
		target_climb_rate=0.0f;
	}

	if(!hit_target_takeoff_alt){
		if(rangefinder_state.alt_healthy&&abs(rangefinder_state.alt_cm + get_vel_z()*0.2 - takeoff_alt)<10.0f){
			hit_target_takeoff_alt=true;
			pos_control->shift_alt_target(-pos_control->get_alt_error());
			takeoff_stop();
		}
	}

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
		execute_land=false;
		lock_time=HAL_GetTick();
		if(robot_state_desired==STATE_FLYING||robot_state_desired==STATE_TAKEOFF){
			arm_motors();
		}else{
			robot_state_desired=STATE_NONE;
		}
		motors->set_desired_spool_state(Motors::DESIRED_SHUT_DOWN);
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		attitude->set_yaw_target_to_current_heading();
		target_yaw=ahrs_yaw_deg();
		pos_control->set_xy_target(get_pos_x(), get_pos_y());
		pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Takeoff:
		if(HAL_GetTick()-lock_time<2000){
			break;
		}
		robot_state=STATE_TAKEOFF;
		// set motors to full range
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		motors->set_throttle_hover(0.35);//设置悬停油门, 需要根据不同的机型自行调整
		takeoff_alt=constrain_float(param->pilot_takeoff_alt.value,30.0f,200.0f);//最小30cm,最大200cm
		set_target_rangefinder_alt(takeoff_alt);
		// initiate take-off
		if (!takeoff_running()) {
			if(get_batt_volt()<param->lowbatt_land_volt.value){
				disarm_motors();
				break;
			}
			takeoff_start(takeoff_alt);
			hit_target_takeoff_alt=false;
			// indicate we are taking off
			set_land_complete(false);
			// clear i terms
			set_throttle_takeoff();

			if(jump){//起飞时直接跳起
				pos_control->get_accel_z_pid().set_integrator(0.0f);
				pos_control->set_alt_target(get_pos_z()+jump_alt);//设置目标高度比当前高度高jump_alt
			}
			takeoff_time=HAL_GetTick();
			if(get_gcs_connected()){
				use_gcs=true;
			}else{
				use_gcs=false;
			}
			if(rc_channels_healthy()){
				use_rc=true;
			}else{
				use_rc=false;
			}
		}

		// get take-off adjusted pilot and takeoff climb rates
		if(ch7<0.0f||target_climb_rate>=0.0f){
			target_climb_rate=100.0f;//给一个初速度
		}

		get_takeoff_climb_rates(target_climb_rate, takeoff_climb_rate);

		// call attitude controller
		if(ch7>=0.7&&ch7<=1.0){//姿态模式
			target_yaw=ahrs_yaw_deg();
			attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
			pos_control->set_xy_target(get_pos_x(), get_pos_y());
			pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		}else{//位置模式
			pos_control->set_pilot_desired_acceleration(target_roll, target_pitch, target_yaw, _dt);
			pos_control->calc_desired_velocity(_dt);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			target_yaw=ahrs_yaw_deg();
			attitude->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate);
		}
		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, _dt, false);
		pos_control->add_takeoff_climb_rate(takeoff_climb_rate, _dt);
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Landed:
		robot_state=STATE_LANDED;
		execute_land=false;
		// set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
		if (target_climb_rate < 0.0f) {
			motors->set_desired_spool_state(Motors::DESIRED_SPIN_WHEN_ARMED);
		} else {
			motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		}
		attitude->set_yaw_target_to_current_heading();
		target_yaw=ahrs_yaw_deg();
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		pos_control->set_xy_target(get_pos_x(), get_pos_y());
		pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Flying:
		robot_state=STATE_FLYING;
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

		// call attitude controller
		if((use_gcs&&!get_gcs_connected())||(use_rc&&!rc_channels_healthy())){
			robot_state_desired=STATE_LANDED;
			target_roll=0.0f;
			target_pitch=0.0f;
			target_yaw_rate=0.0f;
			ch7=0.5;//强制在位置模式下降落
		}

		if(ch7>=0.7&&ch7<=1.0){//姿态模式
			target_yaw+=target_yaw_rate*_dt;
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			pos_control->set_xy_target(get_pos_x(), get_pos_y());
			pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		}else if(ch7>0.3&&ch7<0.7){//位置模式
			target_yaw+=target_yaw_rate*_dt;
			if(get_gyro_filt().length()>M_PI){
				pos_control->set_xy_target(get_pos_x(), get_pos_y());
				pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			}else{
				pos_control->set_pilot_desired_acceleration(target_roll, target_pitch, target_yaw, _dt);
				pos_control->calc_desired_velocity(_dt);
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
			}
			if(rangefinder_state.alt_healthy&&(rangefinder_state.alt_cm<30.0f)){//降落检测
				if(target_climb_rate<-1.0f){
					land_detect++;
					if(land_detect>200){
						execute_land=true;
						land_detect=200;
					}
				}else{
					land_detect=0;
				}
			}else{
				land_detect=0;
			}
		}else{//自主模式
			if((HAL_GetTick()-takeoff_time)<2000){
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
			}else{
				target_yaw_rate=get_mav_yaw_rate_target();
				target_yaw+=target_yaw_rate*_dt;
				pos_control->set_speed_xy(param->mission_vel_max.value);
				pos_control->set_accel_xy(param->mission_accel_max.value);
				if(!get_first_pos){
					ned_target_pos.x=get_mav_x_target()*cosf(yaw_delta)+get_mav_y_target()*sinf(yaw_delta);
					ned_target_pos.y=-get_mav_x_target()*sinf(yaw_delta)+get_mav_y_target()*cosf(yaw_delta);
					get_first_pos=true;
				}
				if(reach_target_point){
					ned_target_pos.x=get_mav_x_target()*cosf(yaw_delta)+get_mav_y_target()*sinf(yaw_delta);
					ned_target_pos.y=-get_mav_x_target()*sinf(yaw_delta)+get_mav_y_target()*cosf(yaw_delta);
				}

				ned_dis_2d.x=ned_target_pos.x-pos_control->get_pos_target().x;//重新计算当前目标与上一个目标点的距离
				ned_dis_2d.y=ned_target_pos.y-pos_control->get_pos_target().y;
				if(ned_dis_2d.length()>1.0f){
					reach_target_point=false;
					vel_desired=ned_dis_2d.normalized()*30;//设置30cm/s的跟踪速度
					pos_control->set_desired_velocity_xy(vel_desired.x, vel_desired.y);
				}else{
					pos_control->set_desired_velocity_xy(0.0f, 0.0f);
					ned_target_dis_2d.x=ned_target_pos.x-get_pos_x();
					ned_target_dis_2d.y=ned_target_pos.y-get_pos_y();
					if(ned_target_dis_2d.length()<50.0){//距离目标点小于50cm认为到达
						reach_target_point=true;
					}
				}

				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
				target_climb_rate=get_mav_vz_target();
				if(get_mav_z_target()>=30.0f&&get_mav_z_target()<=200.0f&&rangefinder_state.alt_healthy){
					pos_control->shift_alt_target(get_mav_z_target()-rangefinder_state.alt_cm);
					set_target_rangefinder_alt(get_mav_z_target());
				}
			}
		}

		if(get_batt_volt()<param->lowbatt_land_volt.value&&(HAL_GetTick()-takeoff_time)>2000){//电量过低，强制降落
			execute_land=true;
		}

		if(robot_state_desired==STATE_LANDED||execute_land){//自动降落
			target_climb_rate=-constrain_float(param->auto_land_speed.value, 0.0f, param->pilot_speed_dn.value);//设置降落速度cm/s
		}

		if(target_climb_rate<-1.0f){
			if(rangefinder_state.alt_healthy&&(rangefinder_state.alt_cm<7.0f)){
				disarm_motors();
			}
		}

		if(get_vib_value()>10.0&&(HAL_GetTick()-takeoff_time)>2000){//猛烈撞击
			disarm_motors();
		}

		if(get_dcm_matrix().c.z>0.5f&&get_accel_filt().z<0){//倾角小于60度
			safe_time=HAL_GetTick();
		}else{
			if((HAL_GetTick()-safe_time)>5000){//大倾角
				disarm_motors();
			}
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
