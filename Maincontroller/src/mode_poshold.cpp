/*
 * mode_poshold.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

static float x_target=0.0f,y_target=0.0f,target_yaw=0.0f,vx_target_bf=0.0f,vy_target_bf=0.0f,vel_lat_cms=0.0f,vel_lon_cms=0.0f;
static uint16_t target_point=0;
static Location gnss_target_pos;
static Vector3f ned_target_pos;
static Vector2f ned_dis_2d;
bool mode_poshold_init(void){
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
	usb_printf("switch mode poshold!\n");
	return true;
}

void mode_poshold(void){
	AltHoldModeState althold_state;
	float takeoff_climb_rate = 0.0f;
	float ch7=get_channel_7();
	// initialize vertical speeds and acceleration
	pos_control->set_speed_z(-param->pilot_speed_dn.value, param->pilot_speed_up.value);
	pos_control->set_accel_z(param->pilot_accel_z.value);

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
		x_target=get_ned_pos_x();
		y_target=get_ned_pos_y();
		pos_control->set_xy_target(x_target, y_target);
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
		if(ch7>=0.7&&ch7<=1.0){//手动模式(上挡位)
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			x_target=get_ned_pos_x();
			y_target=get_ned_pos_y();
			pos_control->set_xy_target(x_target, y_target);
		}else{//定位模式(下挡位)
			vx_target_bf=-get_channel_pitch()*100;//最大速度100cm/s
			vy_target_bf=get_channel_roll()*100;//最大速度100cm/s
			vel_lat_cms=vx_target_bf*ahrs_cos_yaw()-vy_target_bf*ahrs_sin_yaw();
			vel_lon_cms=vx_target_bf*ahrs_sin_yaw()+vy_target_bf*ahrs_cos_yaw();
			pos_control->set_desired_velocity_xy(vel_lat_cms,vel_lon_cms);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
		}
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
		target_yaw=ahrs_yaw_deg();
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		x_target=get_ned_pos_x();
		y_target=get_ned_pos_y();
		pos_control->set_xy_target(x_target, y_target);
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Flying:
		robot_state=STATE_FLYING;
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

		// call attitude controller
		if(ch7>=0.7&&ch7<=1.0){//手动模式(上挡位)
			target_yaw=ahrs_yaw_deg();
			attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
			x_target=get_ned_pos_x();
			y_target=get_ned_pos_y();
			pos_control->set_xy_target(x_target, y_target);
		}else if(ch7>0.3&&ch7<0.7){//定位模式(中挡位)
			target_yaw=ahrs_yaw_deg();
			vx_target_bf=-get_channel_pitch()*100;//最大速度100cm/s
			vy_target_bf=get_channel_roll()*100;//最大速度100cm/s
			vel_lat_cms=vx_target_bf*ahrs_cos_yaw()-vy_target_bf*ahrs_sin_yaw();
			vel_lon_cms=vx_target_bf*ahrs_sin_yaw()+vy_target_bf*ahrs_cos_yaw();
			pos_control->set_desired_velocity_xy(vel_lat_cms,vel_lon_cms);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		}else{//巡线模式
			if(sdlog->gnss_point_num>0){
				if(target_point<sdlog->gnss_point_num){
					gnss_target_pos.lat=(int32_t)(sdlog->gnss_point[target_point].x*1e7);
					gnss_target_pos.lng=(int32_t)(sdlog->gnss_point[target_point].y*1e7);
					ned_target_pos=location_3d_diff_NED(get_gnss_origin_pos(), gnss_target_pos)*100;//cm
					ned_dis_2d.x=ned_target_pos.x-get_pos_x();
					ned_dis_2d.y=ned_target_pos.y-get_pos_y();
					if(ned_dis_2d.length()<100){//距离目标点小于1m认为到达
						target_point++;
					}else{
						if(ned_dis_2d.y>=0){
							target_yaw=acosf(ned_dis_2d.x/ned_dis_2d.length())/M_PI*180;
						}else{
							target_yaw=-acosf(ned_dis_2d.x/ned_dis_2d.length())/M_PI*180;
						}
					}
				}
				//TODO:巡线结束后需要执行的策略
				pos_control->set_xy_target(ned_target_pos.x, ned_target_pos.y);
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
			}else{
				target_yaw=ahrs_yaw_deg();
				vx_target_bf=-get_channel_pitch()*100;//最大速度100cm/s
				vy_target_bf=get_channel_roll()*100;//最大速度100cm/s
				vel_lat_cms=vx_target_bf*ahrs_cos_yaw()-vy_target_bf*ahrs_sin_yaw();
				vel_lon_cms=vx_target_bf*ahrs_sin_yaw()+vy_target_bf*ahrs_cos_yaw();
				pos_control->set_desired_velocity_xy(vel_lat_cms,vel_lon_cms);
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
			}
		}

		if((!rangefinder_state.alt_healthy)&&((target_climb_rate+param->pilot_speed_dn.value)<10)){//cms
			//油门拉到最低时强制油门下降 注意：该功能只在surface tracking无效时使用
			set_thr_force_decrease(true);
		}else{
			set_thr_force_decrease(false);
		}

		if(robot_state_desired==STATE_DRIVE||robot_state_desired==STATE_LANDED){//自动降落
			target_climb_rate=-constrain_float(param->auto_land_speed.value, 0.0f, param->pilot_speed_dn.value);//设置降落速度cm/s
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
