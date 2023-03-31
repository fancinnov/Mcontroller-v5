/*
 * mode_poshold.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

static float target_yaw=0.0f;
static uint16_t target_point=0;
static Location gnss_target_pos;
static Vector3f ned_target_pos, ned_last_pos, ned_takeoff_pos;
static Vector2f ned_dis_2d, ned_dis_2d_smooth;
static bool use_gcs=false;
static bool execute_return=false;
static bool execute_land=false;
static bool reach_return_alt=false;
static float smooth_dt=0.0f;
static uint8_t notify=0;
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
	update_air_resistance();
	rangefinder_state.enabled=true;
	// initialize vertical speeds and acceleration
	pos_control->set_speed_z(-param->pilot_speed_dn.value, param->pilot_speed_up.value);
	pos_control->set_accel_z(param->pilot_accel_z.value);
	pos_control->set_speed_xy(param->poshold_vel_max.value);
	float vel_2d=sqrtf(sq(get_vel_x(),get_vel_y()));
	if(vel_2d<500){
		pos_control->set_accel_xy(param->poshold_accel_max.value);
	}else if(vel_2d<800){
		pos_control->set_accel_xy(param->poshold_accel_max.value/2);
	}else{
		pos_control->set_accel_xy(param->poshold_accel_max.value/3);
	}

	if((use_gcs&&!get_gcs_connected())||get_return()||(get_batt_volt()<param->lowbatt_return_volt.value)){//电量较低或地面站断开连接，强制返航
		execute_return=true;
	}
	if(get_batt_volt()<param->lowbatt_land_volt.value){//电量过低，强制降落
		execute_land=true;
	}

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
		execute_return=false;
		execute_land=false;
		reach_return_alt=false;
		set_return(false);
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
		pos_control->set_xy_target(get_pos_x(), get_pos_y());
		pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Takeoff:
		robot_state=STATE_TAKEOFF;
		// set motors to full range
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

		// initiate take-off
		if (!takeoff_running()) {
			if(execute_return||execute_land){
				disarm_motors();
				break;
			}
			takeoff_start(constrain_float(param->pilot_takeoff_alt.value,0.0f,1000.0f));
			// indicate we are taking off
			set_land_complete(false);
			// clear i terms
			set_throttle_takeoff();
			set_thr_force_decrease(false);//起飞时禁止限制油门
			ned_takeoff_pos.x=get_pos_x();
			ned_takeoff_pos.y=get_pos_y();
			ned_takeoff_pos.z=get_pos_z();
			if(get_gcs_connected()){
				use_gcs=true;
			}else{
				use_gcs=false;
			}
		}

		// get take-off adjusted pilot and takeoff climb rates
		get_takeoff_climb_rates(target_climb_rate, takeoff_climb_rate);

		// call attitude controller
		if(!get_gps_state()){//定位丢失，强制手动
			target_yaw=ahrs_yaw_deg();
			attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
			pos_control->set_xy_target(get_pos_x(), get_pos_y());
			pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		}else{
			if(ch7>=0.7&&ch7<=1.0){//手动姿态
				target_yaw=ahrs_yaw_deg();
				attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
				pos_control->set_xy_target(get_pos_x(), get_pos_y());
				pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
			}else{
				pos_control->set_pilot_desired_acceleration(target_roll, target_pitch, target_yaw, _dt);
				pos_control->calc_desired_velocity(_dt);
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				target_yaw=ahrs_yaw_deg();
				attitude->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate);
			}
		}

		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, _dt, false);
		pos_control->add_takeoff_climb_rate(takeoff_climb_rate, _dt);
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Landed:
		robot_state=STATE_LANDED;
		execute_return=false;
		execute_land=false;
		reach_return_alt=false;
		set_return(false);
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
		pos_control->set_xy_target(get_pos_x(), get_pos_y());
		pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Flying:
		robot_state=STATE_FLYING;
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

		if(!get_gps_state()){//定位丢失，强制手动
			target_yaw+=target_yaw_rate*_dt;
			get_air_resistance_lean_angles(target_roll, target_pitch, DEFAULT_ANGLE_MAX, 0.5f);
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			pos_control->set_xy_target(get_pos_x(), get_pos_y());
			pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
			if(execute_return||execute_land){//如果是正在执行返航过程中定位丢失，那么强制降落
				target_climb_rate=-constrain_float(param->auto_land_speed.value, 0.0f, param->pilot_speed_dn.value);//设置降落速度cm/s
			}
		}else if(execute_land){
			target_yaw+=target_yaw_rate*_dt;
			pos_control->set_pilot_desired_acceleration(target_roll, target_pitch, target_yaw, _dt);
			pos_control->calc_desired_velocity(_dt);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
			target_climb_rate=-constrain_float(param->auto_land_speed.value, 0.0f, param->pilot_speed_dn.value);//设置降落速度cm/s
		}else if(execute_return){
			pos_control->set_speed_xy(param->mission_vel_max.value);
			pos_control->set_accel_xy(param->mission_accel_max.value);
			if(reach_return_alt){
				ned_target_pos.x=ned_takeoff_pos.x;
				ned_target_pos.y=ned_takeoff_pos.y;
				ned_dis_2d.x=ned_target_pos.x-get_pos_x();
				ned_dis_2d.y=ned_target_pos.y-get_pos_y();
				if(ned_dis_2d.length()<200){//距离目标点小于2m认为到达
					execute_land=true;
				}else{
					if(ned_dis_2d.y>=0){
						target_yaw=acosf(ned_dis_2d.x/ned_dis_2d.length())/M_PI*180;
					}else{
						target_yaw=-acosf(ned_dis_2d.x/ned_dis_2d.length())/M_PI*180;
					}
					smooth_dt+=_dt;
					ned_dis_2d.x=ned_target_pos.x-ned_last_pos.x;//重新计算当前目标与上一个目标点的距离
					ned_dis_2d.y=ned_target_pos.y-ned_last_pos.y;
					ned_dis_2d_smooth=ned_dis_2d.normalized()*(param->mission_vel_max.value*smooth_dt+param->mission_accel_max.value*smooth_dt*smooth_dt/2);
					if(ned_dis_2d_smooth.length()<ned_dis_2d.length()){//将目标点进行平滑修正
						ned_target_pos.x=ned_last_pos.x+ned_dis_2d_smooth.x;
						ned_target_pos.y=ned_last_pos.y+ned_dis_2d_smooth.y;
					}
				}
				pos_control->set_xy_target(ned_target_pos.x, ned_target_pos.y);
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
			}else{
				float return_alt_cm=ned_takeoff_pos.z+param->alt_return.value;
				float delta_cm=abs(get_pos_z()-return_alt_cm);
				if(delta_cm<100){//距离目标高度小于1m认为到达
					reach_return_alt=true;
					ned_last_pos.x=get_pos_x();
					ned_last_pos.y=get_pos_y();
					smooth_dt=0.0f;
				}
				if(get_pos_z()<return_alt_cm){
					target_climb_rate=param->pilot_speed_up.value;
				}else{
					target_climb_rate=-param->pilot_speed_dn.value;
				}
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
			}
		}else{
			// call attitude controller
			if(ch7>=0.7&&ch7<=1.0){//手动模式(上挡位)
				target_yaw+=target_yaw_rate*_dt;
				get_air_resistance_lean_angles(target_roll, target_pitch, DEFAULT_ANGLE_MAX, 0.5f);
				attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
				pos_control->set_xy_target(get_pos_x(), get_pos_y());
				pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
			}else if(ch7>0.3&&ch7<0.7){//定位模式(中挡位)
				target_yaw+=target_yaw_rate*_dt;
				pos_control->set_pilot_desired_acceleration(target_roll, target_pitch, target_yaw, _dt);
				pos_control->calc_desired_velocity(_dt);
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
			}else{//巡线模式
				if(sdlog->gnss_point_num>0){
					pos_control->set_speed_xy(param->mission_vel_max.value);
					pos_control->set_accel_xy(param->mission_accel_max.value);
					if(get_gnss_reset_notify()!=notify){
						notify=get_gnss_reset_notify();
						target_point=0;
					}
					if(target_point<sdlog->gnss_point_num){
						if(target_point==0){
							gnss_target_pos.lat=(int32_t)(sdlog->gnss_point[0].x*1e7);
							gnss_target_pos.lng=(int32_t)(sdlog->gnss_point[0].y*1e7);
							ned_last_pos=location_3d_diff_NED(get_gnss_origin_pos(), gnss_target_pos)*100;//cm
							smooth_dt=0.0f;
							target_point++;
						}
						gnss_target_pos.lat=(int32_t)(sdlog->gnss_point[target_point].x*1e7);
						gnss_target_pos.lng=(int32_t)(sdlog->gnss_point[target_point].y*1e7);
						ned_target_pos=location_3d_diff_NED(get_gnss_origin_pos(), gnss_target_pos)*100;//cm
						ned_dis_2d.x=ned_target_pos.x-get_pos_x();
						ned_dis_2d.y=ned_target_pos.y-get_pos_y();
						if(ned_dis_2d.length()<200){//距离目标点小于2m认为到达
							ned_last_pos=ned_target_pos;
							target_point++;
							smooth_dt=0.0f;
						}else{
							if(ned_dis_2d.y>=0){
								target_yaw=acosf(ned_dis_2d.x/ned_dis_2d.length())/M_PI*180;
							}else{
								target_yaw=-acosf(ned_dis_2d.x/ned_dis_2d.length())/M_PI*180;
							}
							smooth_dt+=_dt;
							ned_dis_2d.x=ned_target_pos.x-ned_last_pos.x;//重新计算当前目标与上一个目标点的距离
							ned_dis_2d.y=ned_target_pos.y-ned_last_pos.y;
							ned_dis_2d_smooth=ned_dis_2d.normalized()*(param->mission_vel_max.value*smooth_dt+param->mission_accel_max.value*smooth_dt*smooth_dt/2);
							if(ned_dis_2d_smooth.length()<ned_dis_2d.length()){//将目标点进行平滑修正
								ned_target_pos.x=ned_last_pos.x+ned_dis_2d_smooth.x;
								ned_target_pos.y=ned_last_pos.y+ned_dis_2d_smooth.y;
							}
						}
					}
					//TODO:巡线结束后需要执行的策略
					pos_control->set_xy_target(ned_target_pos.x, ned_target_pos.y);
					pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
					attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
				}else{
					target_yaw+=target_yaw_rate*_dt;
					pos_control->set_pilot_desired_acceleration(target_roll, target_pitch, target_yaw, _dt);
					pos_control->calc_desired_velocity(_dt);
					pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
					attitude->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw, true);
				}
			}
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
