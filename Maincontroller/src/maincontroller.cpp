/*
 * maincontroller.cpp
 *
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */
#include "maincontroller.h"

/*************配置rangefinder ekf定高*************
 * 1.在freertos中启动ekf_rf_alt();
 * 2.更改get_pos_z()中的返回值为ekf_rangefinder->pos_z;
 * 3.更改get_vel_z()中的返回值为ekf_rangefinder->vel_z;
 * ****************************************/

/************Noted*************
 *
 * 控制不稳定,与传感器噪声、机架震动、机体质量分布有很大关系,
 * 需要调filter参数、ekf参数、pid参数、hover_throttle参数
 *
 * ****************************/

static bool accel_cal_succeed=true;
static bool gyro_cal_succeed=false;
static bool compass_cal_succeed=false;
static bool ahrs_stage_compass=false;
static bool get_baro_alt_filt=false;
static bool initial_accel_gyro=false;
static bool initial_mag=false;
static bool initial_baro=false;
static bool calibrate_failure=false;
static bool initial_accel_cal=false;
static bool horizon_correct=false;
static bool ahrs_healthy=false;
static bool initial_compass_cal=false;
static bool initial_gnss=false;
static bool get_gnss_location=false;
static bool get_rangefinder_data=false;
static bool get_mav_yaw=false, get_odom_xy=false;
static bool mag_corrected=false, mag_correcting=false;

static float accel_filt_hz=10;//HZ
static float gyro_filt_hz=20;//HZ
static float mag_filt_hz=2.5;//HZ
static float baro_filt_hz=5;//HZ
static float accel_ef_filt_hz=10;//HZ
static float uwb_pos_filt_hz=5;//HZ
static float pitch_rad_raw=0 , roll_rad_raw=0 , yaw_rad_raw=0;
static float pitch_rad=0 , roll_rad=0 , yaw_rad=0;
static float pitch_deg=0 , roll_deg=0 , yaw_deg=0;
static float cos_roll=0, cos_pitch=0, cos_yaw=0, sin_roll=0, sin_pitch=0, sin_yaw=0;
static float yaw_map=0.0f;
static float mav_x_target=0.0f, mav_y_target=0.0f, mav_vx_target=0.0f, mav_vy_target=0.0f;
static float completion_percent=0;

static Vector3f accel, gyro, mag;								//原生加速度、角速度、磁罗盘测量值
static Vector3f accel_correct, gyro_correct, mag_correct;		//修正后的加速度、角速度、磁罗盘测量值
static Vector3f accel_filt, gyro_filt, mag_filt;				//滤波优化后的加速度、角速度、磁罗盘测量值
static Vector3f accel_ef, gyro_ef;								//地球坐标系下的三轴加速度、角速度
static Vector3f gyro_offset;
static Vector3f odom_3d,odom_offset;
static Location gnss_origin_pos, gnss_current_pos;
static Vector3f ned_current_pos, ned_current_vel;
static Matrix3f dcm_matrix, dcm_matrix_correct;										//旋转矩阵
static LowPassFilter2pVector3f	_accel_filter, _gyro_filter, _accel_ef_filter;
static LowPassFilter2pFloat _baro_alt_filter;
static LowPassFilterVector3f _mag_filter, _uwb_pos_filter;

parameter *param=new parameter();
ap_t *ap=new ap_t();
AHRS *ahrs=new AHRS(_dt);
EKF_Baro *ekf_baro=new EKF_Baro(_dt, 0.0016, 0.000016, 0.000016);
EKF_Rangefinder *ekf_rangefinder=new EKF_Rangefinder(_dt, 1.0, 0.000016, 0.16);
EKF_Odometry *ekf_odometry=new EKF_Odometry(_dt, 0.25, 0.25, 0.0004, 0.04, 0.0004, 0.04);
EKF_GNSS *ekf_gnss=new EKF_GNSS(_dt, 1.0, 1.0, 0.0000001, 0.001, 0.0000001, 0.001);
Motors *motors=new Motors(1/_dt);
Attitude_Multi *attitude=new Attitude_Multi(*motors, gyro_filt, _dt);
PosControl *pos_control=new PosControl(*motors, *attitude);
CompassCalibrator *compassCalibrator=new CompassCalibrator();
AccelCalibrator *accelCalibrator=new AccelCalibrator();
DataFlash *dataflash=new DataFlash();
SDLog *sdlog=new SDLog();

float ahrs_pitch_rad(void){return pitch_rad;}
float ahrs_roll_rad(void){return roll_rad;}
float ahrs_yaw_rad(void){return yaw_rad;}
float ahrs_pitch_deg(void){return pitch_deg;}
float ahrs_roll_deg(void){return roll_deg;}
float ahrs_yaw_deg(void){return yaw_deg;}
float ahrs_cos_roll(void){return cos_roll;}
float ahrs_sin_roll(void){return sin_roll;}
float ahrs_cos_pitch(void){return cos_pitch;}
float ahrs_sin_pitch(void){return sin_pitch;}
float ahrs_cos_yaw(void){return cos_yaw;}
float ahrs_sin_yaw(void){return sin_yaw;}

const Vector3f& get_accel_correct(void){return accel_correct;}	//修正后的三轴机体加速度
const Vector3f& get_gyro_correct(void){return gyro_correct;}	//修正后的三轴机体角速度
const Vector3f& get_mag_correct(void){return mag_correct;}		//修正后的三轴磁场强度
const Vector3f& get_accel_filt(void){return accel_filt;}		//滤波后的三轴机体加速度
const Vector3f& get_gyro_filt(void){return gyro_filt;}			//滤波后的三轴机体角速度
const Vector3f& get_mag_filt(void){return mag_filt;}			//滤波后的三轴磁场强度

const Vector3f& get_accel_ef(void){								//地球坐标系下的三轴加速度
	return accel_ef;
}

const Vector3f& get_gyro_ef(void){								//地球坐标系下的三轴角速度
	return gyro_ef;
}

//return the vib_value
float get_vib_value(void){//震动强度
  return ahrs->vib_value;
}

//return the vib_angle
float get_vib_angle_z(void){//震动向量与大地坐标系z轴的夹角
  return ahrs->vib_angle;
}

float get_channel_roll_angle(void){return get_channel_roll()*ROLL_PITCH_YAW_INPUT_MAX;}
float get_channel_pitch_angle(void){return get_channel_pitch()*ROLL_PITCH_YAW_INPUT_MAX;}
float get_channel_yaw_angle(void){return get_channel_yaw()*ROLL_PITCH_YAW_INPUT_MAX;}

float get_ned_pos_x(void){return ned_current_pos.x;}
float get_ned_pos_y(void){return ned_current_pos.y;}
float get_ned_pos_z(void){return ned_current_pos.z;}
float get_ned_vel_x(void){return ned_current_vel.x;}
float get_ned_vel_y(void){return ned_current_vel.y;}
float get_ned_vel_z(void){return ned_current_vel.z;}
float get_odom_x(void){return odom_3d.x;}
float get_odom_y(void){return odom_3d.y;}
float get_odom_z(void){return odom_3d.z;}
float get_yaw_map(void){return yaw_map;}
bool get_mav_yaw_state(void){return get_mav_yaw;}
bool get_gnss_location_state(void){return get_gnss_location;}

float get_mav_x_target(void){return mav_x_target;}
float get_mav_y_target(void){return mav_y_target;}
float get_mav_vx_target(void){return mav_vx_target;}
float get_mav_vy_target(void){return mav_vy_target;}

void reset_dataflash(void){
	dataflash->reset_addr_num_max();
}

void update_dataflash(void){
	uint16_t addr_num_max=dataflash->get_addr_num_max();
	if(addr_num_max>1000){
		dataflash->reset_addr_num_max();
	}
	if(addr_num_max<=0){
		dataflash->set_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
		dataflash->set_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
		dataflash->set_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
		dataflash->set_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
		dataflash->set_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
		dataflash->set_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
		dataflash->set_param_float(param->angle_max.num, param->angle_max.value);
		dataflash->set_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
		dataflash->set_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
		dataflash->set_param_float(param->_spool_up_time.num, param->_spool_up_time.value);
		dataflash->set_param_float(param->throttle_filt.num, param->throttle_filt.value);
		dataflash->set_param_vector3f(param->accel_offsets.num, param->accel_offsets.value);
		dataflash->set_param_vector3f(param->accel_diagonals.num, param->accel_diagonals.value);
		dataflash->set_param_vector3f(param->accel_offdiagonals.num, param->accel_offdiagonals.value);
		dataflash->set_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
		dataflash->set_param_vector3f(param->mag_diagonals.num, param->mag_diagonals.value);
		dataflash->set_param_vector3f(param->mag_offdiagonals.num, param->mag_offdiagonals.value);
		dataflash->set_param_float(param->angle_roll_p.num, param->angle_roll_p.value);
		dataflash->set_param_float(param->angle_pitch_p.num, param->angle_pitch_p.value);
		dataflash->set_param_float(param->angle_yaw_p.num, param->angle_yaw_p.value);
		dataflash->set_param_pid(param->rate_roll_pid.num, param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
					param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz);
		dataflash->set_param_pid(param->rate_pitch_pid.num, param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
					param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz);
		dataflash->set_param_pid(param->rate_yaw_pid.num, param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
					param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz);
		dataflash->set_param_float(param->pos_z_p.num, param->pos_z_p.value);
		dataflash->set_param_float(param->vel_z_p.num, param->vel_z_p.value);
		dataflash->set_param_pid(param->accel_z_pid.num, param->accel_z_pid.value_p, param->accel_z_pid.value_i,
					param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz);
		dataflash->set_param_float(param->pos_xy_p.num, param->pos_xy_p.value);
		dataflash->set_param_pid_2d(param->vel_xy_pid.num, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i,
					param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz,  param->vel_xy_pid.value_filt_d_hz);
		dataflash->set_param_uint8(param->robot_type.num, param->robot_type.value);
		dataflash->set_param_uint8(param->motor_type.num, param->motor_type.value);
		dataflash->set_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
		dataflash->set_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
		dataflash->set_param_float(param->vib_land.num, param->vib_land.value);
		dataflash->set_param_vector3f(param->horizontal_correct.num, param->horizontal_correct.value);
	}else{
		dataflash->get_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
		dataflash->get_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
		dataflash->get_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
		dataflash->get_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
		dataflash->get_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
		dataflash->get_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
		dataflash->get_param_float(param->angle_max.num, param->angle_max.value);
		dataflash->get_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
		dataflash->get_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
		dataflash->get_param_float(param->_spool_up_time.num, param->_spool_up_time.value);
		dataflash->get_param_float(param->throttle_filt.num, param->throttle_filt.value);
		dataflash->get_param_vector3f(param->accel_offsets.num, param->accel_offsets.value);
		dataflash->get_param_vector3f(param->accel_diagonals.num, param->accel_diagonals.value);
		dataflash->get_param_vector3f(param->accel_offdiagonals.num, param->accel_offdiagonals.value);
		dataflash->get_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
		dataflash->get_param_vector3f(param->mag_diagonals.num, param->mag_diagonals.value);
		dataflash->get_param_vector3f(param->mag_offdiagonals.num, param->mag_offdiagonals.value);
		dataflash->get_param_float(param->angle_roll_p.num, param->angle_roll_p.value);
		dataflash->get_param_float(param->angle_pitch_p.num, param->angle_pitch_p.value);
		dataflash->get_param_float(param->angle_yaw_p.num, param->angle_yaw_p.value);
		dataflash->get_param_pid(param->rate_roll_pid.num, param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
					param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz);
		dataflash->get_param_pid(param->rate_pitch_pid.num, param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
					param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz);
		dataflash->get_param_pid(param->rate_yaw_pid.num, param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
					param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz);
		dataflash->get_param_float(param->pos_z_p.num, param->pos_z_p.value);
		dataflash->get_param_float(param->vel_z_p.num, param->vel_z_p.value);
		dataflash->get_param_pid(param->accel_z_pid.num, param->accel_z_pid.value_p, param->accel_z_pid.value_i,
					param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz);
		dataflash->get_param_float(param->pos_xy_p.num, param->pos_xy_p.value);
		dataflash->get_param_pid_2d(param->vel_xy_pid.num, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i,
					param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz, param->vel_xy_pid.value_filt_d_hz);
		dataflash->get_param_uint8(param->robot_type.num, param->robot_type.value);
		dataflash->get_param_uint8(param->motor_type.num, param->motor_type.value);
		dataflash->get_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
		dataflash->get_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
		dataflash->get_param_float(param->vib_land.num, param->vib_land.value);
		dataflash->get_param_vector3f(param->horizontal_correct.num, param->horizontal_correct.value);
	}
}

#define TFMINI_DATA_Len             9
#define TFMINT_DATA_HEAD            0x59
typedef enum{
	TFMINI_IDLE=0,
	TFMINI_HEAD,
	TFMINI_CHK
}TFmini_state;
static TFmini_state tfmini_state=TFMINI_IDLE;
static uint8_t chk_cal = 0, data_num=0;
static uint8_t tfmini_data[6];
static Vector3f tfmini_offset=Vector3f(0.0f,-9.0f, 0.0f);//cm 	X4:8,-8,0; X8:0,-9,0
static uint16_t cordist = 0, strength=0;
void get_tfmini_data(uint8_t buf)
{
	switch(tfmini_state){
		case TFMINI_IDLE:
			if(TFMINT_DATA_HEAD==buf){
				tfmini_state=TFMINI_HEAD;
				chk_cal=TFMINT_DATA_HEAD;
				data_num=0;
			}
			break;
		case TFMINI_HEAD:
			if(TFMINT_DATA_HEAD==buf){
				chk_cal+=TFMINT_DATA_HEAD;
				tfmini_state=TFMINI_CHK;
			}else{
				tfmini_state=TFMINI_IDLE;
			}
			break;
		case TFMINI_CHK:
			if(data_num<6){
				chk_cal+=buf;
				tfmini_data[data_num]=buf;
				data_num++;
			}else if(data_num==6&&chk_cal==buf){
				cordist=tfmini_data[0]|(tfmini_data[1]<<8);//cm
				strength=tfmini_data[2]|(tfmini_data[3]<<8);
				if(cordist>3&&cordist<=1200){
					rangefinder_state.alt_healthy=true;
					Vector3f pos_offset=dcm_matrix*tfmini_offset;
					rangefinder_state.alt_cm=(float)cordist*MAX(0.707f, dcm_matrix.c.z)+pos_offset.z;
					rangefinder_state.last_healthy_ms=HAL_GetTick();
					get_rangefinder_data=true;
				}else{
					rangefinder_state.alt_healthy=false;
				}
				tfmini_state=TFMINI_IDLE;
			}else{
				tfmini_state=TFMINI_IDLE;
			}
			break;
	}
}

//接收
static mavlink_message_t msg_received;
static mavlink_status_t status;
static uint32_t time_last_heartbeat[5]={0};
static uint32_t time_last_attitude=0;
static mavlink_heartbeat_t heartbeat;
static mavlink_set_mode_t setmode;
static mavlink_command_long_t cmd;
static mavlink_rc_channels_override_t rc_channels;
static mavlink_attitude_t attitude_mav;
static mavlink_local_position_ned_cov_t local_position_ned_cov;
static mavlink_set_position_target_local_ned_t set_position_target_local_ned;
static Vector3f lidar_offset=Vector3f(0.0f,0.0f, -16.0f);//cm
static mavlink_channel_t gcs_channel;
//发送
static mavlink_system_t mavlink_system;
static mavlink_message_t msg_attitude_rpy, msg_command_long, msg_battery_status, msg_rc_channels;
static mavlink_attitude_t attitude_rpy;
static mavlink_command_long_t command_long;
static mavlink_battery_status_t battery_status;
static mavlink_rc_channels_t rc_channels_t;

void parse_mavlink_data(mavlink_channel_t chan, uint8_t data, mavlink_message_t* msg_received, mavlink_status_t* status){
	if (mavlink_parse_char(chan, data, msg_received, status)){
		switch (msg_received->msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
				mavlink_msg_heartbeat_decode(msg_received, &heartbeat);
				HeartBeatFlags|=(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan);
				time_last_heartbeat[(uint8_t)chan]=HAL_GetTick();
				if(heartbeat.type==MAV_TYPE_GCS){//地面站
					gcs_channel=chan;
				}
				break;
			case MAVLINK_MSG_ID_SET_MODE:
				mavlink_msg_set_mode_decode(msg_received, &setmode);
				if(setmode.base_mode==MAV_MODE_AUTO_ARMED){
					arm_motors();
					break;
				}else if(setmode.base_mode==MAV_MODE_AUTO_DISARMED){
					disarm_motors();
					break;
				}
				param->robot_type.value=(uint8_t)((setmode.custom_mode>>24)&0xFF);
				param->motor_type.value=(uint8_t)((setmode.custom_mode>>16)&0xFF);
				dataflash->set_param_uint8(param->robot_type.num, param->robot_type.value);
				dataflash->set_param_uint8(param->motor_type.num, param->motor_type.value);
				motors_init();
				break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
				mavlink_msg_command_long_decode(msg_received, &cmd);
				switch(cmd.command){
					case MAV_CMD_NAV_TAKEOFF:
						set_takeoff(true);
						break;
					case MAV_CMD_PREFLIGHT_CALIBRATION:
						if(is_equal(cmd.param1,1.0f)){					//start accel calibrate
							if(is_equal(cmd.param2,0.0f)){
								accel_cal_succeed=false;
								param->accel_offsets.value={0.0f,0.0f,0.0f};
								param->accel_diagonals.value={1.0f,1.0f,1.0f};
								param->accel_offdiagonals.value={0.0f,0.0f,0.0f};
							}else if(is_equal(cmd.param2,1.0f)){		// level
								accelCalibrator->collect_sample();
							}else if(is_equal(cmd.param2,2.0f)){		// on its LEFT side
								accelCalibrator->collect_sample();
							}else if(is_equal(cmd.param2,3.0f)){		// on its RIGHT side
								accelCalibrator->collect_sample();
							}else if(is_equal(cmd.param2,4.0f)){		// nose DOWN
								accelCalibrator->collect_sample();
							}else if(is_equal(cmd.param2,5.0f)){		// nose UP
								accelCalibrator->collect_sample();
							}else if(is_equal(cmd.param2,6.0f)){		// on its BACK
								accelCalibrator->collect_sample();
							}
						}else if(is_equal(cmd.param1,2.0f)){			//start compass calibrate
							compass_cal_succeed=false;
							initial_compass_cal=false;
							mag_correcting=true;
							mag_corrected=false;
							ahrs->reset();
							param->mag_offsets.value={0.0f,0.0f,0.0f};
							param->mag_diagonals.value={1.0f,1.0f,1.0f};
							param->mag_offdiagonals.value={0.0f,0.0f,0.0f};
						}else if(is_equal(cmd.param1,3.0f)){            //校准水平
							horizon_correct=true;
						}
						send_mavlink_commond_ack(chan, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACK_OK);
						break;
					case MAV_CMD_DO_SET_PARAMETER:
						if(is_equal(cmd.param1,0.5f)){//reset PID parameters in flash
							param->angle_roll_p.value=AC_ATTITUDE_CONTROL_ANGLE_ROLL_P;
							param->angle_pitch_p.value=AC_ATTITUDE_CONTROL_ANGLE_PITCH_P;
							param->angle_yaw_p.value=AC_ATTITUDE_CONTROL_ANGLE_YAW_P;
							param->rate_roll_pid.value_p=AC_ATC_MULTI_RATE_ROLL_P;
							param->rate_roll_pid.value_i=AC_ATC_MULTI_RATE_ROLL_I;
							param->rate_roll_pid.value_d=AC_ATC_MULTI_RATE_ROLL_D;
							param->rate_roll_pid.value_imax=AC_ATC_MULTI_RATE_RP_IMAX;
							param->rate_roll_pid.value_filt_hz=AC_ATC_MULTI_RATE_RP_FILT_HZ;
							param->rate_pitch_pid.value_p=AC_ATC_MULTI_RATE_PITCH_P;
							param->rate_pitch_pid.value_i=AC_ATC_MULTI_RATE_PITCH_I;
							param->rate_pitch_pid.value_d=AC_ATC_MULTI_RATE_PITCH_D;
							param->rate_pitch_pid.value_imax=AC_ATC_MULTI_RATE_RP_IMAX;
							param->rate_pitch_pid.value_filt_hz=AC_ATC_MULTI_RATE_RP_FILT_HZ;
							param->rate_yaw_pid.value_p=AC_ATC_MULTI_RATE_YAW_P;
							param->rate_yaw_pid.value_i=AC_ATC_MULTI_RATE_YAW_I;
							param->rate_yaw_pid.value_d=AC_ATC_MULTI_RATE_YAW_D;
							param->rate_yaw_pid.value_imax=AC_ATC_MULTI_RATE_YAW_IMAX;
							param->rate_yaw_pid.value_filt_hz=AC_ATC_MULTI_RATE_YAW_FILT_HZ;
							param->pos_z_p.value=POSCONTROL_POS_Z_P;
							param->vel_z_p.value=POSCONTROL_VEL_Z_P;
							param->accel_z_pid.value_p=POSCONTROL_ACC_Z_P;
							param->accel_z_pid.value_i=POSCONTROL_ACC_Z_I;
							param->accel_z_pid.value_d=POSCONTROL_ACC_Z_D;
							param->accel_z_pid.value_imax=POSCONTROL_ACC_Z_IMAX;
							param->accel_z_pid.value_filt_hz=POSCONTROL_ACC_Z_FILT_HZ;
							param->pos_xy_p.value=POSCONTROL_POS_XY_P;
							param->vel_xy_pid.value_p=POSCONTROL_VEL_XY_P;
							param->vel_xy_pid.value_i=POSCONTROL_VEL_XY_I;
							param->vel_xy_pid.value_d=POSCONTROL_VEL_XY_D;
							param->vel_xy_pid.value_imax=POSCONTROL_VEL_XY_IMAX;
							param->vel_xy_pid.value_filt_hz=POSCONTROL_VEL_XY_FILT_HZ;
							param->vel_xy_pid.value_filt_d_hz=POSCONTROL_VEL_XY_FILT_D_HZ;

							param->acro_y_expo.value=ACRO_YAW_EXPO;
							param->acro_yaw_p.value=ACRO_YAW_P;
							param->throttle_midzone.value=THR_MIDZ_DEFAULT;
							param->pilot_speed_dn.value=PILOT_VELZ_DOWN_MAX;
							param->pilot_speed_up.value=PILOT_VELZ_UP_MAX;
							param->rangefinder_gain.value=RANGEFINDER_GAIN_DEFAULT;
							param->angle_max.value=DEFAULT_ANGLE_MAX;
							param->pilot_accel_z.value=PILOT_ACCEL_Z_DEFAULT;
							param->pilot_takeoff_alt.value=PILOT_TKOFF_ALT_DEFAULT;
							param->_spool_up_time.value=MOTORS_SPOOL_UP_TIME_DEFAULT;
							param->throttle_filt.value=MAN_THR_FILT_HZ;
							param->t_hover_update_min.value=THR_HOVER_UPDATE_MIN;
							param->t_hover_update_max.value=THR_HOVER_UPDATE_MAX;
							param->vib_land.value=VIB_LAND_THR;

							dataflash->set_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
							dataflash->set_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
							dataflash->set_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
							dataflash->set_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
							dataflash->set_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
							dataflash->set_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
							dataflash->set_param_float(param->angle_max.num, param->angle_max.value);
							dataflash->set_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
							dataflash->set_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
							dataflash->set_param_float(param->_spool_up_time.num, param->_spool_up_time.value);
							dataflash->set_param_float(param->throttle_filt.num, param->throttle_filt.value);
							dataflash->set_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
							dataflash->set_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
							dataflash->set_param_float(param->vib_land.num, param->vib_land.value);

							dataflash->set_param_float(param->angle_roll_p.num, param->angle_roll_p.value);
							dataflash->set_param_float(param->angle_pitch_p.num, param->angle_pitch_p.value);
							dataflash->set_param_float(param->angle_yaw_p.num, param->angle_yaw_p.value);
							dataflash->set_param_pid(param->rate_roll_pid.num, param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
										param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz);
							dataflash->set_param_pid(param->rate_pitch_pid.num, param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
										param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz);
							dataflash->set_param_pid(param->rate_yaw_pid.num, param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
										param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz);
							dataflash->set_param_float(param->pos_z_p.num, param->pos_z_p.value);
							dataflash->set_param_float(param->vel_z_p.num, param->vel_z_p.value);
							dataflash->set_param_pid(param->accel_z_pid.num, param->accel_z_pid.value_p, param->accel_z_pid.value_i,
										param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz);
							dataflash->set_param_float(param->pos_xy_p.num, param->pos_xy_p.value);
							dataflash->set_param_pid_2d(param->vel_xy_pid.num, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i,
										param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz,  param->vel_xy_pid.value_filt_d_hz);
							attitude->get_angle_roll_p()(param->angle_roll_p.value);
							attitude->get_angle_pitch_p()(param->angle_pitch_p.value);
							attitude->get_angle_yaw_p()(param->angle_yaw_p.value);
							attitude->get_rate_roll_pid()(param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
									param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz, _dt);
							attitude->get_rate_pitch_pid()(param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
									param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz, _dt);
							attitude->get_rate_yaw_pid()(param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
									param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz, _dt);
							pos_control->get_pos_z_p()(param->pos_z_p.value);
							pos_control->get_vel_z_p()(param->vel_z_p.value);
							pos_control->get_accel_z_pid()(param->accel_z_pid.value_p, param->accel_z_pid.value_i,
									param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz, _dt);
							pos_control->get_pos_xy_p()(param->pos_xy_p.value);
							pos_control->get_vel_xy_pid()(param->vel_xy_pid.value_p, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i, param->vel_xy_pid.value_i, param->vel_xy_pid.value_d,
									param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz, param->vel_xy_pid.value_filt_d_hz, _dt);
							send_mavlink_param_list(chan);
						}else if(is_equal(cmd.param1,1.0f)){//ANGLE_ROLL_P
							attitude->get_angle_roll_p().kP(cmd.param2);
							param->angle_roll_p.value=cmd.param2;
							dataflash->set_param_float(param->angle_roll_p.num, param->angle_roll_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=1.0f;
							command_long.param2=param->angle_roll_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,2.0f)){//ANGLE_PITCH_P
							attitude->get_angle_pitch_p().kP(cmd.param2);
							param->angle_pitch_p.value=cmd.param2;
							dataflash->set_param_float(param->angle_pitch_p.num, param->angle_pitch_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=2.0f;
							command_long.param2=param->angle_pitch_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,3.0f)){//ANGLE_YAW_P
							attitude->get_angle_yaw_p().kP(cmd.param2);
							param->angle_yaw_p.value=cmd.param2;
							dataflash->set_param_float(param->angle_yaw_p.num, param->angle_yaw_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=3.0f;
							command_long.param2=param->angle_yaw_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,4.0f)){//RATE_ROLL_PID
							attitude->get_rate_roll_pid().kP(cmd.param2);
							attitude->get_rate_roll_pid().kI(cmd.param3);
							attitude->get_rate_roll_pid().kD(cmd.param4);
							attitude->get_rate_roll_pid().imax(cmd.param5);
							attitude->get_rate_roll_pid().filt_hz(cmd.param6);
							param->rate_roll_pid.value_p=cmd.param2;
							param->rate_roll_pid.value_i=cmd.param3;
							param->rate_roll_pid.value_d=cmd.param4;
							param->rate_roll_pid.value_imax=cmd.param5;
							param->rate_roll_pid.value_filt_hz=cmd.param6;
							dataflash->set_param_pid(param->rate_roll_pid.num, param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
									param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=4.0f;
							command_long.param2=param->rate_roll_pid.value_p;
							command_long.param3=param->rate_roll_pid.value_i;
							command_long.param4=param->rate_roll_pid.value_d;
							command_long.param5=param->rate_roll_pid.value_imax;
							command_long.param6=param->rate_roll_pid.value_filt_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,5.0f)){//RATE_PITCH_PID
							attitude->get_rate_pitch_pid().kP(cmd.param2);
							attitude->get_rate_pitch_pid().kI(cmd.param3);
							attitude->get_rate_pitch_pid().kD(cmd.param4);
							attitude->get_rate_pitch_pid().imax(cmd.param5);
							attitude->get_rate_pitch_pid().filt_hz(cmd.param6);
							param->rate_pitch_pid.value_p=cmd.param2;
							param->rate_pitch_pid.value_i=cmd.param3;
							param->rate_pitch_pid.value_d=cmd.param4;
							param->rate_pitch_pid.value_imax=cmd.param5;
							param->rate_pitch_pid.value_filt_hz=cmd.param6;
							dataflash->set_param_pid(param->rate_pitch_pid.num, param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
									param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=5.0f;
							command_long.param2=param->rate_pitch_pid.value_p;
							command_long.param3=param->rate_pitch_pid.value_i;
							command_long.param4=param->rate_pitch_pid.value_d;
							command_long.param5=param->rate_pitch_pid.value_imax;
							command_long.param6=param->rate_pitch_pid.value_filt_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,6.0f)){//RATE_YAW_PID
							attitude->get_rate_yaw_pid().kP(cmd.param2);
							attitude->get_rate_yaw_pid().kI(cmd.param3);
							attitude->get_rate_yaw_pid().kD(cmd.param4);
							attitude->get_rate_yaw_pid().imax(cmd.param5);
							attitude->get_rate_yaw_pid().filt_hz(cmd.param6);
							param->rate_yaw_pid.value_p=cmd.param2;
							param->rate_yaw_pid.value_i=cmd.param3;
							param->rate_yaw_pid.value_d=cmd.param4;
							param->rate_yaw_pid.value_imax=cmd.param5;
							param->rate_yaw_pid.value_filt_hz=cmd.param6;
							dataflash->set_param_pid(param->rate_yaw_pid.num, param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
									param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=6.0f;
							command_long.param2=param->rate_yaw_pid.value_p;
							command_long.param3=param->rate_yaw_pid.value_i;
							command_long.param4=param->rate_yaw_pid.value_d;
							command_long.param5=param->rate_yaw_pid.value_imax;
							command_long.param6=param->rate_yaw_pid.value_filt_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,7.0f)){//POS_Z_P
							pos_control->get_pos_z_p().kP(cmd.param2);
							param->pos_z_p.value=cmd.param2;
							dataflash->set_param_float(param->pos_z_p.num, param->pos_z_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=7.0f;
							command_long.param2=param->pos_z_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,8.0f)){//VEL_Z_P
							pos_control->get_vel_z_p().kP(cmd.param2);
							param->vel_z_p.value=cmd.param2;
							dataflash->set_param_float(param->vel_z_p.num, param->vel_z_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=8.0f;
							command_long.param2=param->vel_z_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,9.0f)){//ACCEL_Z_P
							pos_control->get_accel_z_pid().kP(cmd.param2);
							pos_control->get_accel_z_pid().kI(cmd.param3);
							pos_control->get_accel_z_pid().kD(cmd.param4);
							pos_control->get_accel_z_pid().imax(cmd.param5);
							pos_control->get_accel_z_pid().filt_hz(cmd.param6);
							param->accel_z_pid.value_p=cmd.param2;
							param->accel_z_pid.value_i=cmd.param3;
							param->accel_z_pid.value_d=cmd.param4;
							param->accel_z_pid.value_imax=cmd.param5;
							param->accel_z_pid.value_filt_hz=cmd.param6;
							dataflash->set_param_pid(param->accel_z_pid.num, param->accel_z_pid.value_p, param->accel_z_pid.value_i,
									param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=9.0f;
							command_long.param2=param->accel_z_pid.value_p;
							command_long.param3=param->accel_z_pid.value_i;
							command_long.param4=param->accel_z_pid.value_d;
							command_long.param5=param->accel_z_pid.value_imax;
							command_long.param6=param->accel_z_pid.value_filt_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,10.0f)){//POS_XY_P
							pos_control->get_pos_xy_p().kP(cmd.param2);
							param->pos_xy_p.value=cmd.param2;
							dataflash->set_param_float(param->pos_xy_p.num, param->pos_xy_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=10.0f;
							command_long.param2=param->pos_xy_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,11.0f)){//VEL_XY_PID
							pos_control->get_vel_xy_pid().kP(Vector2f(cmd.param2,cmd.param2));
							pos_control->get_vel_xy_pid().kI(Vector2f(cmd.param3,cmd.param3));
							pos_control->get_vel_xy_pid().kD(Vector2f(cmd.param4,cmd.param4));
							pos_control->get_vel_xy_pid().imax(cmd.param5);
							pos_control->get_vel_xy_pid().filt_hz(cmd.param6);
							pos_control->get_vel_xy_pid().filt_d_hz(cmd.param7);
							param->vel_xy_pid.value_p=cmd.param2;
							param->vel_xy_pid.value_i=cmd.param3;
							param->vel_xy_pid.value_d=cmd.param4;
							param->vel_xy_pid.value_imax=cmd.param5;
							param->vel_xy_pid.value_filt_hz=cmd.param6;
							param->vel_xy_pid.value_filt_d_hz=cmd.param7;
							dataflash->set_param_pid_2d(param->vel_xy_pid.num, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i,
									param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz, param->vel_xy_pid.value_filt_d_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=11.0f;
							command_long.param2=param->vel_xy_pid.value_p;
							command_long.param3=param->vel_xy_pid.value_i;
							command_long.param4=param->vel_xy_pid.value_d;
							command_long.param5=param->vel_xy_pid.value_imax;
							command_long.param6=param->vel_xy_pid.value_filt_hz;
							command_long.param7=param->vel_xy_pid.value_filt_d_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,12.0f)){
							param->acro_y_expo.value=cmd.param2;
							dataflash->set_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=12.0f;
							command_long.param2=param->acro_y_expo.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,13.0f)){
							param->acro_yaw_p.value=cmd.param2;
							dataflash->set_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=13.0f;
							command_long.param2=param->acro_yaw_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,14.0f)){
							param->throttle_midzone.value=cmd.param2;
							dataflash->set_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=14.0f;
							command_long.param2=param->throttle_midzone.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,15.0f)){
							param->pilot_speed_dn.value=cmd.param2;
							dataflash->set_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=15.0f;
							command_long.param2=param->pilot_speed_dn.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,16.0f)){
							param->pilot_speed_up.value=cmd.param2;
							dataflash->set_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=16.0f;
							command_long.param2=param->pilot_speed_up.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,17.0f)){
							param->rangefinder_gain.value=cmd.param2;
							dataflash->set_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=17.0f;
							command_long.param2=param->rangefinder_gain.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,18.0f)){
							param->angle_max.value=cmd.param2;
							dataflash->set_param_float(param->angle_max.num, param->angle_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=18.0f;
							command_long.param2=param->angle_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,19.0f)){
							param->pilot_accel_z.value=cmd.param2;
							dataflash->set_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=19.0f;
							command_long.param2=param->pilot_accel_z.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,20.0f)){
							param->pilot_takeoff_alt.value=cmd.param2;
							dataflash->set_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=20.0f;
							command_long.param2=param->pilot_takeoff_alt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,21.0f)){
							param->_spool_up_time.value=cmd.param2;
							dataflash->set_param_float(param->_spool_up_time.num, param->_spool_up_time.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=21.0f;
							command_long.param2=param->_spool_up_time.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,22.0f)){
							param->throttle_filt.value=cmd.param2;
							dataflash->set_param_float(param->throttle_filt.num, param->throttle_filt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=22.0f;
							command_long.param2=param->throttle_filt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,23.0f)){
							param->t_hover_update_min.value=cmd.param2;
							dataflash->set_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=23.0f;
							command_long.param2=param->t_hover_update_min.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,24.0f)){
							param->t_hover_update_max.value=cmd.param2;
							dataflash->set_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=24.0f;
							command_long.param2=param->t_hover_update_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,25.0f)){
							param->vib_land.value=cmd.param2;
							dataflash->set_param_float(param->vib_land.num, param->vib_land.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=25.0f;
							command_long.param2=param->vib_land.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}
						break;
					default:
						break;
				}
				break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				send_mavlink_param_list(chan);
				break;
			case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
				mavlink_msg_rc_channels_override_decode(msg_received, &rc_channels);
				mav_channels_in[0]=rc_channels.chan1_raw;
				mav_channels_in[1]=rc_channels.chan2_raw;
				mav_channels_in[2]=rc_channels.chan3_raw;
				mav_channels_in[3]=rc_channels.chan4_raw;
				mav_channels_in[4]=rc_channels.chan5_raw;
				mav_channels_in[5]=rc_channels.chan6_raw;
				mav_channels_in[6]=rc_channels.chan7_raw;
				mav_channels_in[7]=rc_channels.chan8_raw;
				set_rc_channels_override(true);
				break;
			case MAVLINK_MSG_ID_ATTITUDE:   // MAV ID: 30
				mavlink_msg_attitude_decode(msg_received, &attitude_mav);
				yaw_map=wrap_PI(-attitude_mav.yaw);  //slam 算法的map坐标系z轴向上，这里统一改为z轴向下, 所以需要加负号
				time_last_attitude=HAL_GetTick();
				get_mav_yaw=true;
				break;
			case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:       // MAV ID: 64
				mavlink_msg_local_position_ned_cov_decode(msg_received, &local_position_ned_cov);
				odom_offset=dcm_matrix*lidar_offset;
				odom_3d.x=local_position_ned_cov.x * 100.0f-odom_offset.x;  //cm
				odom_3d.y=-local_position_ned_cov.y * 100.0f-odom_offset.y;  //cm ，slam 算法的map坐标系z轴向上，这里统一改为z轴向下，所以y也要变符号；
				odom_3d.z=local_position_ned_cov.z * 100.0f; //cm, 虽然slam 算法的map坐标系z轴向上，但是EKF做了匹配，需要得到z轴向上的值；
				get_odom_xy=true;
				break;
			case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
				mavlink_msg_set_position_target_local_ned_decode(msg_received, &set_position_target_local_ned);
				mav_x_target=set_position_target_local_ned.x * 100.0f;
				mav_y_target=-set_position_target_local_ned.y * 100.0f;  //slam 算法的map坐标系z轴向上，这里统一改为z轴向下，所以y也要变符号
				mav_vx_target=set_position_target_local_ned.vx * 100.0f;
				mav_vy_target=-set_position_target_local_ned.vy * 100.0f;
				break;
			default:
				break;
		}
	}
}

void get_mavlink_data(mavlink_channel_t chan, uint8_t *buf, uint32_t len)
{
	for(uint32_t i=0;i<len;i++){
//		s2_printf("c:%d\n",buf[i]);
		parse_mavlink_data(chan, buf[i], &msg_received, &status);
	}
}

//系统启动后必须确保心跳函数1s运行一次
void send_mavlink_heartbeat_data(void){
	mavlink_message_t msg_heartbeat;
	mavlink_heartbeat_t heartbeat;
	mavlink_system.sysid=1;
	mavlink_system.compid=MAV_COMP_ID_AUTOPILOT1;
	heartbeat.type=MAV_TYPE_OCTOROTOR;
	heartbeat.autopilot=MAV_AUTOPILOT_GENERIC;
	heartbeat.base_mode=MAV_MODE_FLAG_STABILIZE_ENABLED|MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	heartbeat.custom_mode=(param->robot_type.value<<24)|(param->motor_type.value<<16);
	if(get_soft_armed()){
	  heartbeat.base_mode|=MAV_MODE_FLAG_SAFETY_ARMED;
	}
	mavlink_msg_heartbeat_encode(mavlink_system.sysid, mavlink_system.compid, &msg_heartbeat, &heartbeat);
#if COMM_0==MAV_COMM
	  mavlink_send_buffer(MAVLINK_COMM_0, &msg_heartbeat);
#endif
#if COMM_1==MAV_COMM
	  mavlink_send_buffer(MAVLINK_COMM_1, &msg_heartbeat);
#endif
#if COMM_2==MAV_COMM
	  mavlink_send_buffer(MAVLINK_COMM_2, &msg_heartbeat);
#endif
#if COMM_3==MAV_COMM
	  mavlink_send_buffer(MAVLINK_COMM_3, &msg_heartbeat);
#endif
#if COMM_4==MAV_COMM
	  mavlink_send_buffer(MAVLINK_COMM_4, &msg_heartbeat);
#endif
	if(get_mav_yaw&&((HAL_GetTick()-time_last_attitude)>1000)){
		//外接机载电脑连上又断了
		get_mav_yaw=false;
	}
}

void send_mavlink_commond_ack(mavlink_channel_t chan, MAV_CMD mav_cmd, MAV_CMD_ACK result){
	mavlink_message_t msg_command_ack;
	mavlink_command_ack_t command_ack;
	command_ack.command=mav_cmd;
	command_ack.result=result;
	mavlink_msg_command_ack_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_ack, &command_ack);
	mavlink_send_buffer(chan, &msg_command_ack);
}

void send_mavlink_data(mavlink_channel_t chan)
{
	if((HAL_GetTick()-time_last_heartbeat[(uint8_t)chan])>5000){
		HeartBeatFlags&=(0xFF^(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan));
		set_rc_channels_override(false);
		return;
	}
	uint32_t time=HAL_GetTick();

	//姿态
	attitude_rpy.pitch=ahrs_pitch_rad();
	attitude_rpy.pitchspeed=gyro_filt.y;
	attitude_rpy.roll=ahrs_roll_rad();
	attitude_rpy.rollspeed=gyro_filt.x;
	attitude_rpy.yaw=ahrs_yaw_rad();
	attitude_rpy.yawspeed=gyro_filt.z;
	attitude_rpy.time_boot_ms=time;
	mavlink_msg_attitude_encode(mavlink_system.sysid, mavlink_system.compid, &msg_attitude_rpy, &attitude_rpy);
	mavlink_send_buffer(chan, &msg_attitude_rpy);

	//罗盘校准
	if(mag_correcting){
		command_long.command=MAV_CMD_PREFLIGHT_CALIBRATION;
		command_long.param1=2.0f;
		command_long.param2=completion_percent;
		mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
		mavlink_send_buffer(chan, &msg_command_long);
	}else{
		if(compass_cal_succeed){
			command_long.command=MAV_CMD_PREFLIGHT_CALIBRATION;
			command_long.param1=2.0f;
			command_long.param2=2.0f;
			mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
			mavlink_send_buffer(chan, &msg_command_long);
			compass_cal_succeed=false;
		}
	}

	//电量
	battery_status.type=2;//3s锂电池
	battery_status.temperature=(int16_t)(get_baro_temp()*100);
	battery_status.voltages[0]=(uint16_t)(get_5v_in()*1000);
	battery_status.voltages[1]=(uint16_t)(get_batt_volt()*1000);
	battery_status.current_battery=(int16_t)(get_batt_current()*100);
	mavlink_msg_battery_status_encode(mavlink_system.sysid, mavlink_system.compid, &msg_battery_status, &battery_status);
	mavlink_send_buffer(chan, &msg_battery_status);

	//电脑端地面站需要显示遥控信号
	if(chan==gcs_channel){
		rc_channels_t.chan1_raw=get_channel_roll();
		rc_channels_t.chan2_raw=get_channel_pitch();
		rc_channels_t.chan3_raw=get_channel_throttle();
		rc_channels_t.chan4_raw=get_channel_yaw();
		rc_channels_t.chan5_raw=get_channel_5();
		rc_channels_t.chan6_raw=get_channel_6();
		rc_channels_t.chan7_raw=get_channel_7();
		rc_channels_t.chan8_raw=get_channel_8();
		rc_channels_t.chancount=RC_INPUT_CHANNELS;
		rc_channels_t.rssi=254;
		mavlink_msg_rc_channels_encode(mavlink_system.sysid, mavlink_system.compid, &msg_rc_channels, &rc_channels_t);
		mavlink_send_buffer(chan, &msg_rc_channels);
	}
}

void send_mavlink_param_list(mavlink_channel_t chan)
{
	command_long.command=MAV_CMD_DO_SET_PARAMETER;

	command_long.param1=1.0f;
	command_long.param2=param->angle_roll_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=2.0f;
	command_long.param2=param->angle_pitch_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=3.0f;
	command_long.param2=param->angle_yaw_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=4.0f;
	command_long.param2=param->rate_roll_pid.value_p;
	command_long.param3=param->rate_roll_pid.value_i;
	command_long.param4=param->rate_roll_pid.value_d;
	command_long.param5=param->rate_roll_pid.value_imax;
	command_long.param6=param->rate_roll_pid.value_filt_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=5.0f;
	command_long.param2=param->rate_pitch_pid.value_p;
	command_long.param3=param->rate_pitch_pid.value_i;
	command_long.param4=param->rate_pitch_pid.value_d;
	command_long.param5=param->rate_pitch_pid.value_imax;
	command_long.param6=param->rate_pitch_pid.value_filt_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=6.0f;
	command_long.param2=param->rate_yaw_pid.value_p;
	command_long.param3=param->rate_yaw_pid.value_i;
	command_long.param4=param->rate_yaw_pid.value_d;
	command_long.param5=param->rate_yaw_pid.value_imax;
	command_long.param6=param->rate_yaw_pid.value_filt_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=7.0f;
	command_long.param2=param->pos_z_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=8.0f;
	command_long.param2=param->vel_z_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=9.0f;
	command_long.param2=param->accel_z_pid.value_p;
	command_long.param3=param->accel_z_pid.value_i;
	command_long.param4=param->accel_z_pid.value_d;
	command_long.param5=param->accel_z_pid.value_imax;
	command_long.param6=param->accel_z_pid.value_filt_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=10.0f;
	command_long.param2=param->pos_xy_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=11.0f;
	command_long.param2=param->vel_xy_pid.value_p;
	command_long.param3=param->vel_xy_pid.value_i;
	command_long.param4=param->vel_xy_pid.value_d;
	command_long.param5=param->vel_xy_pid.value_imax;
	command_long.param6=param->vel_xy_pid.value_filt_hz;
	command_long.param7=param->vel_xy_pid.value_filt_d_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=12.0f;
	command_long.param2=param->acro_y_expo.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=13.0f;
	command_long.param2=param->acro_yaw_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=14.0f;
	command_long.param2=param->throttle_midzone.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=15.0f;
	command_long.param2=param->pilot_speed_dn.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=16.0f;
	command_long.param2=param->pilot_speed_up.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=17.0f;
	command_long.param2=param->rangefinder_gain.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=18.0f;
	command_long.param2=param->angle_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=19.0f;
	command_long.param2=param->pilot_accel_z.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=20.0f;
	command_long.param2=param->pilot_takeoff_alt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=21.0f;
	command_long.param2=param->_spool_up_time.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=22.0f;
	command_long.param2=param->throttle_filt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=23.0f;
	command_long.param2=param->t_hover_update_min.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=24.0f;
	command_long.param2=param->t_hover_update_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=25.0f;
	command_long.param2=param->vib_land.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);
}

void ekf_z_reset(void){
	ekf_rangefinder->reset();
	ekf_baro->reset();
	pos_control->set_alt_target_to_current_alt();
}

void ekf_xy_reset(void){
	ekf_odometry->reset();
	ekf_gnss->reset();
	pos_control->set_xy_target(get_pos_x(), get_pos_y());
}

void motors_init(void){
	Motors::motor_frame_class frame_class;
	Motors::motor_frame_type frame_type;
	Motors::motor_pwm_type motor_type;
	switch(param->robot_type.value){
	case UAV_8_H:
		frame_class=Motors::MOTOR_FRAME_OCTAQUAD;
		frame_type=Motors::MOTOR_FRAME_TYPE_H;
		break;
	case UAV_4_X:
		frame_class=Motors::MOTOR_FRAME_QUAD;
		frame_type=Motors::MOTOR_FRAME_TYPE_X;
		break;
	case UGV_4:
		param->motor_type.value=BRUSH;
		break;
	case UGV_2:
		param->motor_type.value=BRUSH;
		break;
	case SPIDER_6:
		param->motor_type.value=SERVO;
		break;
	default://UAV_8_H
		frame_class=Motors::MOTOR_FRAME_OCTAQUAD;
		frame_type=Motors::MOTOR_FRAME_TYPE_H;
		break;
	}
	switch(param->motor_type.value){
	case ESC:
		motor_type=Motors::PWM_TYPE_ESC;
		break;
	case BRUSH:
		motor_type=Motors::PWM_TYPE_BRUSHED;
		break;
	case SERVO:
		motor_type=Motors::PWM_TYPE_SERVO;
		break;
	default:
		motor_type=Motors::PWM_TYPE_ESC;
		break;
	}
	motors->setup_motors(frame_class,frame_type,motor_type);
	// disable output to motors and servos
	set_rcout_enable(false);
	FMU_PWM_Set_Output_Disable();
	motors->set_interlock(false);
	FMU_LED4_Control(true);
	FMU_LED7_Control(false);
}

void attitude_init(void){
	dcm_matrix_correct.from_euler(param->horizontal_correct.value.x, param->horizontal_correct.value.y, param->horizontal_correct.value.z);
	attitude->get_angle_roll_p()(param->angle_roll_p.value);
	attitude->get_angle_pitch_p()(param->angle_pitch_p.value);
	attitude->get_angle_yaw_p()(param->angle_yaw_p.value);
	attitude->get_rate_roll_pid()(param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
			param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz, _dt);
	attitude->get_rate_pitch_pid()(param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
			param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz, _dt);
	attitude->get_rate_yaw_pid()(param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
			param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz, _dt);
}

void pos_init(void){
	pos_control->get_pos_z_p()(param->pos_z_p.value);
	pos_control->get_vel_z_p()(param->vel_z_p.value);
	pos_control->get_accel_z_pid()(param->accel_z_pid.value_p, param->accel_z_pid.value_i,
			param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz, _dt);
	pos_control->get_pos_xy_p()(param->pos_xy_p.value);
	pos_control->get_vel_xy_pid()(param->vel_xy_pid.value_p, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i, param->vel_xy_pid.value_i, param->vel_xy_pid.value_d,
			param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz, param->vel_xy_pid.value_filt_d_hz, _dt);
	pos_control->set_dt(_dt);
	pos_control->init_xy_controller(true);
}

void update_accel_gyro_data(void){

	accel.x = icm20608_data.accf.x;//m/s/s
	accel.y = icm20608_data.accf.y;//m/s/s
	accel.z = icm20608_data.accf.z;//m/s/s

	if(isnan(accel.x) || isinf(accel.x) || isnan(accel.y) || isinf(accel.y) || isnan(accel.z) || isinf(accel.z)){
		return;
	}

	accel.rotate(ROTATION_YAW_270);
//	usb_printf("ax:%f,ay:%f,az:%f\n",accel.x,accel.y,accel.z);

	gyro.x = icm20608_data.gyrof.x;//°/s
	gyro.y = icm20608_data.gyrof.y;//°/s
	gyro.z = icm20608_data.gyrof.z;//°/s

	if(isnan(gyro.x) || isinf(gyro.x) || isnan(gyro.y) || isinf(gyro.y) || isnan(gyro.z) || isinf(gyro.z)){
		return;
	}

	gyro.rotate(ROTATION_YAW_270);
//	usb_printf("gx:%f,gy:%f,gz:%f\n",gyro.x,gyro.y,gyro.z);

	Matrix3f accel_softiron{param->accel_diagonals.value.x,     param->accel_offdiagonals.value.x,  param->accel_offdiagonals.value.y,
							param->accel_offdiagonals.value.x,  param->accel_diagonals.value.y,     param->accel_offdiagonals.value.z,
							param->accel_offdiagonals.value.y,  param->accel_offdiagonals.value.z,  param->accel_diagonals.value.z};
	accel_correct=accel_softiron*(accel-param->accel_offsets.value);

	gyro_correct=gyro-gyro_offset;

	if(!initial_accel_gyro){
		if(accel_correct.z<-8&&accel_correct.length()<11&&accel_correct.length()>9){
			initial_accel_gyro=true;
			accel_filt=Vector3f(0,0,-GRAVITY_MSS);
			gyro_filt=Vector3f(0,0,0);
			_accel_filter.set_cutoff_frequency(400, accel_filt_hz);
			_gyro_filter.set_cutoff_frequency(400, gyro_filt_hz);
			_accel_ef_filter.set_cutoff_frequency(400, accel_ef_filt_hz);
			ahrs_stage_compass=true;
		}
	}else{
		accel_filt=_accel_filter.apply(accel_correct);
		gyro_filt=_gyro_filter.apply(gyro_correct);
	}
}

static Vector3f delta_velocity;
static uint8_t step;
static bool accel_calibrate(void){
	if(accel_cal_succeed){
		return true;
	}
	if(!initial_accel_cal){
		step=0;
		accelCalibrator->clear();
		accelCalibrator->start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 1.0f);
		initial_accel_cal=true;
	}

	switch(accelCalibrator->get_status()){
		case ACCEL_CAL_NOT_STARTED:
			step=0;
			return false;
		case ACCEL_CAL_WAITING_FOR_ORIENTATION:
			if(step!=accelCalibrator->get_num_samples_collected()+1){
				step=accelCalibrator->get_num_samples_collected()+1;
			}else{
				break;
			}
			switch (step) {
				case ACCELCAL_VEHICLE_POS_LEVEL:
					usb_printf("level\n");
					break;
				case ACCELCAL_VEHICLE_POS_LEFT:
					usb_printf("on its LEFT side\n");
					break;
				case ACCELCAL_VEHICLE_POS_RIGHT:
					usb_printf("on its RIGHT side\n");
					break;
				case ACCELCAL_VEHICLE_POS_NOSEDOWN:
					usb_printf("nose DOWN\n");
					break;
				case ACCELCAL_VEHICLE_POS_NOSEUP:
					usb_printf("nose UP\n");
					break;
				case ACCELCAL_VEHICLE_POS_BACK:
					usb_printf("on its BACK\n");
					break;
				default:
					usb_printf("get all samples\n");
					break;
			}
			break;
		case ACCEL_CAL_COLLECTING_SAMPLE:
			delta_velocity=accel*_dt;
			accelCalibrator->new_sample(delta_velocity,_dt);
			break;
		case ACCEL_CAL_SUCCESS:
			step=0;
			accel_cal_succeed=true;
			accelCalibrator->get_calibration(param->accel_offsets.value, param->accel_diagonals.value, param->accel_offdiagonals.value);
			dataflash->set_param_vector3f(param->accel_offsets.num, param->accel_offsets.value);
			dataflash->set_param_vector3f(param->accel_diagonals.num, param->accel_diagonals.value);
			dataflash->set_param_vector3f(param->accel_offdiagonals.num, param->accel_offdiagonals.value);
			usb_printf("accel calibrate succeed!\n");
			break;
		case ACCEL_CAL_FAILED:
			step=0;
			usb_printf("accel calibrate failed!\n");
			initial_accel_cal=false;
			accel_cal_succeed=false;
			break;
	}
	return accel_cal_succeed;
}

bool gyro_calibrate(void){
	if(gyro_cal_succeed){
		return true;
	}
	Vector3f last_average, best_avg;
	Vector3f new_gyro_offset;
	float best_diff;
	bool converged;

	// remove existing gyro offsets
	new_gyro_offset.zero();
	best_diff = -1.0f;
	last_average.zero();
	converged = false;

	// the strategy is to average 50 points over 0.5 seconds, then do it
	// again and see if the 2nd average is within a small margin of
	// the first

	// we try to get a good calibration estimate for up to 30 seconds
	// if the gyros are stable, we should get it in 1 second
	for (int16_t j = 0; j <= 30*2 ; j++) {
		Vector3f gyro_sum, gyro_avg, gyro_diff;
		Vector3f accel_start;
		float diff_norm=0;
		uint8_t i;

		gyro_sum.zero();
		accel_start = accel_filt;
		for (i=0; i<100; i++) {
			gyro_sum += gyro;
			osDelay(5);
		}

		Vector3f accel_diff = accel_filt - accel_start;
		if (accel_diff.length() > 0.2f) {
			// the accelerometers changed during the gyro sum. Skip
			// this sample. This copes with doing gyro cal on a
			// steadily moving platform. The value 0.2 corresponds
			// with around 5 degrees/second of rotation.
			continue;
		}

		gyro_avg = gyro_sum / i;
		gyro_diff = last_average - gyro_avg;
		diff_norm = gyro_diff.length();

		if (best_diff < 0) {
			best_diff = diff_norm;
			best_avg = gyro_avg;
		} else if (gyro_diff.length() < radians(GYRO_INIT_MAX_DIFF_DPS)) {
			// we want the average to be within 0.1 bit, which is 0.04 degrees/s
			last_average = (gyro_avg * 0.5f) + (last_average * 0.5f);
			if (!converged || last_average.length() < new_gyro_offset.length()) {
				new_gyro_offset = last_average;
			}
			if (!converged) {
				converged = true;
			}
		} else if (diff_norm < best_diff) {
			best_diff = diff_norm;
			best_avg = (gyro_avg * 0.5f) + (last_average * 0.5f);
		}
		last_average = gyro_avg;
		// we've kept the user waiting long enough - use the best pair we
		// found so far
		if (!converged) {
			gyro_offset = best_avg;
			// flag calibration as failed for this gyro
			gyro_cal_succeed = false;
		} else {
			gyro_cal_succeed = true;
			gyro_offset = new_gyro_offset;
			break;
		}
	}
	return gyro_cal_succeed;
}

void update_mag_data(void){
	if(!USE_MAG){
		return;
	}

	mag.x = qmc5883_data.magf.x;
	mag.y = qmc5883_data.magf.y;
	mag.z = qmc5883_data.magf.z;

	if(isnan(mag.x) || isinf(mag.x) || isnan(mag.y) || isinf(mag.y) || isnan(mag.z) || isinf(mag.z)){
		return;
	}

	mag.rotate(ROTATION_YAW_270);

	Matrix3f softiron{     param->mag_diagonals.value.x,    param->mag_offdiagonals.value.x,    param->mag_offdiagonals.value.y,
						param->mag_offdiagonals.value.x,       param->mag_diagonals.value.y,    param->mag_offdiagonals.value.z,
						param->mag_offdiagonals.value.y,    param->mag_offdiagonals.value.z,       param->mag_diagonals.value.z};
	mag_correct=softiron*(mag+param->mag_offsets.value);

	if((!is_equal(param->mag_offsets.value.x,0.0f))&&(!is_equal(param->mag_offsets.value.y,0.0f))&&(!is_equal(param->mag_offsets.value.z,0.0f))){
		mag_corrected=true;
	}

	if(!initial_mag){
		mag_filt=mag_correct;
		_mag_filter.set_cutoff_frequency(100, mag_filt_hz);
		initial_mag=true;
	}else{
		mag_filt = _mag_filter.apply(mag_correct);
	}
}

void compass_calibrate(void){
	if(HAL_GetTick()<2500){
		return;
	}
	if(!initial_compass_cal){
		uint8_t compass_idx=1;
		/* if we got the ahrs, we should set _check_orientation true*/
		compassCalibrator->set_orientation(ROTATION_NONE, true, true, true);
		compassCalibrator->start(true, 2, COMPASS_OFFSETS_MAX_DEFAULT, compass_idx);
		initial_compass_cal=true;
	}
	compassCalibrator->new_sample(mag);
	compassCalibrator->update(calibrate_failure);
	completion_percent=compassCalibrator->get_completion_percent()/100.0f;
	if(is_equal(completion_percent,1.0f)){
		compassCalibrator->get_calibration(param->mag_offsets.value, param->mag_diagonals.value, param->mag_offdiagonals.value);
		dataflash->set_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
		dataflash->set_param_vector3f(param->mag_diagonals.num, param->mag_diagonals.value);
		dataflash->set_param_vector3f(param->mag_offdiagonals.num, param->mag_offdiagonals.value);
		compass_cal_succeed=true;
		mag_correcting=false;
		usb_printf("compass calibrate succeed!\n");
	}
}

static float roll_sum=0, pitch_sum=0;
static uint8_t horizon_correct_flag=0;
void ahrs_update(void){
	if((!gyro_calibrate())||(!accel_calibrate())||(!initial_accel_gyro)){
		ahrs_healthy=false;
		return;
	}

	if(horizon_correct){
		horizon_correct_flag++;
		roll_sum+=roll_rad_raw;
		pitch_sum+=pitch_rad_raw;
		if(horizon_correct_flag==100){
			param->horizontal_correct.value.x=-roll_sum/100;
			param->horizontal_correct.value.y=-pitch_sum/100;
			param->horizontal_correct.value.z=0;
			dataflash->set_param_vector3f(param->horizontal_correct.num, param->horizontal_correct.value);
			dcm_matrix_correct.from_euler(param->horizontal_correct.value.x, param->horizontal_correct.value.y, param->horizontal_correct.value.z);
			horizon_correct=false;
			roll_sum=0;
			pitch_sum=0;
			horizon_correct_flag=0;
		}
	}

	if(ahrs_stage_compass){
		ahrs->update(USE_MAG&&mag_corrected, get_mav_yaw);
		//由ahrs的四元数推出欧拉姿态角
		roll_rad_raw = ahrs->quaternion2.get_euler_roll();
		pitch_rad_raw =ahrs->quaternion2.get_euler_pitch();
		yaw_rad_raw = ahrs->quaternion2.get_euler_yaw();

		//由ahrs的四元数推出旋转矩阵用于控制
		ahrs->quaternion2.rotation_matrix(dcm_matrix);
//		dcm_matrix*=dcm_matrix_correct;
		dcm_matrix.normalize();
		attitude->set_rotation_body_to_ned(dcm_matrix);
		gyro_ef=dcm_matrix*gyro_filt;
		accel_ef=dcm_matrix*accel_filt;
		accel_ef=_accel_ef_filter.apply(accel_ef);

		dcm_matrix.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
		roll_deg=roll_rad*RAD_TO_DEG;
		pitch_deg=pitch_rad*RAD_TO_DEG;
		yaw_deg=yaw_rad*RAD_TO_DEG;
		cos_roll=cosf(roll_rad);
		sin_roll=sinf(roll_rad);
		cos_pitch=cosf(pitch_rad);
		sin_pitch=sinf(pitch_rad);
		cos_yaw=cosf(yaw_rad);
		sin_yaw=sinf(yaw_rad);
		ahrs_healthy=true;
	}
	if(mag_correcting&&initial_mag){
		compass_calibrate();
		ahrs_healthy=false;
	}
}

static float baro_alt_filt=0,baro_alt_init=0;
static uint16_t init_baro=0;
void update_baro_alt(void){
	if(init_baro<200){//前200点不要
		init_baro++;
		return;
	}
	float baro_alt=spl06_data.baro_alt*100.0f;
	if(isnan(baro_alt) || isinf(baro_alt)){
		return;
	}
	if(fabs(baro_alt)>800000 || is_equal(baro_alt, 0.0f)){
		return;
	}
	if(init_baro<300){
		baro_alt_init+=baro_alt/100;
		init_baro++;
		return;
	}
	if(!initial_baro){
		_baro_alt_filter.set_cutoff_frequency(100, baro_filt_hz);
		initial_baro=true;
	}else{
		baro_alt-=baro_alt_init;
		if(baro_alt-baro_alt_filt>200){//过滤掉奇异值
			return;
		}
		baro_alt_filt = _baro_alt_filter.apply(baro_alt);
		get_baro_alt_filt=true;
	}
}

float get_baroalt_filt(void){//cm
	return baro_alt_filt;
}

float get_baro_temp(void){//℃
	return spl06_data.temp;
}

void ekf_baro_alt(void){
	if((!ahrs->is_initialed())||(!initial_baro)||(!ahrs_healthy)){
		return;
	}
	ekf_baro->update(get_baro_alt_filt, get_baroalt_filt());
}

void ekf_rf_alt(void){
	if((!ahrs->is_initialed())||(!rangefinder_state.alt_healthy)||(!ahrs_healthy)){
		return;
	}
	ekf_rangefinder->update(get_rangefinder_data, get_rangefinder_alt());
}

void gnss_update(void){
	if(get_gps_state()){
		if(!initial_gnss){
			gnss_origin_pos.lat=gps_position->lat;//纬度:deg*1e-7
			gnss_origin_pos.lng=gps_position->lon;//经度:deg*1e-7
			gnss_origin_pos.alt=gps_position->alt/10;//海拔：cm
			initial_gnss=true;
		}
		get_gnss_location=true;
		gnss_current_pos.lat=gps_position->lat;//纬度:deg*1e-7
		gnss_current_pos.lng=gps_position->lon;//经度:deg*1e-7
		gnss_current_pos.alt=gps_position->alt/10;   //cm
		ned_current_pos=location_3d_diff_NED(gnss_origin_pos, gnss_current_pos)*100;//cm
		ned_current_vel.x=gps_position->vel_n_m_s*100;//cm
		ned_current_vel.y=gps_position->vel_e_m_s*100;//cm
		ned_current_vel.z=gps_position->vel_d_m_s*100;//cm
	}else{
		initial_gnss=false;
	}
}

//call at 50HZ
bool uwb_pos_filt=false;
static Vector3f uwb_tag_offset=Vector3f(-4.0f,0.0f, -3.0f);//cm
void uwb_position_update(void){
	write_gpio2(true);
	FMU_LED6_Control(get_uwb_position);
	if(uwb_position.x==0&&uwb_position.y==0){
		return;
	}
	if(!uwb_pos_filt){
		uwb_pos_filt=true;
		_uwb_pos_filter.set_cutoff_frequency(50, uwb_pos_filt_hz);
	}
	if(get_uwb_position){
		get_uwb_position=false;
		float theta=1.65;
		odom_offset=dcm_matrix*uwb_tag_offset;
		odom_3d.x=uwb_position.x*cosf(theta)+uwb_position.y*sinf(theta);
		odom_3d.y=-uwb_position.x*sinf(theta)+uwb_position.y*cosf(theta);
		odom_3d.z=uwb_position.z;
		odom_3d = _uwb_pos_filter.apply(odom_3d);
		if(odom_3d.z>30){
			rangefinder_state.alt_healthy=true;
			rangefinder_state.alt_cm=odom_3d.z;
			rangefinder_state.last_healthy_ms=HAL_GetTick();
		}else{
			rangefinder_state.alt_healthy=false;
		}
		get_odom_xy=true;
	}
}

void ekf_odom_xy(void){
	if(!ahrs->is_initialed()||(!ahrs_healthy)){
		return;
	}
	if(odom_3d.x==0&&odom_3d.y==0){
		return;
	}
	ekf_odometry->update(get_odom_xy,odom_3d.x,odom_3d.y);
}

void ekf_gnss_xy(void){
	if(!ahrs->is_initialed()||(!ahrs_healthy)){
		return;
	}
	ekf_gnss->update(get_gnss_location,get_ned_pos_x(),get_ned_pos_y(),get_ned_vel_x(),get_ned_vel_y());
}

float get_pos_x(void){//cm
	return ekf_odometry->pos_x;
}

float get_pos_y(void){//cm
	return ekf_odometry->pos_y;
}

float get_pos_z(void){//cm
	return ekf_baro->pos_z;
}

float get_vel_x(void){//cm/s
	return ekf_odometry->vel_x;
}

float get_vel_y(void){//cm/s
	return ekf_odometry->vel_y;
}

float get_vel_z(void){//cm/s
	return ekf_baro->vel_z;
}

// get_pilot_desired_heading - transform pilot's yaw input into a
// desired yaw rate
// returns desired yaw rate in degrees per second
float get_pilot_desired_yaw_rate(float stick_angle)
{
    float yaw_request;
    float deadband_top = ROLL_PITCH_YAW_INPUT_MAX/10;
    float deadband_bottom = -ROLL_PITCH_YAW_INPUT_MAX/10;

    if(stick_angle<deadband_bottom){
    	stick_angle=-(stick_angle-deadband_bottom)/(-ROLL_PITCH_YAW_INPUT_MAX-deadband_bottom)*ROLL_PITCH_YAW_INPUT_MAX;
    }else if(stick_angle<=deadband_top){
    	stick_angle=0;
    }else{
    	stick_angle=(stick_angle-deadband_top)/(ROLL_PITCH_YAW_INPUT_MAX-deadband_top)*ROLL_PITCH_YAW_INPUT_MAX;
    }

    // calculate yaw rate request
    if (param->acro_y_expo.value <= 0) {
        yaw_request = stick_angle * param->acro_yaw_p.value;
    } else {
        // expo variables
        float y_in, y_in3, y_out;

        // range check expo
        if (param->acro_y_expo.value > 1.0f || param->acro_y_expo.value < 0.5f) {
        	param->acro_y_expo.value = 1.0f;
        }

        // yaw expo
        y_in = stick_angle/ROLL_PITCH_YAW_INPUT_MAX;
        y_in3 = y_in*y_in*y_in;
        y_out = (param->acro_y_expo.value * y_in3) + ((1.0f - param->acro_y_expo.value) * y_in);
        yaw_request = ROLL_PITCH_YAW_INPUT_MAX * y_out * param->acro_yaw_p.value;
    }
    // convert pilot input to the desired yaw rate
    return yaw_request;
}

/*************************************************************
 *  throttle control
 *************************************************************/
static bool _manual_throttle=true;
void set_manual_throttle(bool manual_throttle){_manual_throttle=manual_throttle;}

bool has_manual_throttle(void) { return _manual_throttle; }

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void update_throttle_hover(void)
{
    // if not armed or landed exit
    if (!motors->get_armed() || ap->land_complete) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (has_manual_throttle()) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();
    float climb_rate=get_vel_z();
    // calc average throttle if we are in a level hover
    if (throttle > param->t_hover_update_min.value && throttle < param->t_hover_update_max.value && abs(climb_rate) < 60.0f && abs(roll_deg) < 5.0f && abs(pitch_deg) < 5.0f) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
    }
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
void set_throttle_takeoff(void)
{
    // tell position controller to reset alt target and reset I terms
    pos_control->init_takeoff();
}

float get_throttle_mid(void){
	float throttle_mid=(motors->get_throttle_max()+motors->get_throttle_min())/2;
	return throttle_mid;
}

// transform pilot's manual throttle input to make hover throttle mid stick
// used only for manual throttle modes
// thr_mid should be in the range 0 to 1, e.g. we should use the _throttle_hover
// returns throttle output 0 to 1
float get_pilot_desired_throttle(float throttle_control, float thr_mid)
{
    if (thr_mid <= 0.0f) {
        thr_mid = motors->get_throttle_hover();
    }

    float mid_stick = get_throttle_mid();
    // protect against unlikely divide by zero
    if (mid_stick <= 0) {
        mid_stick = 0.5;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_float(throttle_control,0.0f,1.0f);

    // calculate normalised throttle input
    float throttle_in;
    if (throttle_control < mid_stick) {
        // below the deadband
        throttle_in = throttle_control*0.5f/(float)mid_stick;
    }else if(throttle_control > mid_stick) {
        // above the deadband
        throttle_in = 0.5f + (throttle_control-mid_stick) * 0.5f / (float)(1.0-mid_stick);
    }else{
        // must be in the deadband
        throttle_in = 0.5f;
    }

    float expo = constrain_float(-(thr_mid-0.5)/0.375, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float get_pilot_desired_climb_rate(float throttle_control)
{
    float desired_rate = 0.0f;
    float mid_stick = get_throttle_mid();
    float deadband_top = mid_stick + param->throttle_midzone.value;
    float deadband_bottom = mid_stick - param->throttle_midzone.value;

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1.0f);

    // ensure a reasonable deadzone
    param->throttle_midzone.value = constrain_float(param->throttle_midzone.value, 0, 0.3);

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = param->pilot_speed_dn.value * (throttle_control-deadband_bottom) / deadband_bottom;
    }else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = param->pilot_speed_up.value * (throttle_control-deadband_top) / (1.0f-deadband_top);
    }else{
        // must be in the deadband
        desired_rate = 0.0f;
    }

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float get_non_takeoff_throttle(void)
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

// get_surface_tracking_climb_rate - hold copter at the desired distance above the ground
//      returns climb rate (in cm/s) which should be passed to the position controller
// if use this function, we should set rangefinder_state.alt_healthy=true;
static float target_rangefinder_alt=0.0f;   // desired altitude in cm above the ground
float get_surface_tracking_climb_rate(float target_rate, float current_alt_target, float dt)
{
	if(!rangefinder_state.alt_healthy){
		  // if don't use rangefinder or rangefinder is not healthy, do not use surface tracking
		  return target_rate;
	}

	if((HAL_GetTick()-rangefinder_state.last_healthy_ms)>2000){//rangefinder超过两秒没有更新，认为它不健康
		rangefinder_state.alt_healthy=false;
		return target_rate;
	}

    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;
    float current_alt = get_pos_z();

    uint32_t now = HAL_GetTick();

    // reset target altitude if this controller has just been engaged
    if (now - last_call_ms > RANGEFINDER_TIMEOUT_MS) {
        target_rangefinder_alt = rangefinder_state.alt_cm + current_alt_target - current_alt;
    }
    last_call_ms = now;

    // adjust rangefinder target alt if motors have not hit their limits
    if ((target_rate<0 && !motors->limit.throttle_lower) || (target_rate>0 && !motors->limit.throttle_upper)) {
        target_rangefinder_alt += target_rate * dt;
    }

    /*
      handle rangefinder glitches. When we get a rangefinder reading
      more than RANGEFINDER_GLITCH_ALT_CM different from the current
      rangefinder reading then we consider it a glitch and reject
      until we get RANGEFINDER_GLITCH_NUM_SAMPLES samples in a
      row. When that happens we reset the target altitude to the new
      reading
     */
    float glitch_cm = rangefinder_state.alt_cm - target_rangefinder_alt;
    if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
        rangefinder_state.glitch_count = MAX(rangefinder_state.glitch_count+1,1);
    } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
        rangefinder_state.glitch_count = MIN(rangefinder_state.glitch_count-1,-1);
    } else {
        rangefinder_state.glitch_count = 0;
    }
    if (abs(rangefinder_state.glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
        // shift to the new rangefinder reading
        target_rangefinder_alt = rangefinder_state.alt_cm;
        rangefinder_state.glitch_count = 0;
    }
    if (rangefinder_state.glitch_count != 0) {
        // we are currently glitching, just use the target rate
        return target_rate;
    }

    // calc desired velocity correction from target rangefinder alt vs actual rangefinder alt (remove the error already passed to Altitude controller to avoid oscillations)
    distance_error = (target_rangefinder_alt - rangefinder_state.alt_cm) - (current_alt_target - current_alt);
    velocity_correction = distance_error * param->rangefinder_gain.value;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // return combined pilot climb rate + rate to correct rangefinder alt error
    return (target_rate + velocity_correction);
}

float get_rangefinder_alt(void)
{
	return rangefinder_state.alt_cm;
}

float get_rangefinder_alt_target(void)
{
	return target_rangefinder_alt;
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude->get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->get_accel_z_pid().set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in degrees
void get_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit)
{
    // fetch roll and pitch inputs
    roll_out = get_channel_roll_angle();
    pitch_out = get_channel_pitch_angle();

    // limit max lean angle
    angle_limit = constrain_float(angle_limit, 10.0f, angle_max);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // do circular limit
    float total_in = norm(pitch_out, roll_out);
    if (total_in > angle_limit) {
        float ratio = angle_limit / total_in;
        roll_out *= ratio;
        pitch_out *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_out = (180/M_PI) * atanf(cosf(pitch_out*(M_PI/180))*tanf(roll_out*(M_PI/180)));

    // roll_out and pitch_out are returned
}

/******************take off functions start*********************/
static bool _takeoff=false;
static bool _takeoff_running=false;
static float _takeoff_max_speed=0;
static float _takeoff_start_ms=0;
static float _takeoff_alt_delta=0;

void set_takeoff(bool set){
	_takeoff=set;
}

bool get_takeoff(void){
	if (!ap->land_complete) {
		// can't take off if we're already flying
		return false;
	}
	return _takeoff;
}

bool takeoff_running(void) { return _takeoff_running; }

// start takeoff to specified altitude above home in centimeters
void takeoff_start(float alt_cm)
{
    // calculate climb rate
    const float speed = MAX(param->pilot_speed_up.value*2.0f/3.0f, param->pilot_speed_up.value-50.0f);

    // sanity check speed and target
    if (takeoff_running() || speed <= 0.0f || alt_cm <= 0.0f) {
        return;
    }

    // initialise takeoff state
    _takeoff=false;
    _takeoff_running = true;
    _takeoff_max_speed = speed;
    _takeoff_start_ms = HAL_GetTick();
    _takeoff_alt_delta = alt_cm;
}

bool takeoff_triggered( float target_climb_rate)
{
    if (!ap->land_complete) {
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate <= 0.0f) {
        // can't takeoff unless we want to go up...
        return false;
    }

    return true;
}

void takeoff_stop()
{
	_takeoff=false;
	_takeoff_running = false;
	_takeoff_start_ms = 0;
}

// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void get_takeoff_climb_rates(float& pilot_climb_rate,  float& takeoff_climb_rate)
{
    // return pilot_climb_rate if take-off inactive
    if (!_takeoff_running) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // acceleration of 100cm/s/s
    static const float TAKEOFF_ACCEL = 100.0f;
    const float takeoff_minspeed = MIN(50.0f, _takeoff_max_speed);
    const float time_elapsed = (HAL_GetTick() - _takeoff_start_ms) * 1.0e-3f;
    const float speed = MIN(time_elapsed * TAKEOFF_ACCEL + takeoff_minspeed, _takeoff_max_speed);

    const float time_to_max_speed = (_takeoff_max_speed - takeoff_minspeed) / TAKEOFF_ACCEL;
    float height_gained;
    if (time_elapsed <= time_to_max_speed) {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_elapsed) + takeoff_minspeed * time_elapsed;
    } else {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_to_max_speed) + takeoff_minspeed * time_to_max_speed +(time_elapsed - time_to_max_speed) * _takeoff_max_speed;
    }

    // check if the takeoff is over
    if (height_gained >= _takeoff_alt_delta) {
    	takeoff_stop();
    }

    // if takeoff climb rate is zero return
    if (speed <= 0.0f) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // default take-off climb rate to maximum speed
    takeoff_climb_rate = speed;

    // if pilot's commands descent
    if (pilot_climb_rate < 0.0f) {
        // if overall climb rate is still positive, move to take-off climb rate
        if (takeoff_climb_rate + pilot_climb_rate > 0.0f) {
            takeoff_climb_rate = takeoff_climb_rate + pilot_climb_rate;
            pilot_climb_rate = 0.0f;
        } else {
            // if overall is negative, move to pilot climb rate
            pilot_climb_rate = pilot_climb_rate + takeoff_climb_rate;
            takeoff_climb_rate = 0.0f;
        }
    } else { // pilot commands climb
        // pilot climb rate is zero until it surpasses the take-off climb rate
        if (pilot_climb_rate > takeoff_climb_rate) {
            pilot_climb_rate = pilot_climb_rate - takeoff_climb_rate;
        } else {
            pilot_climb_rate = 0.0f;
        }
    }
}
/******************take off functions end*********************/

// set land_complete flag
void set_land_complete(bool b)
{
    ap->land_complete = b;
}

void rate_controller_run(void){
	attitude->rate_controller_run();
}

// arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool arm_motors(void)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (motors->get_armed()) {
        in_arm_motors = false;
        return true;
    }

    // finally actually arm the motors
    motors->set_armed(true);

    set_soft_armed(true);

    sdlog->Logger_Enable();

    // flag exiting this function
    in_arm_motors = false;

    Buzzer_set_ring_type(BUZZER_ARMED);
    FMU_LED3_Control(true);

    // return success
    return true;
}

// init_disarm_motors - disarm motors
void disarm_motors(void)
{
    // return immediately if we are already disarmed
    if (!motors->get_armed()) {
        return;
    }

    // we are not in the air
    set_land_complete(true);

    // send disarm command to motors
    motors->set_armed(false);

    set_soft_armed(false);

    sdlog->Logger_Disable();

    Buzzer_set_ring_type(BUZZER_DISARM);
    FMU_LED3_Control(false);
}

//解锁电机
void unlock_motors(void){
	if (motors->get_interlock()) {
		return;
	}
	//TODO: add other pre-arm check
	if (is_equal(get_pos_z(),0.0f)){
		return;//高程计没读出数据，禁止电机启动
	}
	// enable output to motors and servos
	set_rcout_enable(true);
	FMU_PWM_Set_Output_Enable();
	motors->set_interlock(true);
	FMU_LED4_Control(false);
	FMU_LED7_Control(true);
}

//锁定电机
void lock_motors(void){
	if (!motors->get_interlock()) {
		return;
	}
	// disable output to motors and servos
	set_rcout_enable(false);
	FMU_PWM_Set_Output_Disable();
	motors->set_interlock(false);
	FMU_LED4_Control(true);
	FMU_LED7_Control(false);
}

void motors_output(void){
	motors->output();
}

// counter to verify landings
static uint32_t land_detector_count = 0;
// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at 100hz
void update_land_detector(void)
{
	ahrs->check_vibration();
	//******************落地前********************
	float deadband_bottom = get_throttle_mid() - param->throttle_midzone.value;
	if((get_channel_throttle()<deadband_bottom)&&(get_vib_value()>param->vib_land.value)&&(motors->get_throttle()<motors->get_throttle_hover())&&(!motors->limit.throttle_lower)){//TODO:降落时防止弹起来
		disarm_motors();
	}
	//******************落地后ls*********************

    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

    if (!motors->get_armed()) {
        // if disarmed, always landed.
        set_land_complete(true);
    } else if (ap->land_complete) {
        // if throttle output is high then clear landing flag
        if (motors->get_throttle() > get_non_takeoff_throttle()) {
            set_land_complete(false);
        }
    } else {

        // check that the average throttle output is near minimum (less than 12.5% hover throttle)
        bool motor_at_lower_limit = motors->limit.throttle_lower && attitude->is_throttle_mix_min();

        // check that vertical speed is within 1m/s of zero
        bool descent_rate_low = fabsf(get_vel_z()) < 100;

        // TODO : if we have a healthy rangefinder only allow landing detection below 2 meters
//        bool rangefinder_check = (!rangefinder_alt_ok() || rangefinder_state.alt_cm_filt.get() < LAND_RANGEFINDER_MIN_ALT_CM);

        if (motor_at_lower_limit && descent_rate_low) {
            // landed criteria met - increment the counter and check if we've triggered
            if( land_detector_count < ((float)LAND_DETECTOR_TRIGGER_SEC)/0.01) {
                land_detector_count++;
            } else {
                set_land_complete(true);
            }
        } else {
            // we've sensed movement up or down so reset land_detector
            land_detector_count = 0;
        }
    }
}

// update_throttle_thr_mix - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
void update_throttle_thr_mix()
{
    // if disarmed or landed prioritise throttle
    if(!motors->get_armed() || ap->land_complete) {
        attitude->set_throttle_mix_min();
        return;
    }

    if (has_manual_throttle()) {
        // manual throttle
        if(get_channel_throttle() <= 0) {
            attitude->set_throttle_mix_min();
        } else {
            attitude->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude->get_att_target_euler_d();
        bool large_angle_request = (norm(angle_target.x, angle_target.y) > LAND_CHECK_LARGE_ANGLE_DEG);

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        Vector3f accel_ef = get_accel_ef();
        accel_ef.z += GRAVITY_MSS;
        bool accel_moving = (accel_ef.length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested decent
        bool descent_not_demanded = pos_control->get_desired_velocity().z >= 0.0f;

        if ( large_angle_request || large_angle_error || accel_moving || descent_not_demanded) {
            attitude->set_throttle_mix_max();
        } else {
            attitude->set_throttle_mix_min();
        }
    }
}

// throttle_loop - should be run at 100 hz
void throttle_loop(void){
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
	update_throttle_thr_mix();
}

//手势开关电机；
//油门0，偏航最大时启动电机；
//油门0，偏航最小时关闭电机。
//注意不是手动油门时，只有在降落状态时才能用手势关闭电机。
void arm_motors_check(void){
	static int16_t arming_counter;
	float throttle=get_channel_throttle();
	// ensure throttle is down
	if (throttle > 0) {
		arming_counter = 0;
		return;
	}

	float tmp = get_channel_yaw();

	// full right
	if (tmp > 0.9) {

		// increase the arming counter to a maximum of 1 beyond the auto trim counter
		if( arming_counter < 250 ) {
			arming_counter++;
		}

		// arm the motors and configure for flight
		if (arming_counter == 250 && !motors->get_armed()) {
			if(!arm_motors()){
				arming_counter=0;
			}
		}

	// full left
	}else if (tmp < -0.9) {
		if (!has_manual_throttle() && !ap->land_complete) {
			arming_counter = 0;
			return;
		}

		// increase the counter to a maximum of 1 beyond the disarm delay
		if( arming_counter < 250 ) {
			arming_counter++;
		}

		// disarm the motors
		if (arming_counter == 250 && motors->get_armed()) {
			disarm_motors();
		}

	// Yaw is centered so reset arming counter
	}else{
		arming_counter = 0;
	}
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control
// throttle_zero is used to determine if the pilot intends to shut down the motors
// Basically, this signals when we are not flying.  We are either on the ground
// or the pilot has shut down the copter in the air and it is free-falling
void set_throttle_zero_flag(float throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = HAL_GetTick();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running,
    // and we are flying. Immediately set as non-zero
    if (throttle_control > 0 && motors->get_interlock()) {
        last_nonzero_throttle_ms = tnow_ms;
        ap->throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms <= THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap->throttle_zero = false;
    } else {
    	ap->throttle_zero = true;
    }
}

//channel8 < 0.1时解锁电机, channel8 > 0.9时锁定电机
static uint8_t disarm_counter=0;
void lock_motors_check(void){
	set_throttle_zero_flag(get_channel_throttle());
	float ch8=get_channel_8();
	if(ch8>0&&ch8<0.1){
		disarm_counter=0;
		unlock_motors();
	}else if(ch8>0.9&&ch8<1.0&&disarm_counter<=10){
		if(disarm_counter==10){
			disarm_motors();
			lock_motors();
		}else{
			disarm_counter++;
		}
	}else{
		return;
	}
}

void zero_throttle_and_relax_ac(void)
{
    motors->set_desired_spool_state(Motors::DESIRED_SPIN_WHEN_ARMED);
    // multicopters do not stabilize roll/pitch/yaw when disarmed
    attitude->set_throttle_out_unstabilized(0.0f, true, param->throttle_filt.value);
}

/********回调函数：获取comm口接收的数据********
 * ***********************************
 * comm0 为usb口，comm1~comm4 为串口1~4
 * 只需配置 Clibrary/config.h 文件中的 COMM_0~COMM_4 为 DEV_COMM 即可启用下面回调函数。
 * 例如：
 * #define COMM_0 DEV_COMM 表示comm0被配置为自定义模式，
 * 此时 parse_comm0_data() 被启用
 * 注意：
 * 当comm口被配置为自定义模式后，接收到的数据会不经过任何处理直接传入下列回调函数，即comm口的其它数据处理功能会被禁用，
 * 若要启用其它数据处理功能，应把comm口重新配置为相应参数
 * ***********************************
 * ***********************************/
void parse_comm0_data(uint8_t data){
	//获取usb接收到的数据
}

void parse_comm1_data(uint8_t data){
	//获取串口S1接收到的数据
}

void parse_comm2_data(uint8_t data){
	//获取串口S2接收到的数据
}

void parse_comm3_data(uint8_t data){
	//获取串口S3接收到的数据
}

void parse_comm4_data(uint8_t data){
	//获取串口S4接收到的数据
}

/********回调函数：在SD卡中写入日志数据名称******
 * ***********************************
 * 注意Logger_Write()函数最多可以一次记录128个字符的字符串
 * 每个名称后面需要加入空格字符，像这样 "%8s "
 * ***********************************
 * ***********************************/
void Logger_Cat_Callback(void){
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s ",//LOG_SENSOR
			"t_ms", "accx", "accy", "accz", "gyrox", "gyroy", "gyroz");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s ",//LOG_SENSOR
			"magx", "magy", "magz", "baro", "voltage", "current");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s ",//LOG_EULER
			"pitchr", "rollr", "yawr", "pitchd", "rolld", "yawd");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s ",//LOG_ACCEL_EARTH_FRAME and VIB
			"efx", "efy", "efz", "vib_vl", "vib_ag");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s ",//LOG_POS_Z
			"barofilt", "alt_t", "pos_z", "vel_z_t", "vel_z", "rf_alt", "rf_alt_t");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s ",//LOG_POS_XY
			"odom_x", "pos_x", "vel_x", "odom_y", "pos_y", "vel_y");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s ",//LOG_VEL_PID_XY
			"v_p_x", "v_i_x", "v_d_x", "v_p_y", "v_i_y", "v_d_y");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_RCIN
			"roll", "pitch", "yaw", "thr", "ch5", "ch6", "ch7", "ch8");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_RCOUT
			"motor1", "motor2", "motor3", "motor4", "motor5", "motor6", "motor7", "motor8");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s ",//LOG_TARGET
			"pos_x_t", "pos_y_t", "vel_x_t", "vel_y_t", "acc_x_t", "acc_y_t", "acc_z_t");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s ",//LOG_CONTROL
			"roll_t", "pitch_t", "p_out", "r_out", "y_out", "t_out", "t_hover");
	osDelay(1);
	//add other loggers
}

/********回调函数：在SD卡中写入日志数据数值********
 * *************************************
 * 注意Logger_Write()函数最多可以一次记录128个字符的字符串
 * 每个数据后面需要加入空格字符，像这样 "%8.3f "
 * ***********************************
 * ***********************************/
void Logger_Data_Callback(void){
	sdlog->Logger_Write("%8ld %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_SENSOR
			HAL_GetTick(), get_accel_correct().x, get_accel_correct().y, get_accel_correct().z,	get_gyro_correct().x, get_gyro_correct().y, get_gyro_correct().z);
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_SENSOR
			get_mag_correct().x, get_mag_correct().y, get_mag_correct().z, spl06_data.baro_alt, get_batt_volt(), get_batt_current());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_EULER
			ahrs_pitch_rad(), ahrs_roll_rad(), ahrs_yaw_rad(), ahrs_pitch_deg(), ahrs_roll_deg(), ahrs_yaw_deg());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_ACCEL_EARTH_FRAME and VIB
			get_accel_ef().x, get_accel_ef().y, get_accel_ef().z, get_vib_value(), get_vib_angle_z());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_POS_Z
			get_baroalt_filt(), pos_control->get_pos_target().z, get_pos_z(), pos_control->get_vel_target_z(), get_vel_z(), get_rangefinder_alt(), get_rangefinder_alt_target());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_POS_XY
			uwb_position.x, get_pos_x(), get_vel_x(), uwb_position.y, get_pos_y(), get_vel_y());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_VEL_PID_XY
			pos_control->get_vel_xy_pid().get_p().x, pos_control->get_vel_xy_pid().get_i().x, pos_control->get_vel_xy_pid().get_d().x, pos_control->get_vel_xy_pid().get_p().y, pos_control->get_vel_xy_pid().get_i().y, pos_control->get_vel_xy_pid().get_d().y);
	osDelay(1);
	sdlog->Logger_Write("%8d %8d %8d %8d %8d %8d %8d %8d ",//LOG_RCIN
			input_channel_roll(), input_channel_pitch(), input_channel_yaw(), input_channel_throttle(), input_channel_5(), input_channel_6(), input_channel_7(), input_channel_8());
	osDelay(1);
	sdlog->Logger_Write("%8d %8d %8d %8d %8d %8d %8d %8d ",//LOG_RCOUT
			pwm_channel.motor[0], pwm_channel.motor[1], pwm_channel.motor[2], pwm_channel.motor[3], pwm_channel.motor[4], pwm_channel.motor[5], pwm_channel.motor[6], pwm_channel.motor[7]);
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_TARGET
			pos_control->get_pos_target().x, pos_control->get_pos_target().y, pos_control->get_vel_target().x, pos_control->get_vel_target().y,	pos_control->get_accel_target().x, pos_control->get_accel_target().y, pos_control->get_accel_target().z);
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_CONTROL
			pos_control->get_roll(), pos_control->get_pitch(), motors->get_pitch(), motors->get_roll(), motors->get_yaw(), motors->get_throttle(), motors->get_throttle_hover());
	osDelay(1);
	//add other loggers
}

void Logger_Update(void){
	sdlog->Logger_Update();
}

/*****************************************************************
 * *******************code for test and debug*********************
 *****************************************************************/
void debug(void){
//	usb_printf("hello, Mcontroller\n");
}
