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

/* *************************************************
 * ****************Dev code begin*******************/
// Warning! Developer can add your new code here!

/* ****************Dev code end*********************
 * *************************************************/

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
static bool reset_horizon_integrator=false;
static bool ahrs_healthy=false;
static bool initial_compass_cal=false;
static bool initial_gnss=false;
static bool get_gnss_location=false;
static bool get_opticalflow=false;
static bool get_rangefinder_data=false;
static bool get_mav_yaw=false, get_odom_xy=false;
static bool mag_corrected=false, mag_correcting=false;
static bool use_uwb_pos_z=false;
static bool rc_channels_sendback=false;
static bool gcs_connected=false;
static bool offboard_connected=false;

static float accel_filt_hz=20;//HZ
static float gyro_filt_hz=20;//HZ
static float mag_filt_hz=5;//HZ
static float baro_filt_hz=2;//HZ
static float accel_ef_filt_hz=10;//HZ
static float uwb_pos_filt_hz=5;//HZ
static float rangefinder_filt_hz=10;//HZ
static float opticalflow_filt_hz=5;//HZ
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
static Vector3f odom_3d,odom_offset,uwb_pos;
static Vector2f odom_vel_xy_filt,odom_pos_xy_filt;
static Location gnss_origin_pos, gnss_current_pos;
static Vector3f ned_current_pos, ned_current_vel;
static Matrix3f dcm_matrix, dcm_matrix_correct;										//旋转矩阵
static LowPassFilter2pVector3f	_accel_filter, _gyro_filter, _accel_ef_filter;
static LowPassFilterFloat _baro_alt_filter;
static LowPassFilterVector3f _mag_filter, _uwb_pos_filter, _flow_gyro_filter;
static LowPassFilterVector2f _air_resistance_filter;

parameter *param=new parameter();
ap_t *ap=new ap_t();
AHRS *ahrs=new AHRS(_dt);
EKF_Baro *ekf_baro=new EKF_Baro(_dt, 0.0016, 1.0, 0.000016, 0.000016);
EKF_Rangefinder *ekf_rangefinder=new EKF_Rangefinder(_dt, 1.0, 0.000016, 0.16);
EKF_Odometry *ekf_odometry=new EKF_Odometry(_dt, 0.0016, 0.0016, 0.000016, 0.000016, 0.0004, 0.0004);
EKF_GNSS *ekf_gnss=new EKF_GNSS(_dt, 0.0016, 0.0016, 0.0016, 0.0016, 0.000016, 0.00016, 0.000016, 0.00016);
EKF_GNSS *ekf_opticalflow=new EKF_GNSS(_dt, 0.0016, 0.0016, 0.0016, 0.0016, 0.000016, 0.00016, 0.000016, 0.00016);
Motors *motors=new Motors(1/_dt);
Attitude_Multi *attitude=new Attitude_Multi(*motors, gyro_filt, _dt);
PosControl *pos_control=new PosControl(*motors, *attitude);
CompassCalibrator *compassCalibrator=new CompassCalibrator();
AccelCalibrator *accelCalibrator=new AccelCalibrator();
DataFlash *dataflash=new DataFlash();
SDLog *sdlog=new SDLog();
UWB *uwb=new UWB();
Rangefinder_state rangefinder_state;
Opticalflow_state opticalflow_state;

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

const Vector3f& get_accel_ef(void){								//地球坐标系下的三轴加速度m/ss
	return accel_ef;
}

const Vector3f& get_gyro_ef(void){								//地球坐标系下的三轴角速度
	return gyro_ef;
}

const Matrix3f& get_dcm_matrix(void){
	return dcm_matrix;
}

const Matrix3f& get_dcm_matrix_correct(void){
	return dcm_matrix_correct;
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
Location get_gnss_origin_pos(void){return gnss_origin_pos;}
Location get_gnss_current_pos(void){return gnss_current_pos;}
float get_ned_pos_x(void){return ned_current_pos.x;}
float get_ned_pos_y(void){return ned_current_pos.y;}
float get_ned_pos_z(void){return ned_current_pos.z;}
float get_ned_vel_x(void){return ned_current_vel.x;}
float get_ned_vel_y(void){return ned_current_vel.y;}
float get_ned_vel_z(void){return ned_current_vel.z;}
float get_odom_x(void){return odom_3d.x;}
float get_odom_y(void){return odom_3d.y;}
float get_odom_z(void){return odom_3d.z;}
float get_uwb_x(void){return uwb_pos.x;}
float get_uwb_y(void){return uwb_pos.y;}
float get_uwb_z(void){return uwb_pos.z;}
float get_yaw_map(void){return yaw_map;}
bool get_gnss_location_state(void){return get_gnss_location;}
bool get_gcs_connected(void){return gcs_connected;}
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
		addr_num_max=dataflash->get_addr_num_max();
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
		dataflash->set_param_float(param->spool_up_time.num, param->spool_up_time.value);
		dataflash->set_param_float(param->throttle_filt.num, param->throttle_filt.value);
		dataflash->set_param_vector3f(param->accel_offsets.num, param->accel_offsets.value);
		dataflash->set_param_vector3f(param->accel_diagonals.num, param->accel_diagonals.value);
		dataflash->set_param_vector3f(param->accel_offdiagonals.num, param->accel_offdiagonals.value);
		dataflash->set_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
		dataflash->set_param_uint16_channel8(param->channel_range.num, param->channel_range.channel);
		dataflash->set_param_float(param->auto_land_speed.num, param->auto_land_speed.value);
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
		dataflash->set_param_vector3f(param->vel_pid_integrator.num, param->vel_pid_integrator.value);
		dataflash->set_param_vector3f(param->rate_pid_integrator.num, param->rate_pid_integrator.value);
		dataflash->set_param_float(param->lowbatt_return_volt.num, param->lowbatt_return_volt.value);
		dataflash->set_param_float(param->lowbatt_land_volt.num, param->lowbatt_land_volt.value);
		dataflash->set_param_float(param->poshold_vel_max.num, param->poshold_vel_max.value);
		dataflash->set_param_float(param->poshold_accel_max.num, param->poshold_accel_max.value);
		dataflash->set_param_float(param->mission_vel_max.num, param->mission_vel_max.value);
		dataflash->set_param_float(param->mission_accel_max.num, param->mission_accel_max.value);
		dataflash->set_param_float(param->alt_return.num, param->alt_return.value);

		/* *************************************************
		* ****************Dev code begin*******************/
		// Warning! Developer can add your new code here!
		/* Demo
		 此处添加您的自定义参数, e.g.
		 dataflash->set_param_vector3f(param->demo_param_1.num, param->demo_param_1.value);
		 dataflash->set_param_float(param->demo_param_2.num, param->demo_param_2.value);
		 * */

		/* ****************Dev code end*********************
		* *************************************************/
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
		dataflash->get_param_float(param->spool_up_time.num, param->spool_up_time.value);
		dataflash->get_param_float(param->throttle_filt.num, param->throttle_filt.value);
		dataflash->get_param_vector3f(param->accel_offsets.num, param->accel_offsets.value);
		dataflash->get_param_vector3f(param->accel_diagonals.num, param->accel_diagonals.value);
		dataflash->get_param_vector3f(param->accel_offdiagonals.num, param->accel_offdiagonals.value);
		dataflash->get_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
		dataflash->get_param_uint16_channel8(param->channel_range.num, param->channel_range.channel);
		dataflash->get_param_float(param->auto_land_speed.num, param->auto_land_speed.value);
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
		dataflash->get_param_vector3f(param->vel_pid_integrator.num, param->vel_pid_integrator.value);
		dataflash->get_param_vector3f(param->rate_pid_integrator.num, param->rate_pid_integrator.value);
		dataflash->get_param_float(param->lowbatt_return_volt.num, param->lowbatt_return_volt.value);
		dataflash->get_param_float(param->lowbatt_land_volt.num, param->lowbatt_land_volt.value);
		dataflash->get_param_float(param->poshold_vel_max.num, param->poshold_vel_max.value);
		dataflash->get_param_float(param->poshold_accel_max.num, param->poshold_accel_max.value);
		dataflash->get_param_float(param->mission_vel_max.num, param->mission_vel_max.value);
		dataflash->get_param_float(param->mission_accel_max.num, param->mission_accel_max.value);
		dataflash->get_param_float(param->alt_return.num, param->alt_return.value);

		/* *************************************************
		 * ****************Dev code begin*******************/
		// Warning! Developer can add your new code here!
		/* Demo
		 此处添加您的自定义参数, e.g.
		 dataflash->get_param_vector3f(param->demo_param_1.num, param->demo_param_1.value);
		 dataflash->get_param_float(param->demo_param_2.num, param->demo_param_2.value);
		 * */

		/* ****************Dev code end*********************
		 * *************************************************/
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
static Vector3f tfmini_offset=Vector3f(0.0f,-4.5f, 0.0f);//激光测距仪相对于机体中心的坐标,单位:cm (机头方向为x轴正方向, 机体右侧为y轴正方向)
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
				if(cordist>3&&cordist<=800){
					Vector3f pos_offset=dcm_matrix*tfmini_offset;
					if(!rangefinder_state.alt_healthy){
						rangefinder_state.alt_cm_filt.reset((float)cordist);//重置滤波器
					}
					rangefinder_state.alt_cm=rangefinder_state.alt_cm_filt.apply((float)cordist);
					rangefinder_state.alt_cm=rangefinder_state.alt_cm*MAX(0.707f, dcm_matrix.c.z)+pos_offset.z;
					rangefinder_state.last_healthy_ms=HAL_GetTick();
					get_rangefinder_data=true;
					rangefinder_state.alt_healthy=true;
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

static float flow_alt, flow_bf_x, flow_bf_y;
static Vector3f flow_gyro_offset;
void opticalflow_update(void){
#if USE_FLOW
	flow_alt=100.0f;//cm
	if(lc302_data.quality==245){
		opticalflow_state.healthy=true;
		get_opticalflow=true;
	}else{
		opticalflow_state.healthy=false;
		return;
	}
	//光流坐标系->机体坐标系//TODO:add gyro offset
	flow_bf_x=(float)lc302_data.flow_y_integral/10000.0f-constrain_float(flow_gyro_offset.y/50, -0.1, 0.1);
	flow_bf_y=-(float)lc302_data.flow_x_integral/10000.0f+constrain_float(flow_gyro_offset.x/50, -0.1, 0.1);
	//机体坐标系->大地坐标系
	opticalflow_state.rads.x=flow_bf_x*ahrs_cos_yaw()-flow_bf_y*ahrs_sin_yaw();
	opticalflow_state.rads.y=flow_bf_x*ahrs_sin_yaw()+flow_bf_y*ahrs_cos_yaw();
	opticalflow_state.flow_dt=(float)lc302_data.integration_timespan/1000000.0f;
	opticalflow_state.vel=opticalflow_state.vel_filter.apply(opticalflow_state.rads*flow_alt/opticalflow_state.flow_dt);
	opticalflow_state.pos+=opticalflow_state.vel*opticalflow_state.flow_dt;
#endif
}

//接收
static uint32_t time_last_heartbeat[5]={0};
static uint32_t time_last_attitude=0;
static mavlink_heartbeat_t heartbeat;
static mavlink_set_mode_t setmode;
static mavlink_mission_count_t mission_count;
static mavlink_mission_item_t mission_item;
static mavlink_command_long_t cmd;
static mavlink_rc_channels_override_t rc_channels;
static mavlink_attitude_t attitude_mav;
static mavlink_local_position_ned_cov_t local_position_ned_cov;
static mavlink_set_position_target_local_ned_t set_position_target_local_ned;
static Vector3f lidar_offset=Vector3f(0.0f,0.0f, -16.0f);//cm
static uint8_t gcs_channel=255;
static uint16_t gnss_point_statis=0;
//发送
static mavlink_system_t mavlink_system;
static mavlink_message_t msg_global_attitude_position, msg_global_position_int, msg_command_long, msg_battery_status, msg_rc_channels, msg_mission_count, msg_mission_item, msg_system_version;
static mavlink_global_vision_position_estimate_t global_attitude_position;
static mavlink_global_position_int_t global_position_int;
static mavlink_mission_count_t mission_count_send;
static mavlink_mission_item_t mission_item_send;
static mavlink_command_long_t command_long;
static mavlink_battery_status_t battery_status;
static mavlink_rc_channels_t rc_channels_t;
static mavlink_timesync_t system_version;

void parse_mavlink_data(mavlink_channel_t chan, uint8_t data, mavlink_message_t* msg_received, mavlink_status_t* status){
	if (mavlink_parse_char(chan, data, msg_received, status)){
		switch (msg_received->msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
				mavlink_msg_heartbeat_decode(msg_received, &heartbeat);
				if((HeartBeatFlags&(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan))==0){
					HeartBeatFlags|=(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan);
					system_version.tc1=VERSION_HARDWARE;
					system_version.ts1=VERSION_FIRMWARE;
					mavlink_msg_timesync_encode(mavlink_system.sysid, mavlink_system.compid, &msg_system_version, &system_version);
					mavlink_send_buffer(chan, &msg_system_version);
				}
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
				}else if(setmode.base_mode==MAV_MODE_PREFLIGHT){
					if(get_soft_armed()||motors->get_interlock()||motors->get_armed()){
						break;
					}
					param->robot_type.value=(uint8_t)((setmode.custom_mode>>24)&0xFF);
					param->motor_type.value=(uint8_t)((setmode.custom_mode>>16)&0xFF);
					dataflash->set_param_uint8(param->robot_type.num, param->robot_type.value);
					dataflash->set_param_uint8(param->motor_type.num, param->motor_type.value);
					motors_init();
				}
				break;
			case MAVLINK_MSG_ID_MISSION_COUNT:
				mavlink_msg_mission_count_decode(msg_received, &mission_count);
				send_mavlink_mission_ack(chan, MAV_MISSION_ACCEPTED);
				gnss_point_statis=0;
				break;
			case MAVLINK_MSG_ID_MISSION_ITEM:
				mavlink_msg_mission_item_decode(msg_received, &mission_item);
				if(mission_item.seq==gnss_point_statis){
					gnss_point_statis++;//统计收到的航点数
				}
				//seq表示当前航点序号,x:lat,y:lon,z:alt
				sdlog->gnss_point[mission_item.seq]=Vector3f(mission_item.x,mission_item.y,mission_item.z);
				send_mavlink_mission_item_reached(chan, mission_item.seq);
				if(mission_count.count==mission_item.seq+1){//最后一个点接收完要把航点写入内存卡
					if(gnss_point_statis==mission_count.count){//如果统计航点数=航点总数，表示全部航点已被接收
						//航点总数
						sdlog->gnss_point_num=mission_count.count;
						sdlog->Logger_Write_Gnss();
					}
					gnss_point_statis=0;
				}
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST:
				//地面站请求航点总数
				send_mavlink_mission_count(chan);
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
				//地面站请求航点列表
				send_mavlink_mission_list(chan);
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
							}else {
								accelCalibrator->collect_sample();
							}
						}else if(is_equal(cmd.param1,2.0f)){			//start compass calibrate
							compass_cal_succeed=false;
							initial_compass_cal=false;
							mag_correcting=true;
							mag_corrected=false;
							ahrs->reset();
							param->mag_offsets.value={0.0f,0.0f,0.0f};
						}else if(is_equal(cmd.param1,3.0f)){            //校准水平
							horizon_correct=true;
							reset_horizon_integrator=true;
							param->vel_pid_integrator.value={0.0f,0.0f,0.0f};
							param->rate_pid_integrator.value={0.0f,0.0f,0.0f};
						}
						send_mavlink_commond_ack(chan, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACK_OK);
						break;
					case MAV_CMD_DO_SET_PARAMETER:
						if(is_equal(cmd.param1,0.5f)){//reset PID parameters in flash

							/* *************************************************
							 * ****************Dev code begin*******************/
							// Warning! Developer can add your new code here!
							/* Demo
							 如果希望通过app的一键重置按钮把参数重置为默认值，那么把参数重置代码添加在这里
							 param->demo_param_1.value={1.0,1.0,1.0};
							 dataflash->set_param_float(param->demo_param_1.num, param->demo_param_1.value);
							 param->demo_param_2.value=1.0f;
							 dataflash->set_param_float(param->demo_param_2.num, param->demo_param_2.value);
							 * */

							/* ****************Dev code end*********************
							 * *************************************************/

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
							param->auto_land_speed.value=AUTO_LAND_SPEED;
							param->rangefinder_gain.value=RANGEFINDER_GAIN_DEFAULT;
							param->angle_max.value=DEFAULT_ANGLE_MAX;
							param->pilot_accel_z.value=PILOT_ACCEL_Z_DEFAULT;
							param->pilot_takeoff_alt.value=PILOT_TKOFF_ALT_DEFAULT;
							param->spool_up_time.value=MOTORS_SPOOL_UP_TIME_DEFAULT;
							param->throttle_filt.value=MAN_THR_FILT_HZ;
							param->t_hover_update_min.value=THR_HOVER_UPDATE_MIN;
							param->t_hover_update_max.value=THR_HOVER_UPDATE_MAX;
							param->vib_land.value=VIB_LAND_THR;
							param->lowbatt_return_volt.value=LOWBATT_RETURN_VOLT;
							param->lowbatt_land_volt.value=LOWBATT_LAND_VOLT;
							param->poshold_vel_max.value=POSHOLD_VEL_MAX;
							param->poshold_accel_max.value=POSHOLD_ACCEL_MAX;
							param->mission_vel_max.value=MISSION_VEL_MAX;
							param->mission_accel_max.value=MISSION_ACCEL_MAX;
							param->alt_return.value=ALT_RETURN;

							dataflash->set_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
							dataflash->set_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
							dataflash->set_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
							dataflash->set_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
							dataflash->set_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
							dataflash->set_param_float(param->auto_land_speed.num, param->auto_land_speed.value);
							dataflash->set_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
							dataflash->set_param_float(param->angle_max.num, param->angle_max.value);
							dataflash->set_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
							dataflash->set_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
							dataflash->set_param_float(param->spool_up_time.num, param->spool_up_time.value);
							dataflash->set_param_float(param->throttle_filt.num, param->throttle_filt.value);
							dataflash->set_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
							dataflash->set_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
							dataflash->set_param_float(param->vib_land.num, param->vib_land.value);
							dataflash->set_param_float(param->lowbatt_return_volt.num, param->lowbatt_return_volt.value);
							dataflash->set_param_float(param->lowbatt_land_volt.num, param->lowbatt_land_volt.value);
							dataflash->set_param_float(param->poshold_vel_max.num, param->poshold_vel_max.value);
							dataflash->set_param_float(param->poshold_accel_max.num, param->poshold_accel_max.value);
							dataflash->set_param_float(param->mission_vel_max.num, param->mission_vel_max.value);
							dataflash->set_param_float(param->mission_accel_max.num, param->mission_accel_max.value);
							dataflash->set_param_float(param->alt_return.num, param->alt_return.value);

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
							attitude->set_lean_angle_max(param->angle_max.value);
							pos_control->set_lean_angle_max_d(param->angle_max.value);
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
							attitude->set_lean_angle_max(param->angle_max.value);
							pos_control->set_lean_angle_max_d(param->angle_max.value);
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
							param->spool_up_time.value=cmd.param2;
							dataflash->set_param_float(param->spool_up_time.num, param->spool_up_time.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=21.0f;
							command_long.param2=param->spool_up_time.value;
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
						}else if(is_equal(cmd.param1,26.0f)){
							param->auto_land_speed.value=cmd.param2;
							dataflash->set_param_float(param->auto_land_speed.num, param->auto_land_speed.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=26.0f;
							command_long.param2=param->auto_land_speed.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,27.0f)){
							param->lowbatt_return_volt.value=cmd.param2;
							dataflash->set_param_float(param->lowbatt_return_volt.num, param->lowbatt_return_volt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=27.0f;
							command_long.param2=param->lowbatt_return_volt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,28.0f)){
							param->lowbatt_land_volt.value=cmd.param2;
							dataflash->set_param_float(param->lowbatt_land_volt.num, param->lowbatt_land_volt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=28.0f;
							command_long.param2=param->lowbatt_land_volt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,29.0f)){
							param->poshold_vel_max.value=cmd.param2;
							dataflash->set_param_float(param->poshold_vel_max.num, param->poshold_vel_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=29.0f;
							command_long.param2=param->poshold_vel_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,30.0f)){
							param->poshold_accel_max.value=cmd.param2;
							dataflash->set_param_float(param->poshold_accel_max.num, param->poshold_accel_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=30.0f;
							command_long.param2=param->poshold_accel_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,31.0f)){
							param->mission_vel_max.value=cmd.param2;
							dataflash->set_param_float(param->mission_vel_max.num, param->mission_vel_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=31.0f;
							command_long.param2=param->mission_vel_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,32.0f)){
							param->mission_accel_max.value=cmd.param2;
							dataflash->set_param_float(param->mission_accel_max.num, param->mission_accel_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=32.0f;
							command_long.param2=param->mission_accel_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}else if(is_equal(cmd.param1,33.0f)){
							param->alt_return.value=cmd.param2;
							dataflash->set_param_float(param->alt_return.num, param->alt_return.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=33.0f;
							command_long.param2=param->alt_return.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						}
						/* *************************************************
						 * ****************Dev code begin*******************/
						// Warning! Developer can add your new code here!
						/* Demo
						 * 接收app设置的参数值
						 else if(is_equal(cmd.param1,1001.0f)){ 		//cmd.param1为自定义参数的mavlink id, 从1001开始
							param->demo_param_1.value.x=cmd.param2;		//cmd.param2~cmd.param7为参数实际内容。
							param->demo_param_1.value.y=cmd.param3;
							param->demo_param_1.value.z=cmd.param4;
							dataflash->set_param_vector3f(param->demo_param_1.num, param->demo_param_1.value);
							//接收完参数还要把参数传回给app用于校验
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=1001.0f;
							command_long.param2=param->demo_param_1.value.x;
							command_long.param3=param->demo_param_1.value.y;
							command_long.param4=param->demo_param_1.value.z;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
						 }
						 * */

						/* ****************Dev code end*********************
						 * *************************************************/
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
				if(rc_channels.target_component==0){			/***APP遥控功能已启用***/
					mav_channels_in[0]=rc_channels.chan1_raw;
					mav_channels_in[1]=rc_channels.chan2_raw;
					mav_channels_in[2]=rc_channels.chan3_raw;
					mav_channels_in[3]=rc_channels.chan4_raw;
					mav_channels_in[4]=rc_channels.chan5_raw;
					mav_channels_in[5]=rc_channels.chan6_raw;
					mav_channels_in[6]=rc_channels.chan7_raw;
					mav_channels_in[7]=rc_channels.chan8_raw;
					set_rc_channels_override(true);				//使能rc_channels_override把控制权给APP
					rc_channels_sendback=false;					//关闭遥控通道回传
				}else if(rc_channels.target_component==1){		/***遥控校准已确认***/
					set_rc_channels_override(false);			//清除rc_channels_override把控制权给遥控器
					rc_channels_sendback=true;					//启动遥控通道回传
					param->channel_range.channel[0]=rc_channels.chan1_raw;	//ch1_min
					param->channel_range.channel[1]=rc_channels.chan2_raw;	//ch2_min
					param->channel_range.channel[2]=rc_channels.chan3_raw;	//ch3_min
					param->channel_range.channel[3]=rc_channels.chan4_raw;	//ch4_min
					param->channel_range.channel[4]=rc_channels.chan5_raw;	//ch1_max
					param->channel_range.channel[5]=rc_channels.chan6_raw;	//ch2_max
					param->channel_range.channel[6]=rc_channels.chan7_raw;	//ch3_max
					param->channel_range.channel[7]=rc_channels.chan8_raw;	//ch4_max
					dataflash->set_param_uint16_channel8(param->channel_range.num, param->channel_range.channel);
					rc_range_init();
					send_mavlink_commond_ack(chan, (MAV_CMD)1, MAV_CMD_ACK_OK);
				}else if(rc_channels.target_component==2){		/***遥控校准已启动***/
					mav_channels_in[0]=1500;
					mav_channels_in[1]=1500;
					mav_channels_in[2]=1500;
					mav_channels_in[3]=1500;
					override_rc_channels(mav_channels_in);		//重置前四通道
					set_rc_channels_override(false);			//清除rc_channels_override把控制权给遥控器
					rc_channels_sendback=true;					//启动遥控通道回传
					send_mavlink_commond_ack(chan, (MAV_CMD)2, MAV_CMD_ACK_OK);
				}else{											/***APP遥控功能已禁用***/
					set_rc_channels_override(false);			//清除rc_channels_override把控制权给遥控器
					rc_channels_sendback=false;					//关闭遥控通道回传
					send_mavlink_commond_ack(chan, (MAV_CMD)3, MAV_CMD_ACK_OK);
				}
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

//系统启动后必须确保心跳函数1s运行一次
void send_mavlink_heartbeat_data(void){
	mavlink_message_t msg_heartbeat;
	mavlink_heartbeat_t heartbeat_send;
	mavlink_system.sysid=1;
	mavlink_system.compid=MAV_COMP_ID_AUTOPILOT1;
	heartbeat_send.type=param->robot_type.value;//机器人类型
	heartbeat_send.system_status=param->motor_type.value;//电机类型
	heartbeat_send.autopilot=robot_main_mode;//主模式
	heartbeat_send.custom_mode=robot_sub_mode;//子模式
	heartbeat_send.base_mode=0;
	if(get_soft_armed()){
		heartbeat_send.base_mode|=MAV_MODE_FLAG_SAFETY_ARMED;
	}
	if(sdlog->m_Logger_Status==SDLog::Logger_Record){
		heartbeat_send.base_mode|=MAV_MODE_FLAG_HIL_ENABLED;
	}
	mavlink_msg_heartbeat_encode(mavlink_system.sysid, mavlink_system.compid, &msg_heartbeat, &heartbeat_send);

	//电量
	battery_status.type=2;//3s锂电池
	battery_status.temperature=(int16_t)(get_baro_temp()*100);
	battery_status.voltages[0]=(uint16_t)(get_5v_in()*1000);
	battery_status.voltages[1]=(uint16_t)(get_batt_volt()*1000);
	battery_status.current_battery=(int16_t)(get_batt_current()*100);
	mavlink_msg_battery_status_encode(mavlink_system.sysid, mavlink_system.compid, &msg_battery_status, &battery_status);

#if COMM_0==MAV_COMM
		mavlink_send_buffer(MAVLINK_COMM_0, &msg_heartbeat);
		if(gcs_channel==MAVLINK_COMM_0){
			mavlink_send_buffer(MAVLINK_COMM_0, &msg_battery_status);
		}
#endif
#if COMM_1==MAV_COMM
	    mavlink_send_buffer(MAVLINK_COMM_1, &msg_heartbeat);
	    if(gcs_channel==MAVLINK_COMM_1){
			mavlink_send_buffer(MAVLINK_COMM_1, &msg_battery_status);
		}
#endif
#if COMM_2==MAV_COMM
	    mavlink_send_buffer(MAVLINK_COMM_2, &msg_heartbeat);
	    if(gcs_channel==MAVLINK_COMM_2){
			mavlink_send_buffer(MAVLINK_COMM_2, &msg_battery_status);
		}
#endif
#if COMM_3==MAV_COMM
	    mavlink_send_buffer(MAVLINK_COMM_3, &msg_heartbeat);
	    if(gcs_channel==MAVLINK_COMM_3){
			mavlink_send_buffer(MAVLINK_COMM_3, &msg_battery_status);
		}
#endif
#if COMM_4==MAV_COMM
	    mavlink_send_buffer(MAVLINK_COMM_4, &msg_heartbeat);
	    if(gcs_channel==MAVLINK_COMM_4){
			mavlink_send_buffer(MAVLINK_COMM_4, &msg_battery_status);
		}
#endif
}

void send_mavlink_mission_ack(mavlink_channel_t chan, MAV_MISSION_RESULT result){
	mavlink_message_t msg_mission_ack;
	mavlink_mission_ack_t mission_ack;
	mission_ack.type=result;
	mavlink_msg_mission_ack_encode(mavlink_system.sysid, mavlink_system.compid, &msg_mission_ack, &mission_ack);
	mavlink_send_buffer(chan, &msg_mission_ack);
}

void send_mavlink_mission_item_reached(mavlink_channel_t chan, uint16_t seq){
	mavlink_message_t msg_mission_item_reached;
	mavlink_mission_item_reached_t mission_item_reached;
	mission_item_reached.seq=seq;
	mavlink_msg_mission_item_reached_encode(mavlink_system.sysid, mavlink_system.compid, &msg_mission_item_reached, &mission_item_reached);
	mavlink_send_buffer(chan, &msg_mission_item_reached);
}

void send_mavlink_mission_count(mavlink_channel_t chan){
	mission_count_send.count=sdlog->gnss_point_num;
	mavlink_msg_mission_count_encode(mavlink_system.sysid, mavlink_system.compid, &msg_mission_count, &mission_count_send);
	mavlink_send_buffer(chan, &msg_mission_count);
}

void send_mavlink_mission_list(mavlink_channel_t chan){
	for(uint16_t i=0;i<sdlog->gnss_point_num;i++){
		mission_item_send.seq=i;
		mission_item_send.x=sdlog->gnss_point[i].x;
		mission_item_send.y=sdlog->gnss_point[i].y;
		mission_item_send.z=sdlog->gnss_point[i].z;
		mavlink_msg_mission_item_encode(mavlink_system.sysid, mavlink_system.compid, &msg_mission_item, &mission_item_send);
		mavlink_send_buffer(chan, &msg_mission_item);
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

static uint8_t accel_cali_num=0;
static uint32_t takeoff_time=0;
void send_mavlink_data(mavlink_channel_t chan)
{
	if((HAL_GetTick()-time_last_heartbeat[(uint8_t)chan])>5000){
		HeartBeatFlags&=(0xFF^(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan));
		if(gcs_channel==chan){
			gcs_channel=255;
		}
		if(offboard_connected){
			offboard_connected=false;
		}
		set_rc_channels_override(false);
		return;
	}
	uint32_t time=HAL_GetTick();

	//姿态+位置
	global_attitude_position.pitch=ahrs_pitch_rad();
	global_attitude_position.roll=ahrs_roll_rad();
	global_attitude_position.yaw=ahrs_yaw_rad();
	global_attitude_position.x=get_pos_x();
	global_attitude_position.y=get_pos_y();
	global_attitude_position.z=get_pos_z();
	global_attitude_position.usec=time;
	mavlink_msg_global_vision_position_estimate_encode(mavlink_system.sysid, mavlink_system.compid, &msg_global_attitude_position, &global_attitude_position);
	mavlink_send_buffer(chan, &msg_global_attitude_position);

	//经纬高+速度
	global_position_int.lat=gps_position->lat;//deg*1e7
	global_position_int.lon=gps_position->lon;//deg*1e7
	global_position_int.alt=gps_position->alt;//mm
	global_position_int.relative_alt=(int32_t)(rangefinder_state.alt_cm*10);//对地高度 mm
	global_position_int.hdg=gps_position->satellites_used;//卫星数
	global_position_int.vx=get_vel_x(); //速度cm/s
	global_position_int.vy=get_vel_y(); //速度cm/s
	global_position_int.vz=get_vel_z(); //速度cm/s
	if(takeoff_time>0){
		global_position_int.time_boot_ms=time-takeoff_time;//起飞时间 ms
	}else{
		global_position_int.time_boot_ms=0;
	}
	mavlink_msg_global_position_int_encode(mavlink_system.sysid, mavlink_system.compid, &msg_global_position_int, &global_position_int);
	mavlink_send_buffer(chan, &msg_global_position_int);

	//加速度计校准
	if(!accel_cal_succeed){
		accel_cali_num=0;
		command_long.command=MAV_CMD_PREFLIGHT_CALIBRATION;
		command_long.param1=1.0f;
		command_long.param2=(float)accelCalibrator->get_num_samples_collected()+1;
		mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
		mavlink_send_buffer(chan, &msg_command_long);
	}else{
		if((accelCalibrator->get_num_samples_collected()==ACCELCAL_VEHICLE_POS_BACK)&&(accel_cali_num<50)){
			command_long.command=MAV_CMD_PREFLIGHT_CALIBRATION;
			command_long.param1=1.0f;
			command_long.param2=(float)accelCalibrator->get_num_samples_collected()+1;
			mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
			mavlink_send_buffer(chan, &msg_command_long);
			accel_cali_num++;
		}
	}

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

	//电脑端地面站需要显示遥控信号
	if((chan==gcs_channel)&&rc_channels_sendback){
		rc_channels_t.chan1_raw=input_channel_roll();
		rc_channels_t.chan2_raw=input_channel_pitch();
		rc_channels_t.chan3_raw=input_channel_throttle();
		rc_channels_t.chan4_raw=input_channel_yaw();
		rc_channels_t.chan5_raw=input_channel_5();
		rc_channels_t.chan6_raw=input_channel_6();
		rc_channels_t.chan7_raw=input_channel_7();
		rc_channels_t.chan8_raw=input_channel_8();
		rc_channels_t.chancount=RC_INPUT_CHANNELS;
		rc_channels_t.rssi=254;
		mavlink_msg_rc_channels_encode(mavlink_system.sysid, mavlink_system.compid, &msg_rc_channels, &rc_channels_t);
		mavlink_send_buffer(chan, &msg_rc_channels);
	}
}

void distribute_mavlink_data(void){
#if COMM_0==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_0){
		send_mavlink_data(MAVLINK_COMM_0);
	}
#endif
#if COMM_1==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_1){
		send_mavlink_data(MAVLINK_COMM_1);
	}
#endif
#if COMM_2==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_2){
		send_mavlink_data(MAVLINK_COMM_2);
	}
#endif
#if COMM_3==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_3){
		send_mavlink_data(MAVLINK_COMM_3);
	}
#endif
#if COMM_4==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_4){
		send_mavlink_data(MAVLINK_COMM_4);
	}
#endif
}

static mavlink_message_t msg_received;
static mavlink_status_t status;
void comm0_callback(uint8_t data){
#if COMM_0==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_0, data, &msg_received, &status);
#endif
}

void comm1_callback(uint8_t data){
#if COMM_1==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_1, data, &msg_received, &status);
#elif COMM_1==GPS_COMM
	get_gps_data(data);
#elif COMM_1==TFMINI_COMM
	get_tfmini_data(data);
#elif COMM_1==LC302_COMM
	get_lc302_data(data);
#endif
}

void comm2_callback(uint8_t data){
#if COMM_2==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_2, data, &msg_received, &status);
#elif COMM_2==GPS_COMM
	get_gps_data(data);
#elif COMM_2==TFMINI_COMM
	get_tfmini_data(data);
#elif COMM_2==LC302_COMM
	get_lc302_data(data);
#endif
}

void comm3_callback(uint8_t data){
#if COMM_3==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_3, data, &msg_received, &status);
#elif COMM_3==GPS_COMM
	get_gps_data(data);
#elif COMM_3==TFMINI_COMM
	get_tfmini_data(data);
#elif COMM_3==LC302_COMM
	get_lc302_data(data);
#endif
}

void comm4_callback(uint8_t data){
#if COMM_4==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_4, data, &msg_received, &status);
#elif COMM_4==GPS_COMM
	get_gps_data(data);
#elif COMM_4==TFMINI_COMM
	get_tfmini_data(data);
#elif COMM_4==LC302_COMM
	get_lc302_data(data);
#endif
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
	command_long.param2=param->spool_up_time.value;
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

	command_long.param1=26.0f;
	command_long.param2=param->auto_land_speed.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=27.0f;
	command_long.param2=param->lowbatt_return_volt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=28.0f;
	command_long.param2=param->lowbatt_land_volt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=29.0f;
	command_long.param2=param->poshold_vel_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=30.0f;
	command_long.param2=param->poshold_accel_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=31.0f;
	command_long.param2=param->mission_vel_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=32.0f;
	command_long.param2=param->mission_accel_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=33.0f;
	command_long.param2=param->alt_return.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	/* *************************************************
	 * ****************Dev code begin*******************/
	// Warning! Developer can add your new code here!
	/* Demo
	 * 刷新参数列表
	command_long.param1=1001.0f;
	command_long.param2=param->demo_param_1.value.x;
	command_long.param3=param->demo_param_1.value.y;
	command_long.param4=param->demo_param_1.value.z;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);
	 * */

	/* ****************Dev code end*********************
	 * *************************************************/
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

extern float *rc_range_min,*rc_range_max;
void rc_range_init(void){
	rc_range_min[0]=(float)param->channel_range.channel[0];
	rc_range_min[1]=(float)param->channel_range.channel[1];
	rc_range_min[2]=(float)param->channel_range.channel[2];
	rc_range_min[3]=(float)param->channel_range.channel[3];
	rc_range_max[0]=(float)param->channel_range.channel[4];
	rc_range_max[1]=(float)param->channel_range.channel[5];
	rc_range_max[2]=(float)param->channel_range.channel[6];
	rc_range_max[3]=(float)param->channel_range.channel[7];
	for(uint8_t i=0;i<4;i++){
		if(rc_range_min[i]<1000||rc_range_min[i]>1100){
			rc_range_min[i]=1100;
		}
	}
	for(uint8_t i=0;i<4;i++){
		if(rc_range_max[i]<1900||rc_range_max[i]>2000){
			rc_range_max[i]=1900;
		}
	}
	rc_range_cal();
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
	motors->set_throttle_hover(constrain_float(motors->get_throttle_hover(), param->t_hover_update_min.value, param->t_hover_update_max.value));
	// disable output to motors and servos
	set_rcout_enable(false);
	FMU_PWM_Set_Output_Disable();
	motors->set_interlock(false);
	FMU_LED4_Control(true);
	FMU_LED7_Control(false);
}

void attitude_init(void){
	dcm_matrix_correct.from_euler(param->horizontal_correct.value.x, param->horizontal_correct.value.y, param->horizontal_correct.value.z);
	attitude->set_rotation_target_to_body(dcm_matrix_correct);
	attitude->get_angle_roll_p()(param->angle_roll_p.value);
	attitude->get_angle_pitch_p()(param->angle_pitch_p.value);
	attitude->get_angle_yaw_p()(param->angle_yaw_p.value);
	attitude->get_rate_roll_pid()(param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
			param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz, _dt);
	attitude->get_rate_pitch_pid()(param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
			param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz, _dt);
	attitude->get_rate_yaw_pid()(param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
			param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz, _dt);
	attitude->set_lean_angle_max(param->angle_max.value);
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
	pos_control->set_lean_angle_max_d(param->angle_max.value);
	sdlog->Logger_Read_Gnss();
	rangefinder_state.alt_cm_filt.set_cutoff_frequency(100, rangefinder_filt_hz);//tfmini默认频率100hz
	opticalflow_state.vel_filter.set_cutoff_frequency(50, opticalflow_filt_hz);//光流默认频率50hz
	_uwb_pos_filter.set_cutoff_frequency(uwb_pos_filt_hz);
}

bool uwb_init(void){
	if(uwb->uwb_init()){
		uwb->config_uwb(tag, 1, 1, 1, 1, 4);
		uwb->set_anchor_positon(1, 0, 0, 0);
		uwb->set_anchor_positon(2, 0, 240, 0);
		uwb->set_anchor_positon(3, 240, 0, 0);
		uwb->set_anchor_positon(4, 0, 0, 150);
	}else{
		return false;
	}
	return true;
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

	gyro.x = icm20608_data.gyrof.x;//rad/s
	gyro.y = icm20608_data.gyrof.y;//rad/s
	gyro.z = icm20608_data.gyrof.z;//rad/s

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
			_flow_gyro_filter.set_cutoff_frequency(400, opticalflow_filt_hz);
			_accel_ef_filter.set_cutoff_frequency(400, accel_ef_filt_hz);
			_air_resistance_filter.set_cutoff_frequency(400, 1);
			ahrs_stage_compass=true;
		}
	}else{
		if(accel_correct.length()<50){//过滤奇异值
			accel_filt=_accel_filter.apply(accel_correct);
			gyro_filt=_gyro_filter.apply(gyro_correct);
			flow_gyro_offset=_flow_gyro_filter.apply(gyro_filt);
		}
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
			MPU_CS_L;
		    IMU_Request_Data();
		    IMU_Get_Data();
		    MPU_CS_H;
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
#if USE_MAG
	mag.x = qmc5883_data.magf.x;
	mag.y = qmc5883_data.magf.y;
	mag.z = qmc5883_data.magf.z;

	if(isnan(mag.x) || isinf(mag.x) || isnan(mag.y) || isinf(mag.y) || isnan(mag.z) || isinf(mag.z)){
		return;
	}

	mag.rotate(ROTATION_YAW_270);

	mag_correct=mag+param->mag_offsets.value;

	if(is_equal(param->mag_offsets.value.x,0.0f)||is_equal(param->mag_offsets.value.y,0.0f)||is_equal(param->mag_offsets.value.z,0.0f)){
		return;
	}

	if(!initial_mag){
		mag_filt=mag_correct;
		_mag_filter.set_cutoff_frequency(100, mag_filt_hz);
		initial_mag=true;
	}else{
		mag_filt = _mag_filter.apply(mag_correct);
		mag_corrected=true;
	}
#endif
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
		compassCalibrator->get_calibration(param->mag_offsets.value);
		dataflash->set_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
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
		roll_sum+=roll_rad;
		pitch_sum+=pitch_rad;
		if(horizon_correct_flag==100){
			param->horizontal_correct.value.x=-roll_sum/100;
			param->horizontal_correct.value.y=-pitch_sum/100;
			param->horizontal_correct.value.z=0;
			dataflash->set_param_vector3f(param->horizontal_correct.num, param->horizontal_correct.value);
			dcm_matrix_correct.from_euler(param->horizontal_correct.value.x, param->horizontal_correct.value.y, param->horizontal_correct.value.z);
			attitude->set_rotation_target_to_body(dcm_matrix_correct);
			horizon_correct=false;
			roll_sum=0;
			pitch_sum=0;
			horizon_correct_flag=0;
		}
	}

	if(ahrs_stage_compass){
		ahrs->update(mag_corrected, get_mav_yaw);
		//由ahrs的四元数推出旋转矩阵用于控制
		ahrs->quaternion2.rotation_matrix(dcm_matrix);
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
#if USE_MAG
	if(mag_correcting){
		compass_calibrate();
		ahrs_healthy=false;
	}
#endif
}

static float baro_alt_filt=0,baro_alt_init=0,baro_alt_last=0,baro_alt_correct=0,gnss_alt_last=0;
static float rf_alt_delta=0, rf_alt_last=0, gnss_alt_delta=0,baro_alt_delta=0,vel_2d=0,accel_2d=0;
static uint16_t init_baro=0;
static float K_gain=0.0f;
static bool rf_correct=false;
void update_baro_alt(void){
	if(init_baro<20){//前20点不要
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
	if(init_baro<30){
		baro_alt_init+=baro_alt/10;
		init_baro++;
		return;
	}
	if(!initial_baro){
		_baro_alt_filter.set_cutoff_frequency(10, baro_filt_hz);
		initial_baro=true;
	}else{
		baro_alt-=baro_alt_init;
		if(get_gps_state()){
			if(is_equal(K_gain, 0.0f)){
				K_gain=constrain_float((float)gps_position->satellites_used/30, 0.0f, 1.0f);
				gnss_alt_last=ned_current_pos.z;
				gnss_alt_delta=0;
			}else{
				K_gain=constrain_float((float)gps_position->satellites_used/30, 0.0f, 1.0f);
				gnss_alt_delta=ned_current_pos.z-gnss_alt_last;
				gnss_alt_last=ned_current_pos.z;
				if(is_equal(gnss_alt_delta, 0.0f)){
					K_gain=0.0f;
				}
			}
			vel_2d=sqrtf(sq(get_vel_x(),get_vel_y()));
		}else{
			K_gain=0.0f;
			vel_2d=0.0f;
		}
		if(rangefinder_state.alt_healthy){
			rf_alt_delta=rangefinder_state.alt_cm-rf_alt_last;
			rf_alt_last=rangefinder_state.alt_cm;
			if(abs(rf_alt_delta)<100.0f&&!is_equal(rf_alt_delta,0.0f)){
				rf_correct=true;
			}else{
				rf_correct=false;
			}
		}else{
			rf_correct=false;
		}
		baro_alt_delta=baro_alt-baro_alt_last;
		baro_alt_last=baro_alt;
		accel_2d=sqrtf(sq(get_accel_ef().x,get_accel_ef().y));
		if(rf_correct&&(baro_alt_delta*rf_alt_delta<0||abs(baro_alt_delta)>15.0f||accel_2d>1.0f)){//防止水平飞行掉高和大风扰动
			baro_alt_delta=rf_alt_delta;
		}else if(K_gain>0.2f&&(abs(baro_alt_delta)>15.0f||accel_2d>1.0f)&&abs(gnss_alt_delta)<100.0f){//防止水平飞行掉高和大风扰动
			baro_alt_delta=gnss_alt_delta;
		}else{
			if(vel_2d<100&&accel_2d<1.0f){
				baro_alt_delta=baro_alt-baro_alt_correct;
			}
			if(abs(get_vel_z())<100.0f){
				baro_alt_delta=constrain_float(baro_alt_delta, -15.0f, 15.0f);
			}
		}
		baro_alt_correct+=baro_alt_delta;
		baro_alt_filt = _baro_alt_filter.apply(baro_alt_correct);
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

void uwb_update(void){
	uwb->uwb_update();
}

static uint32_t currunt_uwb_ms, last_uwb_ms = 0;
void uwb_position_update(void){
#if USE_UWB
	write_gpio2(true);
	FMU_LED6_Control(uwb->get_uwb_position);
	if(!uwb->get_uwb_position){
		return;
	}
	uwb->get_uwb_position=false;
	float theta=1.65;
	uwb_pos.x=uwb->uwb_position.x*cosf(theta)+uwb->uwb_position.y*sinf(theta);
	uwb_pos.y=-uwb->uwb_position.x*sinf(theta)+uwb->uwb_position.y*cosf(theta);
	uwb_pos.z=uwb->uwb_position.z;
	currunt_uwb_ms=HAL_GetTick();
	uwb_pos = _uwb_pos_filter.apply(uwb_pos, (float)(currunt_uwb_ms-last_uwb_ms)/1000.0f);
	last_uwb_ms = currunt_uwb_ms;
#endif
}

uint32_t get_uwb_last_ms(void){
	return last_uwb_ms;
}

void ekf_odom_xy(void){
#if USE_ODOMETRY
	if(!ahrs->is_initialed()||(!ahrs_healthy)){
		return;
	}
	if(odom_3d.x==0&&odom_3d.y==0){
		return;
	}
	ekf_odometry->update(get_odom_xy,odom_3d.x,odom_3d.y);
#endif
}

void ekf_opticalflow_xy(void){
#if USE_FLOW
	if(!ahrs->is_initialed()||(!ahrs_healthy)){
		return;
	}
	ekf_opticalflow->update(get_opticalflow, opticalflow_state.pos.x, opticalflow_state.pos.y, opticalflow_state.vel.x, opticalflow_state.vel.y);
#endif
}

void ekf_gnss_xy(void){
#if USE_GPS
	if(!ahrs->is_initialed()||(!ahrs_healthy)||!get_gps_state()){
		return;
	}
	ekf_gnss->update(get_gnss_location,get_ned_pos_x(),get_ned_pos_y(),get_ned_vel_x(),get_ned_vel_y());
#endif
}

float get_pos_x(void){//cm
	return ekf_opticalflow->pos_x;
}

float get_pos_y(void){//cm
	return ekf_opticalflow->pos_y;
}

float get_pos_z(void){//cm
	return ekf_baro->pos_z;
}

float get_vel_x(void){//cm/s
	return ekf_opticalflow->vel_x;
}

float get_vel_y(void){//cm/s
	return ekf_opticalflow->vel_y;
}

float get_vel_z(void){//cm/s
	return ekf_baro->vel_z;
}

void sdled_update(void){
	if(get_soft_armed()){
		FMU_LED3_Control(true);
	}else{
		FMU_LED3_Control(false);
	}
	osDelay(200);
	if(sdlog->m_Logger_Status!=SDLog::Logger_Record){
		FMU_LED3_Control(false);
	}
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
static void update_throttle_hover(void)
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

static Vector2f air_resistance_bf;
void update_air_resistance(void)
{
	const float euler_roll_angle = ahrs_roll_rad();
	const float euler_pitch_angle = ahrs_pitch_rad();
	const float pilot_cos_pitch_target = cosf(euler_pitch_angle);
	const float pilot_accel_rgt = GRAVITY_MSS * tanf(euler_roll_angle)/pilot_cos_pitch_target;
	const float pilot_accel_fwd = -GRAVITY_MSS * tanf(euler_pitch_angle);

	air_resistance_bf.x=pilot_accel_fwd - (accel_ef.x*cos_yaw + accel_ef.y*sin_yaw);
	air_resistance_bf.y=pilot_accel_rgt - (-accel_ef.x*sin_yaw + accel_ef.y*cos_yaw);
	air_resistance_bf=_air_resistance_filter.apply(air_resistance_bf);
}

void get_air_resistance_lean_angles(float &roll_d, float &pitch_d, float angle_max, float gain)
{
    float angle_limit=angle_max/constrain_float(air_resistance_bf.length()*gain, 1.0f, 3.0f);
    float total_in = norm(roll_d, pitch_d);
    if (total_in > angle_limit) {
		float ratio = angle_limit / total_in;
		roll_d *= ratio;
		pitch_d *= ratio;
	}
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
    use_uwb_pos_z=false;
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
	use_uwb_pos_z=true;
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

// arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool arm_motors(void)
{
	if (!motors->get_interlock()) {
		return false;
	}
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
    takeoff_time=HAL_GetTick();
    Buzzer_set_ring_type(BUZZER_ARMED);

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

    if(reset_horizon_integrator&&robot_state==STATE_FLYING){
		dataflash->set_param_vector3f(param->vel_pid_integrator.num, param->vel_pid_integrator.value);
		param->rate_pid_integrator.value.x=attitude->get_rate_roll_pid().get_integrator();
		param->rate_pid_integrator.value.y=attitude->get_rate_pitch_pid().get_integrator();
		param->rate_pid_integrator.value.z=attitude->get_rate_yaw_pid().get_integrator();
		dataflash->set_param_vector3f(param->rate_pid_integrator.num, param->rate_pid_integrator.value);
	}

    // we are not in the air
    set_land_complete(true);

    // send disarm command to motors
    motors->set_armed(false);

    set_soft_armed(false);

    //停止日志记录
    sdlog->Logger_Disable();
    takeoff_time=0;
    Buzzer_set_ring_type(BUZZER_DISARM);
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

// counter to verify landings
static uint32_t land_detector_count = 0;
// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at 100hz
static void update_land_detector(void)
{
	ahrs->check_vibration();
	//******************落地前********************
	if((pos_control->get_desired_velocity().z<0)&&(get_vib_value()>param->vib_land.value)&&(motors->get_throttle()<motors->get_throttle_hover())&&(!motors->limit.throttle_lower)){//TODO:降落时防止弹起来
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
        // we've landed so reset land_detector
        land_detector_count = 0;
    } else if (ap->land_complete) {
        // if throttle output is high then clear landing flag
        if (motors->get_throttle() > get_non_takeoff_throttle()) {
            set_land_complete(false);
        }
        // we've landed so reset land_detector
        land_detector_count = 0;
    } else {

        // check that the average throttle output is near minimum (less than 12.5% hover throttle)
        bool motor_at_lower_limit = motors->limit.throttle_lower && attitude->is_throttle_mix_min();

        // check that vertical speed is within 1m/s of zero
        bool descent_rate_low = fabsf(get_vel_z()) < 100;

        // if we have a healthy rangefinder only allow landing detection below 2 meters
        bool rangefinder_check = (!rangefinder_state.alt_healthy || rangefinder_state.alt_cm < LAND_RANGEFINDER_MIN_ALT_CM);

        if (motor_at_lower_limit && descent_rate_low && rangefinder_check) {
            // landed criteria met - increment the counter and check if we've triggered
            if( land_detector_count < ((float)LAND_DETECTOR_TRIGGER_SEC)/0.01) {
                land_detector_count++;
            } else {
            	disarm_motors();
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
static void update_throttle_thr_mix()
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
	// update estimated throttle required to hover
	update_throttle_hover();
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
	update_throttle_thr_mix();
	//checks if we have landed and updates the ap.land_complete flag
	update_land_detector();
}

//手势开关电机；
//油门0，偏航最大时启动电机；
//油门0，偏航最小时关闭电机。
//注意不是手动油门时，只有在降落状态时才能用手势关闭电机。
void arm_motors_check(void){
	static int16_t arming_counter;
	float throttle=get_channel_throttle();
	// ensure throttle is down
	if (throttle > 0.05) {
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
    if (throttle_control > 0.01 && get_soft_armed()) {
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

static mavlink_message_t msg_scaled_imu;
static mavlink_scaled_imu_t scaled_imu;
void offboard_callback(void){
	if(offboard_connected){
		scaled_imu.time_boot_ms=HAL_GetTick();
		scaled_imu.xacc=(uint16_t)(get_accel_filt().x*1000);
		scaled_imu.yacc=(uint16_t)(get_accel_filt().y*1000);
		scaled_imu.zacc=(uint16_t)(get_accel_filt().z*1000);
		scaled_imu.xgyro=(uint16_t)(get_gyro_filt().x*1000);
		scaled_imu.ygyro=(uint16_t)(get_gyro_filt().y*1000);
		scaled_imu.zgyro=(uint16_t)(get_gyro_filt().z*1000);
		mavlink_msg_scaled_imu_encode(mavlink_system.sysid, mavlink_system.compid, &msg_scaled_imu, &scaled_imu);
		mavlink_send_buffer(MAVLINK_COMM_0, &msg_scaled_imu);
	}
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
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s ",//LOG_SENSOR
			"magx", "magy", "magz", "baro", "voltage", "current", "sat_num");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s ",//LOG_EULER
			"pitchr", "rollr", "yawr", "pitchd", "rolld", "yawd");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_ACCEL_EARTH_FRAME and VIB
			"gyrox_t", "gyroy_t", "gyroz_t", "efx", "efy", "efz", "vib_vl", "vib_ag");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_POS_Z
			"barofilt", "alt_t", "pos_z", "vel_z_t", "vel_z", "rf_alt", "rf_alt_t", "rtk_alt", "rtk_velz");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s ",//LOG_POS_XY
			"vt_z", "odom_x", "pos_x", "vel_x", "odom_y", "pos_y", "vel_y");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_VEL_PID_XYZ
			"v_p_x", "v_i_x", "v_d_x", "v_p_y", "v_i_y", "v_d_y", "a_p_z", "a_i_z", "a_d_z");
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
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_RATE_CONTROL
			"roll_p", "roll_i", "roll_d", "pitch_p", "pitch_i", "pitch_d", "yaw_p", "yaw_i", "yaw_d");
	osDelay(1);
	sdlog->Logger_Write("%8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_MOTOR_CONTROL
			"roll_t", "pitch_t", "yaw_t", "p_out", "r_out", "y_out", "t_out", "t_hover");
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
			HAL_GetTick(), get_accel_filt().x, get_accel_filt().y, get_accel_filt().z, get_gyro_filt().x, get_gyro_filt().y, get_gyro_filt().z);
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8d ",//LOG_SENSOR
			get_mag_filt().x, get_mag_filt().y, get_mag_filt().z, spl06_data.baro_alt, get_batt_volt(), get_batt_current(), gps_position->satellites_used);
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_EULER
			ahrs_pitch_rad(), ahrs_roll_rad(), ahrs_yaw_rad(), ahrs_pitch_deg(), ahrs_roll_deg(), ahrs_yaw_deg());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_ACCEL_EARTH_FRAME and VIB
			attitude->rate_bf_targets().x, attitude->rate_bf_targets().y, attitude->rate_bf_targets().z, get_accel_ef().x, get_accel_ef().y, -(get_accel_ef().z + GRAVITY_MSS) * 100.0f, get_vib_value(), get_vib_angle_z());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_POS_Z
			get_baroalt_filt(), pos_control->get_pos_target().z, get_pos_z(), pos_control->get_vel_target_z(), get_vel_z(), get_rangefinder_alt(), get_rangefinder_alt_target(), get_ned_pos_z(), get_ned_vel_z());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_POS_XY
			ekf_baro->get_vt(), get_odom_x(), get_pos_x(), get_vel_x(), get_odom_y(), get_pos_y(), get_vel_y());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_VEL_PID_XY
			pos_control->get_vel_xy_pid().get_p().x, pos_control->get_vel_xy_pid().get_integrator().x, pos_control->get_vel_xy_pid().get_d().x,
			pos_control->get_vel_xy_pid().get_p().y, pos_control->get_vel_xy_pid().get_integrator().y, pos_control->get_vel_xy_pid().get_d().y,
			pos_control->get_accel_z_pid().get_p(), pos_control->get_accel_z_pid().get_integrator(), pos_control->get_accel_z_pid().get_d());
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
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_RATE_CONTROL
			attitude->get_rate_roll_pid().get_p(), attitude->get_rate_roll_pid().get_integrator(), attitude->get_rate_roll_pid().get_d(), attitude->get_rate_pitch_pid().get_p(), attitude->get_rate_pitch_pid().get_integrator(), attitude->get_rate_pitch_pid().get_d(),
			attitude->get_rate_yaw_pid().get_p(), attitude->get_rate_yaw_pid().get_integrator(), attitude->get_rate_yaw_pid().get_d());
	osDelay(1);
	sdlog->Logger_Write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_MOTOR_CONTROL
			attitude->get_att_target_euler_d().x, attitude->get_att_target_euler_d().y, attitude->get_att_target_euler_d().z,
			motors->get_pitch(), motors->get_roll(), motors->get_yaw(), motors->get_throttle(), motors->get_throttle_hover());
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
//	usb_printf("l:%d\n",get_comm3_available());
//	usb_printf("ux:%f|uy:%f|uz:%f|x:%f|y:%f|vx:%f|vy:%f\n", uwb->uwb_position.x, uwb->uwb_position.y,  uwb->uwb_position.z, get_pos_x(), get_pos_y(),get_vel_x(), get_vel_y());
//	usb_printf("gps_position lat:%lf ,lon:%lf ,alt:%lf \r\n" , (double)gps_position->lat/10000000.0,(double)gps_position->lon/10000000.0,(double)gps_position->alt/1000000.0);
//	usb_printf("l:%d|%d|%d\n",*(__IO uint8_t*)((uint32_t)0x081D0000),*(__IO uint8_t*)((uint32_t)0x081D0001),*(__IO uint8_t*)((uint32_t)0x081D0002));
//	usb_printf("l:%d\n",dataflash->get_addr_num_max());
//	dataflash->get_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
//	usb_printf("p:%f,%f,%f\n",uwb->uwb_position.x, uwb->uwb_position.y, uwb->uwb_position.z);
//	dataflash->set_param_float(param->acro_yaw_p.num, 3.6);
//	dataflash->get_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
//	usb_printf("%f\n",param->acro_yaw_p.value);
//	usb_printf("%f\n",motors->get_throttle_hover());
//	usb_printf("%f|%f\n",param.throttle_filt.value,param.angle_max.value);
//	usb_printf("x:%f ",param->accel_offsets.value.x);
//	usb_printf("lat:%d \n",gps_position->lat);
//	usb_printf("y:%f ",param->accel_offsets.value.y);
//	usb_printf("z:%f\n",param->accel_offsets.value.z);
//	usb_printf("baro:%f,",spl06_data.baro_alt);
//	usb_printf("temp:%f\n",spl06_data.temp);
//	usb_printf("alt:%f\n",get_rangefinder_alt());
//	float cos_tilt = ahrs_cos_pitch() * ahrs_cos_roll();
//	usb_printf("%f|%f|%f\n", pitch_deg, roll_deg, yaw_deg);
//	usb_printf("l:%f\n",get_mag_filt().length());
//	usb_printf("v:%f,i:%f\n",get_batt_volt(),get_batt_current());
//	usb_printf("gx:%f|gy:%f|gz:%f\n", gyro_filt.x, gyro_filt.y, gyro_filt.z);
//	usb_printf("mx:%f|my:%f|mz:%f\n", mag_filt.x, mag_filt.y, mag_filt.z);
//	usb_printf("x:%f,y:%f,z:%f\n",gyro_offset.x,gyro_offset.y,gyro_offset.z);
//	usb_printf("accelx:%f,accely:%f,accelz:%f\n",accel_correct.x,accel_correct.y,accel_correct.z);
//	usb_printf("accelx:%f,accely:%f,accelz:%f\n ",accel_filt.x,accel_filt.y,accel_filt.z);
//	usb_printf("accelx:%f,accely:%f,accelz:%f\n ",accel_ef.x,accel_ef.y,accel_ef.z);
//	usb_printf("ax:%f, ay:%f, az:%f, accelx:%f,accely:%f,accelz:%f\n ",dcm_matrix.a.x, dcm_matrix.a.y, dcm_matrix.a.z, accel_filt.x,accel_filt.y,accel_filt.z);
//	usb_printf("ax:%f,ay:%f,az:%f\n",mpu9250_data.accf.x,mpu9250_data.accf.y,mpu9250_data.accf.z);
//	usb_printf("%f,%f,%f\n",mpu9250_data.gyrof.x, mpu9250_data.gyrof.y, mpu9250_data.gyrof.z);
//	usb_printf("optical:%f,%f\n",opticalflow_state.pos.x, opticalflow_state.pos.y);
//	usb_printf("hover:%f|%f\n",param->t_hover_update_min.value,param->t_hover_update_max.value);
//	usb_printf("m:%d|%d\n",param.motor_type.value, param.robot_type.value);
//	usb_printf("pos:%f|%f|%f|%f\n",get_pos_x(),get_vel_x(),get_pos_y(),get_vel_y());
//	usb_printf("pitch:%f|roll:%f|yaw:%f\n", pitch_rad, roll_rad, yaw_rad);
//	usb_printf("vib:%f\n", param->vib_land.value);
//	s2_printf("x:%f,y:%f\n", x_target, y_target);
//	usb_printf("pos_z:%f|%f|%f|%f\n",get_baroalt_filt(),get_pos_z(),get_vel_z(),accel_ef.z);
//	usb_printf("speed:%f\n",param->auto_land_speed.value);
//	usb_printf("z:%f\n",attitude->get_angle_roll_p().kP());
//	usb_printf("gx:%f|gy:%f|gz:%f\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);
//	usb_printf("r:%d,p:%d,y:%f,t:%f,5:%f,6:%f,7:%f,8:%f\n",mav_channels_in[2],mav_channels_in[3],get_channel_yaw(), get_channel_throttle(),get_channel_5(),get_channel_6(),get_channel_7(),get_channel_8());
//	usb_printf("0:%f,1:%f,4:%f,5:%f\n",motors->get_thrust_rpyt_out(0),motors->get_thrust_rpyt_out(1),motors->get_thrust_rpyt_out(4), motors->get_thrust_rpyt_out(5));
//	usb_printf("roll:%f,pitch:%f,yaw:%f,throttle:%f\n",motors->get_roll(),motors->get_pitch(),motors->get_yaw(), motors->get_throttle());
//	usb_printf("yaw:%f,yaw_throttle:%f\n",yaw_deg,motors->get_yaw());
//	usb_printf("c:%f,p:%f\n",compass_calibrate(),param.mag_offsets.value.x);
//	usb_printf("i:%f|%f\n",param->vel_pid_integrator.value.x, param->vel_pid_integrator.value.y);
//	usb_printf("i:%f|%f\n",attitude->get_rate_roll_pid().get_integrator(), attitude->get_rate_pitch_pid().get_integrator());
//	usb_printf("ox:%f, oy:%f, oz:%f\n",param->mag_offsets.value.x, param->mag_offsets.value.y, param->mag_offsets.value.z);
//	usb_printf("dx:%f, dy:%f, dz:%f\n",param->mag_diagonals.value.x, param->mag_diagonals.value.y, param->mag_diagonals.value.z);
//	usb_printf("odx:%f, ody:%f, odz:%f\n",param->mag_offdiagonals.value.x, param->mag_offdiagonals.value.y, param->mag_offdiagonals.value.z);
//	usb_printf("motor:%d|%d|%d|%d\n",pwm_channel.motor[0], motors->get_armed(), get_soft_armed(), motors->get_interlock());
//	usb_printf("%d\n",robot_sub_mode);
//  usb_printf("vib_value:%f, vib_angle:%f\n", get_vib_value(), get_vib_angle_z());
//	usb_printf("point:%d,%f,%f,%f\n",sdlog->gnss_point_num,sdlog->gnss_point[0].x,sdlog->gnss_point[0].y,sdlog->gnss_point[0].z);
//	Servo_Set_Value(2,1500);
//	Servo_Set_Value(3,1500);
}
