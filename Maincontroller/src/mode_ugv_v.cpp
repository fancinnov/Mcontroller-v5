/*
 * mode_ugv_v.cpp
 *
 *  Created on: 2022年9月27日
 *      Author: 25053
 */
#include "maincontroller.h"

static float channel_roll = 0.0; //滚转角通道控制左右 y
static float channel_yaw = 0.0; //偏航角控制旋转 rotate
static float channel_pitch = 0.0;  //俯仰角通道控制前后 x
static float channel_throttle=0.0;//油门通道控制是否启动电机
static float target_yaw=0.0f;
static uint16_t target_point=0;
static Location gnss_target_pos;
static Vector3f ned_target_pos;
static Vector2f ned_dis_2d;
void mode_ugv_v(void)
{
	robot_state=STATE_DRIVE;
	//(1)首先获取遥控器通道值
	channel_roll = get_channel_roll(); //range(-1,1)
	channel_yaw = get_channel_yaw(); //range(-1,1)
	channel_pitch = get_channel_pitch(); //range(-1,1)
	channel_throttle=get_channel_throttle();  //range(0-1)

	//(2)输出
	if(get_soft_armed()&&(channel_throttle>0.1)){//解锁后,油门推起来才允许电机输出
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
			if(target_point>sdlog->gnss_point_num){
				write_gpio1(false);//停止
				write_gpio2(false);
				write_gpio3(false);//停止
				write_gpio4(false);
			}else{
				float delta_yaw=wrap_180(target_yaw-ahrs_yaw_deg(), 1);
				usb_printf("yaw:%f|%f|%f\n",delta_yaw, target_yaw, ahrs_yaw_deg());
				if(delta_yaw>=0&&delta_yaw<=90){
					write_gpio1(true);//前进
					write_gpio2(false);
					write_gpio3(true);//右转
					write_gpio4(false);
				}else if(delta_yaw<0&&delta_yaw>=-90){
					write_gpio1(true);//前进
					write_gpio2(false);
					write_gpio3(false);//左转
					write_gpio4(true);
				}else if(delta_yaw<-90&&delta_yaw>-180){
					write_gpio1(false);//后退
					write_gpio2(true);
					write_gpio3(false);//左转
					write_gpio4(true);
				}else if(delta_yaw>90&&delta_yaw<=180){
					write_gpio1(false);//后退
					write_gpio2(true);
					write_gpio3(true);//右转
					write_gpio4(false);
				}
			}
		}else{
			write_gpio1(false);//停止
			write_gpio2(false);
			write_gpio3(false);//停止
			write_gpio4(false);
		}
	}else{
		write_gpio1(false);//停止
		write_gpio2(false);
		write_gpio3(false);//停止
		write_gpio4(false);
	}
}


