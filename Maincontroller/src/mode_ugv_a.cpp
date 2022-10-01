/*
 * mode_ugv_a.cpp
 *
 *  Created on: 2022年9月25日
 *      Author: 25053
 */
#include "maincontroller.h"

static float channel_roll = 0.0; //滚转角通道控制左右 y
static float channel_yaw = 0.0; //偏航角控制旋转 rotate
static float channel_pitch = 0.0;  //俯仰角通道控制前后 x
static float channel_throttle=0.0;//油门通道控制是否启动电机
void mode_ugv_a(void)
{
	robot_state=STATE_DRIVE;
	//(1)首先获取遥控器通道值
	channel_roll = get_channel_roll(); //range(-1,1)
	channel_yaw = get_channel_yaw(); //range(-1,1)
	channel_pitch = get_channel_pitch(); //range(-1,1)
	channel_throttle=get_channel_throttle();  //range(0-1)

    //(2)输出
	if(get_soft_armed()&&(channel_throttle>0.1)){//解锁后,油门推起来才允许电机输出
		//前进后退
		if(channel_pitch>0.9){
			write_gpio1(true);//前进
			write_gpio2(false);
		}else if(channel_pitch<-0.9){
			write_gpio1(false);//后退
			write_gpio2(true);
		}else{
			write_gpio1(false);//停止
			write_gpio2(false);
		}

		//左转右转
		if(channel_roll>0.9){
			write_gpio3(true);//右转
			write_gpio4(false);
		}else if(channel_roll<-0.9){
			write_gpio3(false);//左转
			write_gpio4(true);
		}else{
			write_gpio3(false);//停止
			write_gpio4(false);
		}
	}else{//未解锁时电机无输出
		write_gpio1(false);//停止
		write_gpio2(false);
		write_gpio3(false);//停止
		write_gpio4(false);
	}
}


