/*
 * mode_usv_a.cpp
 *
 *  Created on: 2023年5月19日
 *      Author: 25053
 */
#include "maincontroller.h"

static float channel_roll = 0.0; //滚转角通道控制左右 y
static float channel_pitch = 0.0;  //俯仰角通道控制前后 x
static float channel_throttle=0.0;//油门通道控制是否启动电机
void mode_usv_a(void)
{
	robot_state=STATE_DRIVE;
	//(1)首先获取遥控器通道值
	channel_roll = get_channel_roll(); //range(-1,1)
	channel_pitch = get_channel_pitch(); //range(-1,1)
	channel_throttle=get_channel_throttle();  //range(0-1)

    //(2)输出
	if(get_soft_armed()&&(channel_throttle>0.1)){//解锁后,油门推起来才允许电机输出
		Motor_Set_Value(1, 1500+500*constrain_float(-channel_pitch+channel_roll, -1.0, 1.0));
		Motor_Set_Value(5, 1500+500*constrain_float(-channel_pitch-channel_roll, -1.0, 1.0));
	}else{//未解锁时电机无输出
		Motor_Set_Value(1, 1500);
		Motor_Set_Value(5, 1500);
	}
}



