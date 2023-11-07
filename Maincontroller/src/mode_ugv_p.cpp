/*
 * mode_ugv_p.cpp
 *
 *  Created on: 2023年6月12日
 *      Author: 10427
 */

#include "maincontroller.h"

static float channel_roll = 0.0;
static float channel_yaw = 0.0; //偏航角控制左转右转
static float channel_pitch = 0.0;  //俯仰角通道控制前后 x
static float channel_throttle=0.0;//油门通道控制是否启动电机
static float target_yaw=0.0f;
static float control_yaw = 0.0f;

static float AA = 0.0;//代表四个电机
static float BB = 0.0;
static float CC = 0.0;
static float DD = 0.0;

static void control_car(float channel_pitch, float channel_yaw)
{


	if(channel_pitch < 0.0f)
			{
				 AA = (-channel_pitch + channel_yaw) * 2500;
				 BB = (-channel_pitch + channel_yaw) * 2500;
				 CC = (-channel_pitch - channel_yaw) * 2500;
				 DD = (-channel_pitch - channel_yaw) * 2500;

			}
			else
			{
				 AA = (-channel_pitch - channel_yaw) * 2500;
				 BB = (-channel_pitch - channel_yaw) * 2500;
				 CC = (-channel_pitch + channel_yaw) * 2500;
				 DD = (-channel_pitch + channel_yaw) * 2500;
			}


				if (AA > 2500) AA = 2500;
				else if (AA < -2500) AA = -2500;

				if (BB > 2500) BB = 2500;
				else if (BB < -2500) BB = -2500;

				if (CC > 2500) CC = 2500;
				else if (CC < -2500) CC = -2500;

				if (DD > 2500) DD = 2500;
				else if (DD < -2500) DD = -2500;

				if(AA > 0.0f)
				{
					Motor_Set_Value(3, AA);
					Motor_Set_Value(4, 0);
				}
				else
				{
					Motor_Set_Value(3, 0);
					Motor_Set_Value(4, -AA);
				}

				if(BB > 0.0f)
				{
					Servo_Set_Value(1, BB);
					Servo_Set_Value(2, 0);
				}
				else
				{
					Servo_Set_Value(1, 0);
					Servo_Set_Value(2, -BB);
				}

				if(CC > 0.0f)
				{
					Motor_Set_Value(7, CC);
					Motor_Set_Value(8, 0);
				}
				else
				{
					Motor_Set_Value(7, 0);
					Motor_Set_Value(8, -CC);
				}
				if(DD > 0.0f)
				{
					Servo_Set_Value(3, DD);
					Servo_Set_Value(4, 0);
				}
				else
				{
					Servo_Set_Value(3, 0);
					Servo_Set_Value(4, -DD);
				}
}
bool mode_ugv_p_init(void)
{
	if(motors->get_armed()||motors->get_interlock()){//电机未锁定,禁止切换至该模式
		Buzzer_set_ring_type(BUZZER_ERROR);
		return false;
	}
	channel_roll = 0.0;
	channel_yaw = 0.0;
	channel_pitch = 0.0;
	channel_throttle=0.0;

	target_yaw = ahrs_yaw_rad();

	Buzzer_set_ring_type(BUZZER_MODE_SWITCH);


	reset_servo_freq(400);//用舵机口控制电机，重置舵机口频率。

	usb_printf("switch mode ugv_p !\n");
	return true;
}
void mode_ugv_p(void)
{
	robot_state=STATE_DRIVE;
	//(1)首先获取遥控器通道值
	channel_roll = get_channel_roll(); //range(-1,1)
	channel_yaw = get_channel_yaw(); //range(-1,1)
	channel_pitch = get_channel_pitch(); //range(-1,1)
	channel_throttle=get_channel_throttle();  //range(0-1)
	//(2)输出
	if(get_soft_armed()&&(channel_throttle>0.1))
	{
		float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());//获取摇杆设置的目标偏航角速度
		target_yaw = M_PI;
		//target_yaw+=target_yaw_rate*_dt;
		control_yaw = attitude->get_steering_out_heading(target_yaw, M_PI/2, false, false, _dt);
		//usb_printf("control_yaw:%f\n",control_yaw);
		control_car(channel_pitch, control_yaw);
	}
	else
	{
		target_yaw = ahrs_yaw_rad();
	}

}


