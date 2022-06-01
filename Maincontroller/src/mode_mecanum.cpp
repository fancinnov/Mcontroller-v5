/*
 * mode_mecanum.cpp
 *
 *  Created on: 2022年4月10日
 *      Author: 10427
 */

#include "maincontroller.h"

static float channel_roll = 0.0; //滚转角通道控制左右 y
static float channel_yaw = 0.0; //偏航角控制旋转 rotate
static float channel_pitch = 0.0;  //俯仰角通道控制前后 x
static float channel_throttle=0.0;//油门通道控制是否启动电机

bool mode_mecanum_init(void)
{
	if(motors->get_armed()||motors->get_interlock()){//电机未锁定,禁止切换至该模式
		Buzzer_set_ring_type(BUZZER_ERROR);
		return false;
	}
	channel_roll = 0.0;
	channel_yaw = 0.0;
	channel_pitch = 0.0;
	channel_throttle=0.0;
	Buzzer_set_ring_type(BUZZER_MODE_SWITCH);
	usb_printf("switch mode mecanum!\n");
	return true;
}

void mode_mecanum(void)
{
	robot_state=STATE_DRIVE;
	//(1)首先获取遥控器通道值
	channel_roll = get_channel_roll(); //range(-1,1)
	channel_yaw = get_channel_yaw(); //range(-1,1)
	channel_pitch = get_channel_pitch(); //range(-1,1)
	channel_throttle=get_channel_throttle();  //range(0-1)

  //(2)对遥控器通道值进行混控
	float left_head = channel_pitch+channel_roll+channel_yaw;
	float right_head = -channel_pitch+channel_roll+channel_yaw;
	float left_tail = -channel_pitch+channel_roll-channel_yaw;
	float right_tail = channel_pitch+channel_roll-channel_yaw;

	float max = abs(left_head);//取出最大值
	if (max < abs(right_head)){max = abs(right_head);}
	if (max < abs(left_tail)){max = abs(left_tail);}
	if (max < abs(right_tail)){max = abs(right_tail);}

  //(3)将混控值映射至PWM脉宽输出给电机
	if(get_soft_armed()&&(channel_throttle>0.1)){//解锁后,油门推起来才允许电机输出
		//电机1左前，用m1口控制电机1速度，用gpio1控制电机正反转
		if (left_head >= 0)
		{
			write_gpio1(true);//用gpio口控制电机1正反转
			Motor_Set_Value(1, left_head / max * PWM_BRUSH_MAX);//用m1 PWM口控制电机1转速
		}
		else
		{
			write_gpio1(false);
			Motor_Set_Value(1, abs(left_head) / max * PWM_BRUSH_MAX);
		}

			//电机2右前，用m5口控制电机2速度，用gpio5控制电机正反转
		if (right_head >= 0)
		{
			write_gpio5(true);
			Motor_Set_Value(5, right_head / max * PWM_BRUSH_MAX);
		}
		else
		{
			write_gpio5(false);
			Motor_Set_Value(5, abs(right_head) / max * PWM_BRUSH_MAX);
		}

			//电机3左后，用m2口控制电机3速度，用gpio2控制电机正反转
		if (left_tail >= 0)
		{
			write_gpio2(true);
			Motor_Set_Value(2, left_tail / max * PWM_BRUSH_MAX);
		}
		else
		{
			write_gpio2(false);
			Motor_Set_Value(2, abs(left_tail) / max * PWM_BRUSH_MAX);
		}

			//电机4右后，用m6口控制电机4速度，用gpio6控制电机正反转
		if (right_tail >= 0)
		{
			write_gpio6(true);
			Motor_Set_Value(6,  right_tail / max * PWM_BRUSH_MAX);
		}
		else
		{
			write_gpio6(false);
			Motor_Set_Value(6, abs(right_tail) / max * PWM_BRUSH_MAX);
		}
	}else{//未解锁时电机无输出
		Motor_Set_Value(1, PWM_BRUSH_MIN);
		Motor_Set_Value(2, PWM_BRUSH_MIN);
		Motor_Set_Value(5, PWM_BRUSH_MIN);
		Motor_Set_Value(6, PWM_BRUSH_MIN);
	}
}
