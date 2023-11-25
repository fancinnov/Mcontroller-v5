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
static float control_pitch = 0.0f;

extern float t_pitch_rate;
extern float ctrl_pitch;

float target_pitch_rate = 0;
float target_yaw_rate = 0;


float AA = 0.0;//代表四个电机
float BB = 0.0;
float CC = 0.0;
float DD = 0.0;

//设置的目标点
float target_x = 100;
float target_y = 100;
P *p_position = new P(1);

float get_distance_cos(float x_current, float y_current, float target_x, float target_y)
{
	float distance = sqrt(pow(x_current - target_x, 2) + pow(y_current - target_y, 2));
	float deltaX = target_x - x_current;
	float deltaY = target_y - y_current;
	// 计算夹角的cos值
	float cosTheta = deltaY / sqrt(deltaX * deltaX + deltaY * deltaY);


	float distance_cos = distance * cosTheta;
	return distance_cos;
}
float get_distance(float x_current, float y_current, float target_x, float target_y)
{
	float distance = sqrt(pow(x_current - target_x, 2) + pow(y_current - target_y, 2));

	return distance;
}
float calculateAngle(float x1, float y1, float x2, float y2)
{
	float deltaX = x2 - x1;
	float deltaY = y2 - y1;

    // 计算夹角的cos值
	float cosTheta = deltaY / sqrt(deltaX * deltaX + deltaY * deltaY);

    // 用acos得到夹角的弧度值
	float angleRadians = acos(cosTheta);

    return angleRadians;
}

void control_car(float channel_pitch, float channel_yaw)
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



float get_desired_pitch_rate(float channel_pitch)
{
	if(channel_pitch <= 0.1 && channel_pitch >= -0.1)
	{
		channel_pitch = 0.0f;
		attitude->get_throttle_speed_pid().reset_I();
	}

	return channel_pitch * 1.0;
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

	target_yaw = ahrs_yaw_deg();

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
		 //位置环添加
		 float distance = get_distance(get_pos_x(), get_pos_y(), target_x, target_y);
		 float pos_error = get_distance_cos(get_pos_x(), get_pos_y(), target_x, target_y);
		 float control_pos = p_position->get_p(pos_error);

					if(pos_error >= 0 && distance > 20)
					{


//						 float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());//获取摇杆设置的目标偏航角速度
//						 target_yaw+=target_yaw_rate*_dt;
						 target_yaw = calculateAngle(get_pos_x(), get_pos_y(), target_x, target_y);
						 control_yaw = attitude->get_steering_out_heading(target_yaw*DEG_TO_RAD, M_PI/2, false, false, _dt);

//
//						 //20230616 速度环添加
//						 float target_pitch_rate = get_desired_pitch_rate(get_channel_pitch());
//						 t_pitch_rate = target_pitch_rate;
						 control_pitch = attitude->get_throttle_out_speed(control_pos, false, false, 2.0, 1.0, _dt);
//						 ctrl_pitch = control_pitch;
//
//						 usb_printf("target_pitch_rate:%f control_pitch:%f\n",target_pitch_rate,control_pitch);



						 AA = (-control_pitch + control_yaw) * 2500;
						 BB = (-control_pitch + control_yaw) * 2500;
						 CC = (-control_pitch - control_yaw) * 2500;
						 DD = (-control_pitch - control_yaw) * 2500;

					}
					else if(pos_error < 0 && distance > 20)
					{

//						float target_yaw_rate = -get_pilot_desired_yaw_rate(get_channel_yaw_angle());//获取摇杆设置的目标偏航角速度
//						target_yaw+=target_yaw_rate*_dt;
						target_yaw = calculateAngle(get_pos_x(), get_pos_y(), target_x, target_y);
						control_yaw = attitude->get_steering_out_heading(target_yaw*DEG_TO_RAD, M_PI/2, false, false, _dt);

//						//20230616 速度环
//						float target_pitch_rate = get_desired_pitch_rate(get_channel_pitch());
//						t_pitch_rate = target_pitch_rate;
						control_pitch = attitude->get_throttle_out_speed(control_pos, false, false, 2.0, 1.0, _dt);
//						ctrl_pitch = control_pitch;
//
//						usb_printf("target_pitch_rate:%f control_pitch:%f\n",target_pitch_rate,control_pitch);


						AA = (-control_pitch + control_yaw) * 2500;
						BB = (-control_pitch + control_yaw) * 2500;
						CC = (-control_pitch - control_yaw) * 2500;
						DD = (-control_pitch - control_yaw) * 2500;

					}
					else
					{
						control_pitch = 0;
						channel_pitch = 0;
						attitude->get_throttle_speed_pid().reset_I();

						float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());//获取摇杆设置的目标偏航角速度
						//target_yaw = M_PI;
						target_yaw+=target_yaw_rate*_dt;
//						control_yaw = attitude->get_steering_out_heading(target_yaw*DEG_TO_RAD, M_PI/2, false, false, _dt);
						control_yaw = 0;
						//attitude->get_throttle_speed_pid().reset_I();

						AA = (-channel_pitch + control_yaw) * 2500;
						BB = (-channel_pitch + control_yaw) * 2500;
						CC = (-channel_pitch - control_yaw) * 2500;
						DD = (-channel_pitch - control_yaw) * 2500;

//						usb_printf("target_pitch_rate:%f control_pitch:%f\n",target_pitch_rate,control_pitch);

					}





					usb_printf("target_yaw_rate:%f control_yaw:%f\n",target_yaw_rate,control_yaw);


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

//		float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());//获取摇杆设置的目标偏航角速度
//		//target_yaw = M_PI;
//		target_yaw+=target_yaw_rate*_dt;
//		control_yaw = attitude->get_steering_out_heading(target_yaw*DEG_TO_RAD, M_PI/2, false, false, _dt);
//		//usb_printf("control_yaw:%f\n",control_yaw);
//		control_car(channel_pitch, control_yaw);


	}
	else
	{
		target_yaw = ahrs_yaw_deg();
	}

}
