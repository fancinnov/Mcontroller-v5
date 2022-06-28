/*
 * demo_pid.cpp
 *
 *  Created on: 2022年6月16日
 *      Author: 10427
 */
#include "maincontroller.h"

PID *pid = new PID(0.1, 0.1, 0.1, 1, 20, 0.0025); //定义PID对象 参数分别是p；i；d；积分结果最大值；输入error的低通滤波截止频率(Hz)；pid运行周期(s)
PID *accel_pid = new PID(0.1, 0.1, 0.1, 1, 20, 0.0025);
P *p_pos = new P(0.1);// 定义P对象 参数为p
P *v_pos = new P(0.1);
PID_2D *pid_2d = new PID_2D(0.1,0.1,0.2,0.2,0.3,0.3,1,20,30,0.0025);

float demo_pid_single(float target, float current) //单环pid
{
	float error = target - current; //误差 = 目标值-当前值

	pid->set_input_filter_all(error);

	return pid->get_p() + pid->get_i() + pid->get_d();

}

float demo_pid_poscontrol(float target, float current1, float current2, float current3) //串级pid（3环）
{
	float error1 = target - current1;//最外环
	float p_result1 = p_pos->get_p(error1);


	float error2 = p_result1 - current2;//中间环
	float p_result2 = v_pos->get_p(error2);

	float error3 = p_result2 - current3;// 最内环
	accel_pid->set_input_filter_all(error3);

	return accel_pid->get_pid();

}

Vector2f pid_2d_demo(Vector2f &target, Vector2f &current) //二维pid
{
	Vector2f error;
	error.x = target.x - current.x;
	error.y = target.y - current.y;

	pid_2d->set_input(error);

	return pid_2d->get_pid();

}
