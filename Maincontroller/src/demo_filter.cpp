/*
 * demo_filter.cpp
 *
 *  Created on: 2022年6月16日
 *      Author: 10427
 */
#include "maincontroller.h"

LowPassFilterFloat throttle; //这里以float类型为例说明，可根据不同需求选择int类型，long类型，Vector2f类型以及Vector3f类型。
LowPassFilter2pFloat throttle_2p;

float throttle_in,throtte_out;
float throttle_in_2p, throttle_out_2p;

bool filter_init() // 运行一次，设置滤波器的采样频率以及截止频率。
{
	throttle.set_cutoff_frequency(400, 1); //两个参数分别为采样频率(Hz)，截至频率(Hz)。采样频率的设置决定demo函数的循环频率。
	throttle.reset(0);
	return true;
}
bool filter_2p_init() // 运行一次，设置滤波器的采样频率以及截止频率。
{
	throttle_2p.set_cutoff_frequency(400, 1);//两个参数分别为采样频率(Hz)，截至频率(Hz)。采样频率的设置决定demo函数的循环频率。
	throttle_2p.reset();
	return true;
}
void filter_demo() //循环运行，输入是采样值，输出是滤波后的油门。循环频率与采样频率一致。
{
	throtte_out = throttle.apply(throttle_in);
}

void filter_2p_demo() //循环运行，输入是采样值，输出是滤波后的油门。循环频率与采样频率一致。
{
	throttle_out_2p = throttle_2p.apply(throttle_in_2p);
}


