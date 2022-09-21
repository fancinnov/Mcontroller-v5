/*
 * ekf_baro.h
 *
 *  Created on: 2021年7月9日
 *      Author: JackyPan
 */
#pragma once

#ifndef INCLUDE_EKF_EKF_BARO_H_
#define INCLUDE_EKF_EKF_BARO_H_

#include "common.h"

class EKF_Baro{

public:
	EKF_Baro(float dt, float Q1, float Q2, float R1, float R2);
	void update(bool &get_baro_alt_filt, float baro_alt);
	float pos_z=0, vel_z=0;
	bool is_initialed(void){return initialed;}
	void reset(void){
		initialed=false;
	}

private:
	float Qt[2*2]={0.0016, 		0,
					0,     0.0016};//观测数据的方差
	float _filt_alpha(float dt, float filt_hz);
	float _alpha=0;
	bool initialed=false;
	float delta_x=0, delta_v=0;
	float T_baro=0.0025; //2.5ms
	float G[2*2]={ 1, T_baro,
				   0, 	1};
	float GT[2*2]={1,	   0,
				   T_baro, 1};
	float h_x=0, h_v=0, zt=0, zt_last=0, vt=0, t_now=0, t_last=0;
	float error_x, error_v;
	float error_xv[2*2]={1,0,
				 	 	 0,1};
	float inv[4];
	float H[2*2]={1,0,
				  0,1};
	float HT[2*2]={1,0,
					0,1};
	float Rt[2*2]={ 0.000016,       0,	//预测数据x方差
					0,          0.000016};	//预测数据v方差
	float error[2*2]={  1.0,        0,
					    0,          1.0};
	float error_p[2*2];
	float* error1;
	float* error2;
	float* Kal;
	float accelz_filt_hz=10;//Hz 震动对于速度预测影响非常大 所以要把截止频率设低一些
	float accelz_filt=0;
	DerivativeFilterFloat_Size7 _climb_rate_filter;
};
#endif /* INCLUDE_EKF_EKF_BARO_H_ */
