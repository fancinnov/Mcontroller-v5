/*
 * ekf_rangfinder.h
 *
 *  Created on: 2021年7月9日
 *      Author: JackyPan
 */
#pragma once

#ifndef INCLUDE_EKF_EKF_RANGEFINDER_H_
#define INCLUDE_EKF_EKF_RANGEFINDER_H_

#include "common.h"

class EKF_Rangefinder{

public:
	EKF_Rangefinder(float dt, float Q, float R1, float R2);
	void update(bool &get_rangefinder_data, float rangefinder_alt);
	bool is_initialed(void){return initialed;}
	void reset(void){
		initialed=false;
	}
	float pos_z=0, vel_z=0;

private:
	float Qt=1.0f; //观测数据的方差
	bool initialed=false;
	float _filt_alpha(float dt, float filt_hz);
	float _alpha=0;
	float delta_x_rf=0;
	float T_rangefinder=0.0025; //2.5ms
	float G_rf[2*2]={ 1, T_rangefinder,
				   0, 	1};
	float GT_rf[2*2]={1,	 0,
			T_rangefinder, 1};
	float h_rf=0 ,error_x_rf=0, zt_rf=0;
	float H_rf[1*2]={1,0};
	float HT_rf[2*1]={1,0};
	float Rt_rf[2*2]={ 0.000016,          0, 	//预测数据x方差
						0,              0.16}; 	//预测数据v方差
	float error_rf[2*2]={ 1.0,       0,
						  0,         1.0};
	float error_p_rf[2*2];
	float* error1_rf;
	float* error2_rf;
	float* Kal_rf;
	float accelz_filt_hz_rf=30;//Hz
	float accelz_filt_rf=0;
};

#endif /* INCLUDE_EKF_EKF_RANGEFINDER_H_ */
