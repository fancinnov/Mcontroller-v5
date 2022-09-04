/*
 * ekf_gnss.h
 *
 *  Created on: 2021年8月20日
 *      Author: 25053
 */
#pragma once

#ifndef INCLUDE_EKF_EKF_GNSS_H_
#define INCLUDE_EKF_EKF_GNSS_H_

#include "common.h"

class EKF_GNSS{
public:
	EKF_GNSS(float dt, float Q1, float Q2, float Q3, float Q4, float R1, float R2, float R3, float R4);
	void update(bool &get_gnss, float gnss_pos_x, float gnss_pos_y, float gnss_vel_x, float gnss_vel_y);
	bool is_initialed(void){return initialed;}
	void reset(void){
		initialed=false;
	}
	float pos_x=0, pos_y=0, vel_x=0, vel_y=0;

private:
	float _filt_alpha(float dt, float filt_hz);
	float _alpha=0;
	float Qt[4]={1.0f,1.0f,1.0f,1.0f};//观测数据的方差
	float T_odom=0.0025; //2.5ms
	bool initialed=false;
	/*****************x-axis******************/
	float delta_x_odomx=0, delta_v_velx=0;
	float G_odomx[2*2]={ 1, T_odom,
						 0, 	1};
	float GT_odomx[2*2]={1,	 0,
			 	 	 	 T_odom, 1};
	float h_odomx=0 ,error_x_odomx=1, zt_odomx=0;
	float h_velx=0 ,error_v_velx=1, zt_velx=0;
	float H_odomx[2*2]={1,0,
						0,1};
	float HT_odomx[2*2]={1,0,
						 0,1};
	float Rt_odomx[2*2]={ 0.0000001,   0, 		//预测数据x方差
						  0,        0.001}; 	//预测数据v方差
	float error_xv_x[2*2]={1,0,
			 	 	 	   0,1};
	float error_odomx[2*2]={ 1.0,   0,
						     0,     1.0};
	float error_p_odomx[2*2];
	float* error1_odomx;
	float* error2_odomx;
	float* Kal_odomx;
	float inverse_x[4]={};
	float accelx_filt=0;
	/*****************y-axis******************/
	float delta_x_odomy=0, delta_v_vely=0;
	float G_odomy[2*2]={ 1, T_odom,
						 0, 	1};
	float GT_odomy[2*2]={1,	      0,
			 	 	 	 T_odom, 1};
	float h_odomy=0 ,error_x_odomy=1, zt_odomy=0;
	float h_vely=0 ,error_v_vely=1, zt_vely=0;
	float H_odomy[2*2]={1,0,
						0,1};
	float HT_odomy[2*2]={1,0,
						 0,1};
	float Rt_odomy[2*2]={ 0.0000001,   0, 	//预测数据x方差
						  0,        0.001}; //预测数据v方差
	float error_xv_y[2*2]={1,0,
				 	 	   0,1};
	float error_odomy[2*2]={ 1.0,  0,
						     0,    1.0};
	float error_p_odomy[2*2];
	float* error1_odomy;
	float* error2_odomy;
	float* Kal_odomy;
	float inverse_y[4]={};
	float accely_filt=0;
	float accelxy_filt_hz=10;
};

#endif /* INCLUDE_EKF_EKF_GNSS_H_ */
