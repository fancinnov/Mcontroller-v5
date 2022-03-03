/*
 * ekf_odom.h
 *
 *  Created on: 2021年7月9日
 *      Author: 25053
 */
#pragma once

#ifndef INCLUDE_EKF_EKF_ODOMETRY_H_
#define INCLUDE_EKF_EKF_ODOMETRY_H_

#include "common.h"

class EKF_Odometry{
public:
	EKF_Odometry(float dt, float Q1, float Q2, float R1, float R2, float R3, float R4);
	void update(bool &get_odometry, float odom_pos_x, float odom_pos_y);
	bool is_initialed(void){return initialed;}
	void reset(void){
		initialed=false;
	}
	float pos_x=0, pos_y=0, vel_x=0, vel_y=0;

private:
	float Qt[2]={1.0f,1.0f}; //观测数据的方差
	float T_odom=0.0025; //2.5ms
	bool initialed=false;
	/*****************x-axis******************/
	float delta_x_odomx=0;
	float G_odomx[2*2]={ 1, T_odom,
						 0, 	1};
	float GT_odomx[2*2]={1,	 0,
			 	 	 	 T_odom, 1};
	float h_odomx=0 ,error_x_odomx=0, zt_odomx=0;
	float H_odomx[1*2]={1,0};
	float HT_odomx[2*1]={1,0};
	float Rt_odomx[2*2]={ 0.0025,   0, 		//预测数据x方差
						  0,        2.5};   //预测数据v方差
	float error_odomx[2*2]={ 1.0,   0,
						     0,     1.0};
	float error_p_odomx[2*2];
	float* error1_odomx;
	float* error2_odomx;
	float* Kal_odomx;

	/*****************y-axis******************/
	float delta_x_odomy=0;
	float G_odomy[2*2]={ 1, T_odom,
						 0, 	1};
	float GT_odomy[2*2]={1,	      0,
			 	 	 	 T_odom, 1};
	float h_odomy=0 ,error_x_odomy=0, zt_odomy=0;
	float H_odomy[1*2]={1,0};
	float HT_odomy[2*1]={1,0};
	float Rt_odomy[2*2]={ 0.0025,   0, 		//预测数据x方差
						  0,        2.5}; 	//预测数据v方差
	float error_odomy[2*2]={ 1.0,     0,
						     0,       1.0};
	float error_p_odomy[2*2];
	float* error1_odomy;
	float* error2_odomy;
	float* Kal_odomy;
};

#endif /* INCLUDE_EKF_EKF_ODOMETRY_H_ */
