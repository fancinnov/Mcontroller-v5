/*
 * ahrs.h
 *
 *  Created on: 2021年7月9日
 *      Author: JackyPan
 */
#pragma once

#ifndef INCLUDE_AHRS_AHRS_H_
#define INCLUDE_AHRS_AHRS_H_

#include "common.h"

#define var_length 20 //the length of variance window

class AHRS{

public:
	AHRS(float dt);
	void update(bool &get_mag, bool &get_mav_yaw);
	bool is_initialed(void){return initialed;}
	void set_declination(float rad){declination=rad;}
	float get_declination(void){return declination;}
	void reset(void){
		initialed=false;
	}
	//AHRS输出quaternion2
	Quaternion quaternion2=Quaternion();
	void check_vibration(void);
	float vib_value=0, vib_angle=0;
private:
	float T=0.0025;

	bool initialed=false;

	Quaternion quaternion=Quaternion();

	float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;
	float wx=0.0f, wy=0.0f, wz=0.0f;
	Vector3f accel;
	float accel_length=0.0f;
	float accel_error_gain=1.0f;
	float ax=0.0f, ay=0.0f, az=0.0f;
	float Ak[4*4]={0};
	float AkT[4*4]={0};
	float temp33[3*3]={0}, temp33inv[3*3]={0};
	float q_factor1[4]={0};
	float Hk1[3*4]={0};
	float Hk1T[4*3]={0};
	float zk1_hk1[3*1]={0};
	float mx=1.0f, my=0.0f, mz=0.0f;
	float q_factor2[4]={0};
	float Hk2[3*4]={0};
	float Hk2T[4*3]={0};
	float zk2_hk2[3*1]={0};

	float P_posteriori[4*4]={ 0.125, 0.0003, 0.0003, 0.0003,
							 0.0003, 0.125, 0.0003, 0.0003,
							 0.0003, 0.0003, 0.125, 0.0003,
							 0.0003, 0.0003, 0.0003, 0.125 };
	float e6 = 1e6, P_priori[4*4], *tempP1, *tempP2, *KGain;
	float Q_PredNoise[4*4] = {  e6, 0, 0, 0,
								0, e6, 0, 0,
								0, 0, e6, 0,
								0, 0, 0, e6};
	float q_priori[4]={0}, q_priori_length=1.0f;
	float Rk1[3*3] = { 0.8, 0, 0,
					   0, 0.8, 0,
					   0, 0, 0.8};
	float Rk2[3*3] = {  0.4, 0, 0,
						0, 0.4, 0,
						0, 0, 0.4};

	float ekf_gain;
	float declination=0.0f;
	Vector2f mag_ef_2d;
	Vector3f mag_bf;

	Vector3f accel_average, accel_variance;
	Vector3f accel_array[var_length];
};

#endif /* INCLUDE_AHRS_AHRS_H_ */
