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
	void update(bool use_mag, bool get_mav_yaw);
	bool is_initialed(void){return initialed;}
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

	float yaw_rad_record=0;
	float mag_correct=0.0f;

	Quaternion quaternion=Quaternion();

	float P_posteriori[4*4]={ 0.125, 0.0003, 0.0003, 0.0003,
							 0.0003, 0.125, 0.0003, 0.0003,
							 0.0003, 0.0003, 0.125, 0.0003,
							 0.0003, 0.0003, 0.0003, 0.125 };
	float e6 = 1e6, P_priori[4*4], *tempP1, *tempP2, *KGain;
	float Q_PredNoise[4*4] = {  e6, 0, 0, 0,
								0, e6, 0, 0,
								0, 0, e6, 0,
								0, 0, 0, e6};
	float q_priori[4], q_priori_length;
	float Rk1[3*3] = { 2, 0, 0,
					   0, 2, 0,
					   0, 0, 2};
	float Rk2[3*3] = {  1, 0, 0,
						0, 1, 0,
						0, 0, 1};

	float ekf_gain=0.005;

	Vector3f accel_average, accel_variance;
	Vector3f accel_array[var_length];
};

#endif /* INCLUDE_AHRS_AHRS_H_ */
