/*
 * demo_ekf.cpp
 *
 *  Created on: 2022年6月18日
 *      Author: 10427
 */
#include"maincontroller.h"

//Q表示观测数据的方差；R1表示预测数据的位置的方差；R2表示预测数据速度的方差
EKF_Baro *ekf_baro_1=new EKF_Baro(_dt, 0.0016, 1.0, 0.000016, 0.000016);

//Q表示观测数据的方差；R1表示预测数据的位置的方差；R2表示预测数据速度的方差
EKF_Rangefinder *ekf_rangefinder_1=new EKF_Rangefinder(_dt, 1.0, 0.000016, 0.16);

/*Q1表示观测数据的位置的x轴方差；Q2表示观测数据的位置的y轴方差；
 R1表示预测数据的位置的x轴方差；R2表示预测数据的速度的x轴方差；R3表示预测数据的位置的y轴方差；R4表示预测数据的速度的y轴方差*/
EKF_Odometry *ekf_odometry_1=new EKF_Odometry(_dt, 0.0016, 0.0016, 0.000016, 0.00016, 0.000016, 0.00016);

/*Q1表示观测数据的位置的x轴方差；Q2表示观测数据的速度的x轴方差；Q3表示观测数据的位置的y轴方差；Q4表示观测数据的速度的y轴方差；观测数据就是GPS的测量值
 R1表示预测数据的位置的x轴方差；R2表示预测数据的速度的x轴方差；R3表示预测数据的位置的y轴方差；R4表示预测数据的速度的y轴方差；预测数据跟加速度测量值相关*/
EKF_GNSS *ekf_gnss_1=new EKF_GNSS(_dt, 1.0, 1.0, 1.0, 1.0, 0.0000001, 0.001, 0.0000001, 0.001);

bool get_baro_alt_filt = false;
bool get_gnss_location = false;
bool get_odom_xy = false;
bool get_rangefinder_data = false;

void ekf_baro_demo()
{
	ekf_baro_1->update(get_baro_alt_filt, get_baroalt_filt());
}
void ekf_gnss_demo()
{
	ekf_gnss_1->update(get_gnss_location,get_ned_pos_x(),get_ned_pos_y(),get_ned_vel_x(),get_ned_vel_y());
}

void ekf_odometry_demo()
{
	ekf_odometry_1->update(get_odom_xy,get_odom_x(),get_odom_y());
}

void ekf_rf_demo()
{
	ekf_rangefinder_1->update(get_rangefinder_data, get_rangefinder_alt());
}
