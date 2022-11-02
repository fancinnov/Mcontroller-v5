/*
 * config.h
 *
 *  Created on: 2021年7月12日
 *      Author: JackyPan
 */

#pragma once

#ifndef INCLUDE_CONFIG_H_
#define INCLUDE_CONFIG_H_

#include "hal.h"

/**** 数字舵机PWM频率为320HZ ****
 **** 模拟舵机PWM频率为50HZ  ****
 **** 电机PWM频率为400HZ    ****/
#define PWM_MOTOR_FREQ 400
#define PWM_DIGITAL_SERVO_FREQ 320
#define PWM_ANALOG_SERVO_FREQ 50

#define PWM_SERVO_MIN 500   // 脉宽 500us
#define PWM_SERVO_MAX 2500  // 脉宽 2500us

#define PWM_ESC_MIN 1000	// 脉宽 1000us
#define PWM_ESC_MAX 2000	// 脉宽 2000us
#define PWM_ESC_SPIN_ARM 1100
#define PWM_ESC_SPIN_MIN 1100
#define PWM_ESC_SPIN_MAX 1950

#define PWM_BRUSH_MIN 0		// 脉宽 0us
#define PWM_BRUSH_MAX 2500	// 脉宽 2500us
#define PWM_BRUSH_SPIN_ARM 100
#define PWM_BRUSH_SPIN_MIN 100
#define PWM_BRUSH_SPIN_MAX 2400

/**************遥控信号输入**************/
#define RC_INPUT_PPM	0   		//PPM信号
#define RC_INPUT_SBUS	1  			//SBUS信号

//配置遥控通道数
#define RC_INPUT_CHANNELS  	8       //8 RC input channel (value:1000~2000)

//遥控器输入脉宽有效值范围
#define RC_INPUT_MIN 	1000.0f // 脉宽 1000us
#define RC_INPUT_MAX 	2000.0f // 脉宽 2000us
#define RC_INPUT_MID 	1500.0f // 脉宽 1500us
#define RC_INPUT_RANGE 	1000.0f // 脉宽 1000us

//串口模式
#define DEV_COMM 	0x01 //自定义模式
#define MAV_COMM  	0x02 //Mavlink模式
#define GPS_COMM  	0x03 //GPS模式
#define TFMINI_COMM 0X04 //TFmini激光测距仪
#define LC302_COMM 0X05 //LC302光流

/***************usb+串口配置****************
 * *************COMM_0:USB口***************
 * *************COMM_1:Serial串口1**********
 * *************COMM_2:Serial串口2**********
 * *************COMM_3:Serial串口3**********
 * *************COMM_4:Serial串口4**********
 * @param:
 * COMM_0~COMM_4可以配置为下列可选参数,参数及其含义如下：
 * (1)DEV_COMM 		自定义模式
 * (2)MAV_COMM  	Mavlink模式
 * (3)GPS_COMM  	GPS模式
 * (4)TFMINI_COMM  	TFmini激光测距仪
 * (5)LC302_COMM	LC302光流模块
 * **************************************/
#define COMM_0 MAV_COMM
#define COMM_1 GPS_COMM
#define COMM_2 MAV_COMM
#define COMM_3 TFMINI_COMM
#define COMM_4 MAV_COMM

//配置LED
#define FMU_LED_CONTROLL_ENABLE 1 // if use system default led control, set 1; if you want to control led by yourself, set 0;

//配置磁罗盘
#define USE_MAG 1 // if use mag, set 1; if not use mag, set 0;

//配置GPS
#define USE_GPS 0 // if use gps, set 1; if not use gps, set 0;

//配置UWB
#define USE_UWB 1 // if use uwb, set 1; if don't use uwb, set 0;

//配置光流
#define USE_FLOW 1 // if use optical flow, set 1; if don't use optical flow, set 0;

//配置里程计
#define USE_ODOMETRY 0 // if use odometry, set 1; if don't use odometry, set 0;

//配置flash
#define USE_FRAM 2 //保持默认值,请勿更改

#if USE_FRAM==1
	#define ADDR_FLASH        ((uint16_t)0x0000)
	#define ADDR_FLASH_DATA   ((uint16_t)0x0002) /* the beginning of the real data package */
	#define DATA_FLASH_LENGTH ((uint16_t)0x0020) /* each data package takes 32 bytes */
#elif USE_FRAM==2
	#define ADDR_FLASH        ((uint16_t)0x0000)
	#define ADDR_FLASH_DATA   ((uint16_t)0x0020) /* the beginning of the real data package */
	#define DATA_FLASH_LENGTH ((uint16_t)0x0020) /* each data package takes 32 bytes */
#else
	#define ADDR_FLASH        ((uint32_t)0x081D0000) /* bank2, Sector22: 128KB | 0x081D0000 - 0x081EFFFF, 1地址表示1byte */
	/* the Sector22 must be the number of all data package,
	 * so we can get how many packages were saved in the flash by reading Sector22*/
	#define ADDR_FLASH_DATA   ((uint32_t)0x081E0000) /* bank2, Sector23: 128KB | 0x081E0000 - 0x081FFFFF, the beginning of the real data package */
	#define DATA_FLASH_LENGTH ((uint32_t)0x00000020) /* each data package takes 32 bytes */
#endif

#define VERSION_HARDWARE 573
#define VERSION_FIRMWARE 2022110101

#endif /* INCLUDE_CONFIG_H_ */
