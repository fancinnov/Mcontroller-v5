/*
 * hal.h
 *
 *  Created on: 2020.06.18
 *      Author: JackyPan
 *
 *  NOTED!
 *  All of the codes in this project(no matter any version, modified or not) can only be used exclusively on Mcontroller manufactured by Fancinnov,
 *  or you will bear legal liability.
 */
#pragma once
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_H
#define __HAL_H

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <float.h>
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usart.h"
#include "adc.h"
#include "spi.h"
#include "common/mavlink.h"
#include "usbd_cdc_if.h"
#include "define.h"
#include "ringbuffer.h"
#include "config.h"
#include "gnss/gps.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONSTRAIN(in, min, max)  (in > max ? max : (in < min ? min : in))
#define ABS(a) ((a) > 0 ? (a) : -(a))

#define MPU_CS_H     HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET)
#define MPU_CS_L     HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET)
#define BARO_CS_H     HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET)
#define BARO_CS_L     HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET)

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
} Vector3_Int16;

typedef struct{
	float x;
	float y;
	float z;
}Vector3_Float;

typedef struct{
	Vector3_Int16 acc_raw;
	Vector3_Int16 gyro_raw;
	Vector3_Float accf;			//m/s/s
	Vector3_Float gyrof;		//°/s
	int16_t temp;   //temperature
}MPU6000_Data;

typedef struct{
	Vector3_Int16 acc_raw;
	Vector3_Int16 gyro_raw;
	Vector3_Int16 mag_raw;
	Vector3_Float accf;			//m/s/s
	Vector3_Float gyrof;		//°/s
	Vector3_Float magf;			//uT
	int16_t temp;   			//temperature
}MPU9250_Data;

typedef struct{
	Vector3_Int16 acc_raw;
	Vector3_Int16 gyro_raw;
	Vector3_Int16 mag_raw;
	Vector3_Float accf;			//m/s/s
	Vector3_Float gyrof;		//°/s
	Vector3_Float magf;			//uT
	int16_t temp;   			//temperature
}ICM20608_Data;

typedef struct{
	Vector3_Int16 mag_raw;
	Vector3_Float magf;			//guass
	int16_t temp;   			//temperature
}QMC5883_Data;

typedef struct{
	Vector3_Int16 acc_raw;
	Vector3_Int16 mag_raw;
	Vector3_Float accf;			//m/s/s
	Vector3_Float magf;			//guass
	int16_t temp;   //temperature
}LSM303D_Data;

typedef struct{
	Vector3_Int16 gyro_raw;
	Vector3_Float gyrof;		//°/s
	int16_t temp;   //temperature
}L3GD20_Data;

typedef struct {
	float temp;					//℃
	float baro_alt; 			//m
} MS5611_Data;

typedef struct {
	float temp;					//℃
	float baro_alt; 			//m
} MS5607_Data;

typedef struct {
	float temp;					//℃
	float baro_alt; 			//m
} SPL06_Data;

typedef struct {
	uint16_t motor[8];  // ESC:1000~2000; BRUSH:0~2500
	uint16_t servo[4];  // SERVO:500~2500
} PWM_Channel;

typedef enum {
	BUZZER_NONE=-1,
	BUZZER_IDLE,
	BUZZER_INITIALED,
	BUZZER_INITIALED_1,
	BUZZER_INITIALED_2,
	BUZZER_ERROR,
	BUZZER_ERROR_1,
	BUZZER_ERROR_2,
	BUZZER_ARMED,
	BUZZER_DISARM,
}BUZZER_TYPE;

typedef enum ROBOT_TYPE{
	UAV_8_H,
	UAV_4_X,
	UGV_4,
	UGV_2,
	SPIDER_6,
	USER_DEF
}ROBOT_TYPE;

typedef enum MOTOR_TYPE{
	ESC,
	BRUSH,
	SERVO,
}MOTOR_TYPE;

typedef enum {
	STATE_FLY=0,
	STATE_CLIMB=1,
	STATE_DRIVE=2
}ROBOT_STATE;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern MPU6000_Data mpu6000_data;
extern MPU9250_Data mpu9250_data;
extern ICM20608_Data icm20608_data;
extern QMC5883_Data qmc5883_data;
extern LSM303D_Data lsm303d_data;
extern L3GD20_Data l3gd20_data;
extern MS5607_Data ms5607_data;
extern SPL06_Data spl06_data;
extern MS5611_Data ms5611_data;
extern PWM_Channel pwm_channel;
extern ROBOT_STATE robot_state;

/****************c/c++ interface*******************************/
void motors_init(void);
void attitude_init(void);
void pos_init(void);
void update_accel_gyro_data(void);
void update_mag_data(void);
void update_baro_alt(void);
bool gyro_calibrate(void);
void ahrs_update(void);
void gnss_update(void);
bool uwb_init(void);
void uwb_update(void);
void uwb_position_update(void);
void ekf_baro_alt(void);
void ekf_rf_alt(void);
void ekf_odom_xy(void);
void ekf_gnss_xy(void);
void update_throttle_hover(void);
void rate_controller_run(void);
void motors_output(void);
void update_land_detector(void);
void throttle_loop(void);
void get_tfmini_data(uint8_t buf);
void parse_mavlink_data(mavlink_channel_t chan, uint8_t data, mavlink_message_t* msg_received, mavlink_status_t* status);
void send_mavlink_data(mavlink_channel_t chan);
void send_mavlink_param_list(mavlink_channel_t chan);
void send_mavlink_heartbeat_data(void);
void Logger_Update(void);
void reset_dataflash(void);
void update_dataflash(void);
void lock_motors_check(void);
void arm_motors_check(void);
bool mode_althold_init(void);
void mode_althold(void);
bool mode_stabilize_init(void);
void mode_stabilize(void);
bool mode_autonav_init(void);
void mode_autonav(void);
bool mode_poshold_init(void);
void mode_poshold(void);
void parse_comm0_data(uint8_t data);
void parse_comm1_data(uint8_t data);
void parse_comm2_data(uint8_t data);
void parse_comm3_data(uint8_t data);
void parse_comm4_data(uint8_t data);
void debug(void);
/****************c/c++ interface*******************************/
bool get_task_initialed(void);

void comm_callback(void);
void COMM1_Callback(void);//串口1中断回调函数
void COMM2_Callback(void);//串口2中断回调函数
void COMM3_Callback(void);//串口3中断回调函数
void COMM4_Callback(void);//串口4中断回调函数

void TIM_400HZ_Callback(void);//400HZ定时器中断回调函数
void TIM_200HZ_Callback(void);//200HZ定时器中断回调函数
void TIM_100HZ_Callback(void);//100HZ定时器中断回调函数
void TIM_50HZ_Callback(void);//50HZ定时器中断回调函数

/**
  * @brief  控制LED灯亮灭
  * @param  status: true/false
  * 		true 亮，false 灭
  */
void Led1_Control(bool status);
void Led2_Control(bool status);
void Led3_Control(bool status);
void Led4_Control(bool status);
void Led5_Control(bool status);
void Led6_Control(bool status);
void Led7_Control(bool status);

#if  FMU_LED_CONTROLL_ENABLE==1
#define 	FMU_LED1_Control(status)  Led1_Control(status)
#define 	FMU_LED2_Control(status)  Led2_Control(status)
#define 	FMU_LED3_Control(status)  Led3_Control(status)
#define 	FMU_LED4_Control(status)  Led4_Control(status)
#define 	FMU_LED5_Control(status)  Led5_Control(status)
#define 	FMU_LED6_Control(status)  Led6_Control(status)
#define 	FMU_LED7_Control(status)  Led7_Control(status)
#else
#define     FMU_LED1_Control(status)
#define     FMU_LED2_Control(status)
#define     FMU_LED3_Control(status)
#define     FMU_LED4_Control(status)
#define     FMU_LED5_Control(status)
#define     FMU_LED6_Control(status)
#define     FMU_LED7_Control(status)
#endif

void Buzzer_Set_Output_Enable(void);//启动蜂鸣音
void Buzzer_Set_Output_Disable(void);//关闭蜂鸣音
void Buzzer_set_Output_freq(uint16_t freq);//设置蜂鸣音频率 ,freq单位HZ
void Buzzer_set_ring_type(BUZZER_TYPE ring_type);//启动系统提示音
BUZZER_TYPE Buzzer_get_ring_type(void);//获取系统提示音状态
void Buzzer_ring(void);//启动系统提示音调度函数

//i2c驱动
/****************************
 * @param
 *  ClockSpeed: 时钟速率, 范围100khz~400Khz
 * 			100000~400000
 * 	Address： Mcontroller本机地址, 范围0~127
 * 			0x00~0x7F
 * **************************/
void Set_I2C_Mode(uint32_t ClockSpeed, uint32_t Address);
/******* Blocking mode *******/
HAL_StatusTypeDef I2C_Master_Transmit_Delayms(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_Master_Receive_Delayms(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_Slave_Transmit_Delayms(uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_Slave_Receive_Delayms(uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_Mem_Write_Delayms(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_Mem_Read_Delayms(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_IsDeviceReady_Delayms(uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
/******* Non-Blocking mode *******/
HAL_StatusTypeDef I2C_Master_Transmit_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef I2C_Master_Receive_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef I2C_Slave_Transmit_IT(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef I2C_Slave_Receive_IT(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef I2C_Mem_Write_IT(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef I2C_Mem_Read_IT(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_I2C_StateTypeDef I2C_GetState(void);

//spi驱动, Mcontroller为主机
/***************************************************
 * @param
 *  FirstBit: 大端在前或小端在前
 *  		SPI_FIRSTBIT_MSB / SPI_FIRSTBIT_LSB
 *  Prescaler: 波特率分频,SPI时钟频率=84/X Mhz,
 *  		SPI_BAUDRATEPRESCALER_X (X=2/4/8/16/32/64/128/256)
 *  CPol: 空闲时刻的时钟电平
 *  		SPI_POLARITY_LOW / SPI_POLARITY_HIGH
 *  CPha: 时钟触发边沿，第一边或第二边
 *  		SPI_PHASE_1EDGE / SPI_PHASE_2EDGE
 * *************************************************/
void Set_SPI_Mode(uint32_t FirstBit, uint32_t Prescaler, uint32_t CPol, uint32_t CPha);
void Set_SPI_NS_Pin(bool set);
/******* Blocking mode *******/
HAL_StatusTypeDef SPI_Transmit_Delayms(uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef SPI_Receive_Delayms(uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef SPI_TransmitReceive_Delayms(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
/******* Non-Blocking mode *******/
HAL_StatusTypeDef SPI_Transmit_IT(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI_Receive_IT(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI_TransmitReceive_IT(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_SPI_StateTypeDef SPI_GetState(void);

//IMU驱动
void IMU_Init(void);
void IMU_Request_Data(void);
void IMU_Get_Data(void);

//磁罗盘驱动
void MAG_Init(void);
void MAG_Get_Data(void);

//气压计驱动
uint8_t BARO_Init(void);
void BARO_Get_Date(void);

/***fram驱动函数为底层驱动，它的上层函数在Cpplibrary中的flash.h***/
void FRAM_Init(void);//FRAM 初始化
uint8_t FRAM_Get_Status(void);//获取fram状态
void FRAM_Write_Data(uint32_t addr, uint8_t *data, uint8_t length);
void FRAM_Read_Data(uint32_t addr, uint8_t *data, uint8_t length);

/***flash驱动函数为底层驱动，它的上层函数在Cpplibrary中的flash.h***/
/* @brief: write data into flash
 * @param:
 * 	addr: 	  the address of the data package
 * 	data:     the pointer of the data package
 * 	length:   the length of the data package
 * */
void Flash_Write_Data(uint32_t addr, uint8_t *data, uint8_t length);
/* @brief: read data from flash
 * @param:
 * 	addr: 	  the address of the data package
 * 	data:     the pointer of the data package
 * 	length:   the length of the data package
 * */
void Flash_Read_Data(uint32_t addr, uint8_t *data, uint8_t length);

/****serial port and usb port****/
extern uint8_t COMM_0, COMM_1, COMM_2, COMM_3, COMM_4;
/***************usb+串口配置****************
 * *************comm0:USB口***************
 * *************comm1:Serial串口1**********
 * *************comm2:Serial串口2**********
 * *************comm3:Serial串口3**********
 * *************comm4:Serial串口4**********
 * @param:
 * comm0~comm4可以配置为下列可选参数,参数及其含义如下：
 * (1)DEV_COMM 		自定义模式
 * (2)MAV_COMM  	Mavlink模式
 * (3)GPS_COMM  	GPS模式
 * (4)TFMINI_COMM  	TFmini激光测距仪
 * **************************************/
void config_comm(uint8_t comm0, uint8_t comm1, uint8_t comm2, uint8_t comm3, uint8_t comm4);
/**
  * @brief  采用缓冲的方式从usb或串口发送数据（强烈推荐采用这种方式）
  * @param  chan: 端口号（MAVLINK_COMM_0为USB;MAVLINK_COMM_1~MAVLINK_COMM_4为串口1~4）
  * @param  buf: 待发送数据的数组起始地址
  * @param  len: 待发送数据的数组长度
  * @retval None
  */
void comm_send_buf(mavlink_channel_t chan, uint8_t* buf, uint16_t len);

//usb port
extern uint32_t USB_Buffer_length;
/**
  * @brief  usb数据发送函数
  * @param  buf: 待发送数据的数组起始地址
  * @param  len: 待发送数据的数组长度
  * @retval None
  */
void send_usb_data(uint8_t* buf, uint16_t len);
/**
  * @brief  usb中断接收回调函数
  * @param  buf: 已接收数据的数组起始地址
  * @param  len: 已接收数据的数组长度
  * @retval None
  */
void read_usb_data_callback(uint8_t* Buf, uint32_t Len);// 回调函数
/**
  * @brief  从usb口输出字符，用法与printf相同，主要用于调试
  */
void usb_printf(const char* s, ...);
void usb_printf_dir(const char* s, ...);

//serial port
extern uint32_t URAT_DMA_Buffer_length; // tx/rx buffer max length
void set_s1_baudrate(uint32_t baudrate);//配置串口1波特率，默认初始波特率为115200
void set_s2_baudrate(uint32_t baudrate);//配置串口2波特率，默认初始波特率为115200
void set_s3_baudrate(uint32_t baudrate);//配置串口3波特率，默认初始波特率为115200
void set_s4_baudrate(uint32_t baudrate);//配置串口4波特率，默认初始波特率为115200
/**
  * @brief  通过环形缓冲区，从串口1-串口4输出字符，用法与printf相同，主要用于调试
  */
void s1_printf(const char* s, ...);
void s2_printf(const char* s, ...);
void s3_printf(const char* s, ...);
void s4_printf(const char* s, ...);
/**
  * @brief  直接从串口1-串口4输出字符，不通过环形缓冲区
  */
void s1_printf_dir(const char* s, ...);
void s2_printf_dir(const char* s, ...);
void s3_printf_dir(const char* s, ...);
void s4_printf_dir(const char* s, ...);
/**
  * @brief  采用中断方式从串口1发送数据
  * @param  buf: 待发送数据的数组起始地址
  * @param  size: 待发送数据的长度
  * @retval 串口发送状态
  */
HAL_StatusTypeDef s1_send_buf(uint8_t* buf, uint16_t size);
/**
  * @brief  采用中断方式从串口2发送数据
  * @param  buf: 待发送数据的数组起始地址
  * @param  size: 待发送数据的长度
  * @retval 串口发送状态
  */
HAL_StatusTypeDef s2_send_buf(uint8_t* buf, uint16_t size);
/**
  * @brief  采用中断方式从串口3发送数据
  * @param  buf: 待发送数据的数组起始地址
  * @param  size: 待发送数据的长度
  * @retval 串口发送状态
  */
HAL_StatusTypeDef s3_send_buf(uint8_t* buf, uint16_t size);
/**
  * @brief  采用中断方式从串口4发送数据
  * @param  buf: 待发送数据的数组起始地址
  * @param  size: 待发送数据的长度
  * @retval 串口发送状态
  */
HAL_StatusTypeDef s4_send_buf(uint8_t* buf, uint16_t size);
/**
  * @brief  采用等待方式从串口发送数据
  * @param  buf: 待发送数据的数组起始地址
  * @param  size: 待发送数据的长度
  * @param  timeout: 最长等待时间(单位：ms)
  * @retval 串口发送状态
  */
HAL_StatusTypeDef s1_send_buf_delayms(uint8_t* buf, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef s2_send_buf_delayms(uint8_t* buf, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef s3_send_buf_delayms(uint8_t* buf, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef s4_send_buf_delayms(uint8_t* buf, uint16_t size, uint32_t timeout);

/**
  * @brief  这个函数可以把待发送数据存入mavlink缓冲区, 无阻塞
  * @param  chan: 消息发送端口（MAVLINK_COMM_0~MAVLINK_COMM_4）
  * 			MAVLINK_COMM_0 为usb口
  * 			MAVLINK_COMM_1为串口1
  * 			MAVLINK_COMM_2为串口2
  * 			MAVLINK_COMM_3为串口3
  * 			MAVLINK_COMM_4为串口4
  * @param  msg: 待发送的mavlink消息包
  * @retval None
  */
void mavlink_send_buffer(mavlink_channel_t chan, mavlink_message_t *msg);
//把mavlink缓冲区中的数据以非阻塞方式从MAVLINK_COMM_0~MAVLINK_COMM_4中发送出去
void mav_send_data(void);
#define EVENTBIT_HEARTBEAT_COMM_0 (1<<0) //usb
#define EVENTBIT_HEARTBEAT_COMM_1 (1<<1) //串口1
#define EVENTBIT_HEARTBEAT_COMM_2 (1<<2) //串口2
#define EVENTBIT_HEARTBEAT_COMM_3 (1<<3) //串口3
#define EVENTBIT_HEARTBEAT_COMM_4 (1<<4) //串口4
extern uint8_t HeartBeatFlags;//判断EVENTBIT_HEARTBEAT_COMM_0~EVENTBIT_HEARTBEAT_COMM_4五个通道是否有心跳包接收

//ADC
void adc_update(void);			// 刷新全部adc数据
float get_batt_volt(void);   	// 获取外部电池供电电压值，Unit:V
float get_batt_current(void);	// 获取外部电池供电电流值，Unit:A
float get_5v_in(void);			// 获取板载5v供电电压测量值，返回值范围（0~6.6v）
float get_pressure(void);       // 获取外部模拟输入的电压测量值，返回值范围（0~6.6v）

/******************************************
 * **************RCOUT MAP*****************
 * ****************************************
 * **********motor1 ----- motor5 **********
 * **********motor2 ----- motor6 **********
 * **********motor3 ----- motor7 **********
 * **********motor4 ----- motor8 **********
 * **********servo1 ----- servo3 **********
 * **********servo2 ----- servo4 **********
 * *****gpio2 gpio1 ----- gpio5 gpio6******
 * *****gpio4 gpio3 ----- gpio7 gpio8******
 * ****************************************
 * ****************************************/
/**
  * @brief  是否启用rc控制引脚（包括电机、舵机、GPIO）
  * @param  set: ture/false
  * 		true为启用，false禁用
  */
void set_rcout_enable(bool set);

//重新设置电机motor1~motor8输出口PWM频率
void reset_motor_freq(uint16_t freq);
//重新设置舵机servo1~servo4输出口PWM频率
void reset_servo_freq(uint16_t freq);

/**
  * @brief  配置电机控制引脚输出的pwm脉宽.
  * @param  i: 电机号（1~8）
  * @param  value: pwm脉宽（1000~2000）
  * @retval None
  */
void Motor_Set_Value(uint8_t i, uint16_t value);

/**
  * @brief  配置舵机控制引脚输出的pwm脉宽.
  * @param  i: 舵机号（1~4）
  * @param  value: pwm脉宽（500~2500）
  * @retval None
  */
void Servo_Set_Value(uint8_t i, uint16_t value);

/**
  * @brief  把配置好的所有电机脉宽值传递给底层寄存器
  */
void set_motors_value(void);

/**
  * @brief  把配置好的所有舵机脉宽值传递给底层寄存器
  */
void set_servos_value(void);

void FMU_PWM_Set_Output_Enable(void);//调用这个函数之后电机、舵机PWM输出才会生效
void FMU_PWM_Set_Output_Disable(void);//调用这个函数之后电机、舵机PWM输出会终止

/**********************配置gpio口模式***************************
 * @param  mode:
 * 				GPIO_MODE_OUTPUT_PP(推挽输出模式);
 * 				GPIO_MODE_OUTPUT_OD(开漏输出模式);
 * 				GPIO_MODE_INPUT(输入模式);
 * 				GPIO_MODE_ANALOG(模拟模式)
 * 				GPIO_MODE_IT_RISING(中断模式|上升沿触发);
 *				GPIO_MODE_IT_FALLING(中断模式|下降沿触发);
 *				GPIO_MODE_IT_RISING_FALLING(中断模式|双边沿触发);
 * *********************************************************/
void set_gpio1_mode(uint32_t mode);
void set_gpio2_mode(uint32_t mode);
void set_gpio3_mode(uint32_t mode);
void set_gpio4_mode(uint32_t mode);
void set_gpio5_mode(uint32_t mode);
void set_gpio6_mode(uint32_t mode);
void set_gpio7_mode(uint32_t mode);
void set_gpio8_mode(uint32_t mode);

/**
  * @brief  在输出模式下，控制gpio引脚输出 (gpio1~gpio8)
  * @param  set: ture/false
  * 		true为高电平，false为低电平
  */
void write_gpio1(bool set);
void write_gpio2(bool set);
void write_gpio3(bool set);
void write_gpio4(bool set);
void write_gpio5(bool set);
void write_gpio6(bool set);
void write_gpio7(bool set);
void write_gpio8(bool set);

/**
  * @brief  在输入模式下，读取gpio引脚电平 (gpio1~gpio8)
  * @return  返回值: ture/false
  * 		true为高电平，false为低电平
  */
bool read_gpio1(void);
bool read_gpio2(void);
bool read_gpio3(void);
bool read_gpio4(void);
bool read_gpio5(void);
bool read_gpio6(void);
bool read_gpio7(void);
bool read_gpio8(void);

/**
 * @brief  在中断模式下，gpio外部中断回调函数(gpio1~gpio8)
 *
 * **/
void gpio1_interrupt_callback(void);
void gpio2_interrupt_callback(void);
void gpio3_interrupt_callback(void);
void gpio4_interrupt_callback(void);
void gpio5_interrupt_callback(void);
void gpio6_interrupt_callback(void);
void gpio7_interrupt_callback(void);
void gpio8_interrupt_callback(void);

//RC-INPUT
void PPM_RX_InterruptHandler(void);
void SBUS_RX_InterruptHandler(void);
HAL_StatusTypeDef sbus_output_buf(uint8_t* buf, uint16_t size);
HAL_StatusTypeDef sbus_output_buf_delayms(uint8_t* buf, uint16_t size, uint32_t timeout);
void RC_Input_Init(uint8_t mode);//初始化遥控接收机（PPM/SBUS）
void RC_Input_Loop(void);//接收遥控器数据
void set_rc_channels_override(bool set);//设置Mavlink覆盖遥控器信号
extern uint16_t *mav_channels_in;

//初始化wifi模组
void wifi_init(void);

//设置8个通道的遥控器信号（系统已经调用过了，一般情况下无需再次调用）
//@range: _channel_roll/_channel_pitch/_channel_yaw:-1.0~1.0; _channel_throttle:0~1.0;
void set_channel_roll(float roll);
void set_channel_pitch(float pitch);
void set_channel_yaw(float yaw);
void set_channel_throttle(float throttle);
void set_channel_5(float value); //range: 0~1
void set_channel_6(float value); //range: 0~1
void set_channel_7(float value); //range: 0~1
void set_channel_8(float value); //range: 0~1

//获取8个通道的遥控原生信号脉宽（1000~2000,单位：us）
uint16_t input_channel_roll(void);
uint16_t input_channel_pitch(void);
uint16_t input_channel_yaw(void);
uint16_t input_channel_throttle(void);
uint16_t input_channel_5(void);
uint16_t input_channel_6(void);
uint16_t input_channel_7(void);
uint16_t input_channel_8(void);
//获取解析后的8个通道遥控数据
float get_channel_roll(void);		//滚转通道，range: -1~1
float get_channel_pitch(void);		//俯仰通道，range: -1~1
float get_channel_yaw(void);		//偏航通道，range: -1~1
float get_channel_throttle(void); 	//油门通道，range: 0~1
float get_channel_5(void); 			//5通道，range: 0~1
float get_channel_6(void); 			//6通道，range: 0~1
float get_channel_7(void); 			//7通道，range: 0~1
float get_channel_8(void); 			//8通道，range: 0~1

#ifdef __cplusplus
}
#endif

#endif /* __HAL_H */
