/*
 * common.cpp
 *
 *  Created on: 2020.07.16
 *      Author: JackyPan
 */
#include "maincontroller.h"

static bool _soft_armed=false;//心跳包中表示是否解锁的标志位
static bool _thr_force_decrease=false;//强制油门下降
ROBOT_STATE robot_state=STATE_NONE;
ROBOT_STATE robot_state_desired=STATE_NONE;
ROBOT_MAIN_MODE robot_main_mode=MODE_AIR;
ROBOT_SUB_MODE robot_sub_mode=MODE_STABILIZE;

bool get_soft_armed(void){
	return _soft_armed;
}

void set_soft_armed(bool soft_armed){
	_soft_armed=soft_armed;
}

bool get_thr_force_decrease(void){
	return _thr_force_decrease;
}

void set_thr_force_decrease(bool force_decrease){
	_thr_force_decrease=force_decrease;
}

bool mode_init(void){
	return mode_stabilize_init();
}

/* 模式切换说明：
 * (1) 模式切换分为主模式和子模式, 其中主模式包括空中模式和地面模式由channel5控制, 子模式隶属于主模式由channel6控制;
 * (2) channel5>0.9时为无人车模式, channel5<=0.9时为空中模式;
 * (3) 在空中模式下:
 * 		channel6>0.7为自稳模式;
 * 		0.7>=channel6>0.3为定高模式;
 * 		0.3>=channel6为自主导航模式;
 * (4) 在无人车模式下:
 *		channel6>0.7为遥控直通模式;
 * 		0.7>=channel6>0.3为无人车定速模式;
 * 		0.3>=channel6为无人车定点模式;
 * */
//should be run at 400hz
void mode_update(void){
	float ch5=get_channel_5();//用于机器人主模态切换
	float ch6=get_channel_6();//用于机器人子模态切换
	if(ch5>0.9){//无人车模式
		robot_state_desired=STATE_DRIVE;
	}else{//无人机模式
		if(robot_main_mode==MODE_UGV){
			robot_state_desired=STATE_FLYING;//从UGV切换过来时，自动起飞
		}else{
			robot_state_desired=STATE_NONE;//清空状态标志
		}
	}
	switch(robot_state){
	case STATE_STOP:
		if(robot_state_desired==STATE_DRIVE){
			robot_main_mode=MODE_UGV;
		}
		break;
	case STATE_DRIVE:
		if(robot_state_desired==STATE_FLYING){
			robot_main_mode=MODE_AIR;
		}
		break;
	default:
		break;
	}
	switch(robot_main_mode){
	case MODE_AIR:
		if(ch6>0.7){
			if(robot_sub_mode!=MODE_STABILIZE){
				if(mode_stabilize_init()){
					robot_sub_mode=MODE_STABILIZE;
				}
			}
		}else if(ch6>0.3){
			if(robot_sub_mode!=MODE_ALTHOLD){
				if(mode_althold_init()){
					robot_sub_mode=MODE_ALTHOLD;
				}
			}
		}else{
			if(robot_sub_mode!=MODE_POSHOLD){
				if(mode_poshold_init()){
					robot_sub_mode=MODE_POSHOLD;
				}
			}
		}
		break;
	case MODE_MECANUM:
		if(robot_sub_mode!=MODE_MECANUM_A){
			if(mode_mecanum_init()){
				robot_sub_mode=MODE_MECANUM_A;
			}
		}
		break;
	case MODE_SPIDER:
		break;
	case MODE_UGV:
		robot_state=STATE_DRIVE;
		if(ch6>0.7){
			//无人车遥控直通模式
			if(robot_sub_mode!=MODE_UGV_A){
				//init()
				robot_sub_mode=MODE_UGV_A;
			}
		}else if(ch6>0.3){
			//无人车定速模式
			if(robot_sub_mode!=MODE_UGV_V){
				//init()
				robot_sub_mode=MODE_UGV_V;
			}
		}else{
			//无人车定点模式
			if(robot_sub_mode!=MODE_UGV_P){
				//init()
				robot_sub_mode=MODE_UGV_P;
			}
		}
		break;
	default:
		break;
	}
	switch(robot_sub_mode){
	case MODE_STABILIZE:
		mode_stabilize();
		break;
	case MODE_ALTHOLD:
		mode_althold();
		break;
	case MODE_POSHOLD:
		mode_poshold();
		break;
	case MODE_AUTONAV:
		mode_autonav();
		break;
	case MODE_PERCH:
		mode_perch();
		break;
	case MODE_MECANUM_A:
		mode_mecanum();
		break;
	case MODE_UGV_A:
		//无人车遥控直通update()
		break;
	case MODE_UGV_V:
		//无人车定速模式update()
		break;
	case MODE_UGV_P:
		//无人车定点模式update()
		break;
	default:
		break;
	}
}
