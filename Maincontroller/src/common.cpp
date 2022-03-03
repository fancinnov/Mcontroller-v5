/*
 * cpp.cpp
 *
 *  Created on: 2020.xx.xx
 *      Author: JackyPan
 */
#include "common.h"

static bool _soft_armed=false;//心跳包中表示是否解锁的标志位
static bool _thr_force_decrease=false;//强制油门下降

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
