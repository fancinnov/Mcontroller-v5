/*
 * uwb.h
 *
 *  Created on: Nov 20, 2021
 *      Author: 25053
 */
#pragma once

#ifndef INCLUDE_UWB_UWB_H_
#define INCLUDE_UWB_UWB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hal.h"
#include "uwb/deca_device_api.h"
#include "uwb/deca_regs.h"
#include "uwb/trilateration.h"

#define TAG_ID 0x02
#define MASTER_TAG 0x01
#define MAX_SLAVE_TAG 0x02
#define SLAVE_TAG_START_INDEX 0x01

#define ANCHOR_MAX_NUM 4 //3 4
#define ANCHOR_IND 1  // 1 2 3 4

typedef enum{
	none=0,
	tag=1,
	anchor
}uwb_modes;

typedef enum{
	idle=0,
	receive,
	poll,
	resp,
	final,
	dis,
	release,
	release_confirm,
	release_wait,
	statistics
}uwb_states;

bool uwb_init(void);
void uwb_mode_init(uwb_modes mode);
void uwb_update(void);
void set_anchor_positon(uint8_t id, double x, double y, double z);
extern bool get_uwb_position;

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_UWB_UWB_H_ */
