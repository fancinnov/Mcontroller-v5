/*
 * flash.h
 *
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */
#pragma once

#ifndef __FLASH_H
#define __FLASH_H

#include "common.h"

#define _float_to_byte(buf, wire_offset, f)    *(float *)&buf[wire_offset] = f
#define _byte_to_float(buf, wire_offset, f)    f = *(float *)&buf[wire_offset]

class DataFlash{
public:
	DataFlash(){}
	void reset_addr_num_max(void);
	uint16_t get_addr_num_max(void);
	void set_param_uint8(uint16_t num, uint8_t value);
	void set_param_float(uint16_t num, float value);
	void set_param_vector3f(uint16_t num, Vector3f value);
	void set_param_pid(uint16_t num, float p, float i, float d, float i_max, float filt_hz);
	void set_param_pid_2d(uint16_t num, float p, float i, float d, float i_max, float filt_hz, float filt_d_hz);
	void get_param_uint8(uint16_t num, uint8_t &value);
	void get_param_float(uint16_t num, float &value);
	void get_param_vector3f(uint16_t num, Vector3f &value);
	void get_param_pid(uint16_t num, float &p, float &i, float &d, float &i_max, float &filt_hz);
	void get_param_pid_2d(uint16_t num, float &p, float &i, float &d, float &i_max, float &filt_hz, float &filt_d_hz);

private:
	uint16_t addr_num_max=0;
	uint8_t addr[2];
	uint8_t buf[DATA_FLASH_LENGTH];
	void update_addr_num_max(uint16_t addr_num);
	void write_data(uint32_t addr, uint8_t *data, uint8_t length);
	void read_data(uint32_t addr, uint8_t *data, uint8_t length);
};

#endif
