/*
 * sdlog.h
 *
 *  Created on: 2020.07.26
 *      Author: JackyPan
 */
#pragma once
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDLOG_H
#define __SDLOG_H

#include "common.h"

class SDLog{
private:
	typedef enum{
		LOG_CAT = 0,
		LOG_DATA,
		LOG_END,
	}Log_Type;

	UINT log_num;
	FIL log_file;
	FIL index_file;
	char buff_read[4];	/* Pointer to data buffer */
	UINT btr=4;         /* Number of bytes to read */
	UINT br;	        /* number of bytes read */
	uint16_t index_log;
	FRESULT res;
	UINT len_filename;
	char file_name[10];
	va_list args;
	uint8_t buffer_max=128;
	UINT len;
	char log_buffer[128];
	FRESULT Open_Log_File(void);
	void Close_Log_File(void);
	void Log_To_File(Log_Type log_type);

public:
	SDLog(void);

	typedef enum{
		Logger_Idle = 0,
		Logger_Open,
		Logger_Close,
		Logger_Record
	}Logger_Status;
	Logger_Status m_Logger_Status;

	void Logger_Update(void);
	void Logger_Enable(void);
	void Logger_Disable(void);
	void Logger_Write(const char* s, ...);
};

#endif /* __SDLOG_H */
