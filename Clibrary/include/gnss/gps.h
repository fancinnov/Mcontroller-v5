/*
 * gps.h
 *
 *  Created on: 2020.01.18
 *      Author: JackyPan
 */
#pragma once
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hal.h"

#define GPS_READ_BUFFER_SIZE 128
#define DEBUG_GPS_ENABLE 0

#if  DEBUG_GPS_ENABLE==1
#define 	GPS_DEBUG(format, ...) usb_printf (format, ##__VA_ARGS__)
#else
#define     GPS_DEBUG(format,...)
#endif

	typedef enum {
		GPS = 0,    ///< normal GPS output
		RTCM        ///< request RTCM output. This is used for (fixed position) base stations
	}OutputMode;

	typedef enum {
		UBLOX = 0,    ///< UBLOX模组
		UM482        ///< UM482模组
	}GnssType;

	typedef enum {
		gnss_comm1 = 0,  //串口1
		gnss_comm2,		//串口2
		gnss_comm3,		//串口3
		gnss_comm4		//串口4
	}GnssComm;

	typedef struct
	{
		uint64_t timestamp; // required for logger
		uint8_t count;
		uint8_t svid[20];
		uint8_t used[20];
		uint8_t elevation[20];
		uint8_t azimuth[20];
		uint8_t snr[20];
		uint8_t _padding0[3]; // required for logger
		//const uint8_t SAT_INFO_MAX_SATELLITES = 20;
	}satellite_info_s;

	typedef struct
	{
		uint64_t timestamp; // required for logger
		uint64_t time_utc_usec;
		int32_t lat;
		int32_t lon;
		int32_t alt;
		int32_t alt_ellipsoid;
		float s_variance_m_s;
		float c_variance_rad;
		float eph;
		float epv;
		float hdop;
		float vdop;
		int32_t noise_per_ms;
		int32_t jamming_indicator;
		float vel_m_s;
		float vel_n_m_s;
		float vel_e_m_s;
		float vel_d_m_s;
		float vel_n_std;
		float vel_e_std;
		float vel_d_std;
		float cog_rad;
		int32_t timestamp_time_relative;
		uint8_t fix_type;
		uint8_t heading_status;//0：无效解；4：固定解；5：浮动解；
		uint8_t vel_ned_valid;
		uint8_t satellites_used;
		uint8_t gps_used;
		uint8_t bds_used;
		uint8_t glo_used;
		float baseline_n;//基线
		float baseline_e;
		float baseline_u;
		float heading; //双天线定向, 单位：度
		float lat_noise;
		float lon_noise;
		float alt_noise;
		uint8_t _padding0[5]; // required for logger
	}vehicle_gps_position_s;

	struct tm
	{
		int tm_sec;     /* second (0-61, allows for leap seconds) */
		int tm_min;     /* minute (0-59) */
		int tm_hour;    /* hour (0-23) */
		int tm_mday;    /* day of the month (1-31) */
		int tm_mon;     /* month (0-11) */
		int tm_year;    /* years since 1900 */
		int tm_wday;    /* day of the week (0-6) */                         /*not supported by NuttX*/
		int tm_yday;    /* day of the year (0-365) */                       /*not supported by NuttX*/
		int tm_isdst;   /* non-0 if daylight savings time is in effect */   /*not supported by NuttX*/
	};

	struct SurveyInStatus
	{
		uint32_t mean_accuracy;       /**< [mm] */
		uint32_t duration;            /**< [s] */
		uint8_t flags;                /**< bit 0: valid, bit 1: active */
	};

#define SAT_INFO_MAX_SATELLITES 20

	extern vehicle_gps_position_s *gps_position;
	extern GnssComm gnss_comm;
	bool GPS_Init(GnssType type);// type：GNSS模组类型
	void GPS_Baud_Reset(uint32_t baud);
	void get_gps_data(uint8_t buf);
	bool get_gps_state(void);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H */
