/*
 * um482.h
 *
 *  Created on: 2022年6月16日
 *      Author: 25053
 */

#ifndef INCLUDE_GNSS_UM482_H_
#define INCLUDE_GNSS_UM482_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "gnss/gps.h"

void ParseUM482(uint8_t b);

typedef enum {
	UM482_IDLE = 0,
	UM482_Header1_24,
	UM482_Header2_24,
	UM482_Header3_24,
	UM482_Gnss_4,
	UM482_Length_1,
	UM482_UTC_6,//年月日时分秒
	UM482_RTK_Status_1,//0：无效解；1：单点定位解；2：伪距差分；4：固定解；	5：浮动解；
	UM482_Heading_Status_1,//0：无效解；4：固定解；5：浮动解；
	UM482_GPS_Num_1,
	UM482_BDS_Num_1,
	UM482_GLO_Num_1,
	UM482_Baseline_N_4,
	UM482_Baseline_E_4,
	UM482_Baseline_U_4,
	UM482_Baseline_NStd_4,
	UM482_Baseline_EStd_4,
	UM482_Baseline_UStd_4,
	UM482_Heading_4,
	UM482_Pitch_4,
	UM482_Roll_4,
	UM482_Speed_4,
	UM482_Vel_N_4,
	UM482_Vel_E_4,
	UM482_Vel_U_4,
	UM482_Vel_NStd_4,
	UM482_Vel_EStd_4,
	UM482_Vel_UStd_4,
	UM482_Lat_8,
	UM482_Lon_8,
	UM482_Alt_8,
	UM482_ECEF_X_8,
	UM482_ECEF_Y_8,
	UM482_ECEF_Z_8,
	UM482_LatStd_4,
	UM482_LonStd_4,
	UM482_AltStd_4,
	UM482_ECEFStd_X_4,
	UM482_ECEFStd_Y_4,
	UM482_ECEFStd_Z_4,
	UM482_Base_Lat_8,
	UM482_Base_Lon_8,
	UM482_Base_Alt_8,
	UM482_SEC_Lat_8,
	UM482_SEC_Lon_8,
	UM482_SEC_Alt_8,
	UM482_Week_Second_4,
	UM482_Diffage_4,
	UM482_Speed_Heading_4,
	UM482_Undulation_4,
	Remain_float_8,
	UM482_Num_GAL_1,
	Remain_char_3,
	UM482_CRC_4
}UM482_State;

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_GNSS_UM482_H_ */
