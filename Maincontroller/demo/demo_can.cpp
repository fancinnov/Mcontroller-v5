/*
 * demo_can.cpp
 *
 *  Created on: Aug 26, 2022
 *      Author: 25053
 */
#include"maincontroller.h"

void can_init(void){//在系统初始化的时候运行一次
	//首先配置CAN接收过滤器
	canFilterConfig->FilterBank = 14;
	canFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig->FilterIdHigh = 0x0000;
	canFilterConfig->FilterIdLow = 0x0000;
	canFilterConfig->FilterMaskIdHigh = 0x0000;
	canFilterConfig->FilterMaskIdLow = 0x0000;
	canFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilterConfig->FilterActivation = ENABLE;
	canFilterConfig->SlaveStartFilterBank = 14;
	//配置can发送数据的header
	canTxHeader->StdId = 0x200;
	canTxHeader->RTR = CAN_RTR_DATA;
	canTxHeader->IDE = CAN_ID_STD;
	canTxHeader->DLC = 8;
	canTxHeader->TransmitGlobalTime = DISABLE;
	set_can_recieve_multifold(2);//设置CAN总线接收倍率,倍率数对应于开辟的接收缓存数
	set_comm3_as_can();//把串口3重置为fdcan
}

static uint32_t notify=0;
void can_update(void){//以自己需要的频率循环运行即可
	//CAN数据的接收
	if(get_can_notification()!=notify){
		notify=get_can_notification();
		//获取接收到的can header,本demo直接打印了header中的id.由于开辟了两个接收缓存,所以调用的时候需要指明缓存编号为0和1
		usb_printf("id:%x|%x|%d\n",get_canRxHeader_prt(0)->StdId, get_canRxHeader_prt(1)->StdId, notify);
		//获取接收到的can data,打印了缓存0接收的8字节数据
		usb_printf("data:%d|%d|%d|%d|%d|%d|%d|%d\n",get_canRxData_prt(0)[0],get_canRxData_prt(0)[1],get_canRxData_prt(0)[2],get_canRxData_prt(0)[3],get_canRxData_prt(0)[4],get_canRxData_prt(0)[5],get_canRxData_prt(0)[6],get_canRxData_prt(0)[7]);
		//获取接收到的can data,打印了缓存1接收的8字节数据
		usb_printf("data:%d|%d|%d|%d|%d|%d|%d|%d\n",get_canRxData_prt(1)[0],get_canRxData_prt(1)[1],get_canRxData_prt(1)[2],get_canRxData_prt(1)[3],get_canRxData_prt(1)[4],get_canRxData_prt(1)[5],get_canRxData_prt(1)[6],get_canRxData_prt(1)[7]);
	}
	//CAN数据的发送
	canTxHeader->StdId = 0x200;;//配置发送数据id
	canTxData[0]=5000>>8;
	canTxData[1]=5000&0xFF;
	canTxData[2]=0;
	canTxData[3]=0;
	canTxData[4]=0;
	canTxData[5]=0;
	canTxData[6]=0;
	canTxData[7]=0;
	can_send_data();//demo演示了发送8字节数据
}


