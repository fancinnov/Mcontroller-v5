/*
 * ringbuffer.h
 *
 *  Created on: 2020.01.18
 *      Author: JackyPan
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "hal.h"

typedef struct {
	uint8_t* pBuff;
	uint8_t* pEnd;  // pBuff + legnth
	uint8_t* wp;    // Write Point
	uint8_t* rp;    // Read Point
	uint16_t length;
	uint8_t  flagOverflow; // set when buffer overflowed
} RingBuffer;

//初始化RingBuffer
void rbInit(RingBuffer* pRingBuff, uint8_t* buff, uint16_t length);
//清空RingBuffer
void rbClear(RingBuffer* pRingBuff);
//把一字节数据存进RingBuffer
void rbPush(RingBuffer* pRingBuff, uint8_t value);
//读取一个字节数据出来
uint8_t rbPop(RingBuffer* pRingBuff);
//当前还有多少数据没被读取
uint16_t rbGetCount(const RingBuffer* pRingBuff);
//当前RingBuffer是不是空的
int8_t rbIsEmpty(const RingBuffer* pRingBuff);
//当前RingBuffer是不是满的
int8_t rbIsFull(const RingBuffer* pRingBuff);

#ifdef __cplusplus
}
#endif

#endif /* __RINGBUFFER_H */
