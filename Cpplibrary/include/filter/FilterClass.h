/*
 * FilterClass.h
 *
 *  Created on: 2022年7月21日
 *      Author: 25053
 */

#ifndef INCLUDE_FILTER_FILTERCLASS_H_
#define INCLUDE_FILTER_FILTERCLASS_H_

/// @file	FilterClass.h
/// @brief	A pure virtual interface class
///
#pragma once

#include <inttypes.h>

template <class T>
class Filter
{
public:
    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T apply(T sample) = 0;

    // reset - clear the filter
    virtual void reset()  = 0;

};

// Typedef for convenience
typedef Filter<int8_t> FilterInt8;
typedef Filter<uint8_t> FilterUInt8;
typedef Filter<int16_t> FilterInt16;
typedef Filter<uint16_t> FilterUInt16;
typedef Filter<int32_t> FilterInt32;
typedef Filter<uint32_t> FilterUInt32;

#endif /* INCLUDE_FILTER_FILTERCLASS_H_ */
