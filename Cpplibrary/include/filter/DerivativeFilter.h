/*
 * DerivativeFilter.h
 *
 *  Created on: 2022年7月21日
 *      Author: 25053
 */

#ifndef INCLUDE_FILTER_DERIVATIVEFILTER_H_
#define INCLUDE_FILTER_DERIVATIVEFILTER_H_

#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"

// 1st parameter <T> is the type of data being filtered.
// 2nd parameter <FILTER_SIZE> is the number of elements in the filter
template <class T, uint8_t FILTER_SIZE>
class DerivativeFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    // constructor
    DerivativeFilter() : FilterWithBuffer<T,FILTER_SIZE>() {
    };

    // update - Add a new raw value to the filter, but don't recalculate
    void update(T sample, uint32_t timestamp);

    // return the derivative value
    float slope(void);

    // reset - clear the filter
    virtual void        reset();

private:
    bool            _new_data;
    float           _last_slope;

    // microsecond timestamps for samples. This is needed
    // to cope with non-uniform time spacing of the data
    uint32_t        _timestamps[FILTER_SIZE];
};

typedef DerivativeFilter<float,5> DerivativeFilterFloat_Size5;
typedef DerivativeFilter<float,7> DerivativeFilterFloat_Size7;
typedef DerivativeFilter<float,9> DerivativeFilterFloat_Size9;


#endif /* INCLUDE_FILTER_DERIVATIVEFILTER_H_ */
