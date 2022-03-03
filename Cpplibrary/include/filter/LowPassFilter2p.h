/*
 * LowPassFilter2p.h
 *
 *  Created on: 2021.04.21
 *      Author: JackyPan
 */
#pragma once
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LowPassFilter2p_H
#define __LowPassFilter2p_H

#include "hal.h"
#include "math/math_inc.h"

/// @file   LowPassFilter2p.h
/// @brief  A class to implement a second order low pass filter
template <class T>
class DigitalBiquadFilter {
public:
    struct biquad_params {
        float cutoff_freq;
        float sample_freq;
        float a1;
        float a2;
        float b0;
        float b1;
        float b2;
    };

    DigitalBiquadFilter();

    T apply(const T &sample, const struct biquad_params &params);
    void reset();
    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);

private:
    T _delay_element_1;
    T _delay_element_2;
};

template <class T>
class LowPassFilter2p {
public:
    LowPassFilter2p();
    // constructor
    LowPassFilter2p(float sample_freq, float cutoff_freq);
    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);
    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    float get_sample_freq(void) const;
    T apply(const T &sample);
    void reset(void);

protected:
    struct DigitalBiquadFilter<T>::biquad_params _params;

private:
    DigitalBiquadFilter<T> _filter;
};

typedef LowPassFilter2p<int>      LowPassFilter2pInt;
typedef LowPassFilter2p<long>     LowPassFilter2pLong;
typedef LowPassFilter2p<float>    LowPassFilter2pFloat;
typedef LowPassFilter2p<Vector2f> LowPassFilter2pVector2f;
typedef LowPassFilter2p<Vector3f> LowPassFilter2pVector3f;

#endif /* __LowPassFilter2p_H */
