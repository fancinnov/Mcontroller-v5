//
/// @file	LowPassFilter.h
/// @brief	A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

/*
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */

/*
  Note that this filter can be used in 2 ways:

   1) providing dt on every sample, and calling apply like this:

      // call once
      filter.set_cutoff_frequency(frequency_hz);

      // then on each sample
      output = filter.apply(sample, dt);

   2) providing a sample freq and cutoff_freq once at start

      // call once
      filter.set_cutoff_frequency(sample_freq, frequency_hz);

      // then on each sample
      output = filter.apply(sample);

  The second approach is more CPU efficient as it doesn't have to
  recalculate alpha each time, but it assumes that dt is constant
 */

#pragma once

#include "hal.h"
#include "math/math_inc.h"

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF {
public:
    DigitalLPF();
    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T &sample, float cutoff_freq, float dt);
    T apply(const T &sample);

    void compute_alpha(float sample_freq, float cutoff_freq);

    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    const T &get() const;
    void reset(T value);

private:
    T _output;
    float alpha = 1.0f;
};

// LPF base class
template <class T>
class LowPassFilter {
public:
    LowPassFilter();
    LowPassFilter(float cutoff_freq);
    LowPassFilter(float sample_freq, float cutoff_freq);

    // change parameters
    void set_cutoff_frequency(float cutoff_freq);
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);

    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    T apply(T sample, float dt);
    T apply(T sample);
    const T &get() const;
    void reset(T value);
    void reset(void) { reset(T()); }

protected:
    float _cutoff_freq;

private:
    DigitalLPF<T> _filter;
};

// typedefs for compatibility
typedef LowPassFilter<int>      LowPassFilterInt;
typedef LowPassFilter<long>     LowPassFilterLong;
typedef LowPassFilter<float>    LowPassFilterFloat;
typedef LowPassFilter<Vector2f> LowPassFilterVector2f;
typedef LowPassFilter<Vector3f> LowPassFilterVector3f;
