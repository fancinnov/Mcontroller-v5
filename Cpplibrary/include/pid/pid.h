/*
 * pid.h
 *
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */
#pragma once

#ifndef __PID_H
#define __PID_H

#include <hal.h>
#include "math/math_inc.h"

#define PID_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define PID_FILT_HZ_MIN      0.01f   // minimum input filter frequency

#ifdef __cplusplus
// @class	PID
// @brief	Copter PID control class
class PID {
public:

    // Constructor for PID
    PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff = 0);
    // set_dt - set time step in seconds
    void        set_dt(float dt);

    // set_input_filter_all - set input to PID controller
    //  input is filtered before the PID controllers are run
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_all(float input);

    // set_input_filter_d - set input to PID controller
    //  only input to the D portion of the controller is filtered
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_d(float input);

    // get_pid - get results from pid controller
    float       get_pid();
    float       get_pi();
    float       get_p();
    float       get_i();
    float       get_d();
    float       get_ff(float requested_rate);

    // reset_I - reset the integrator
    void        reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void        reset_filter() { _flags._reset_filter = true; }

    /// operator function call for easy initialisation
    void operator() (float p, float i, float d, float imaxval, float input_filt_hz, float dt, float ffval = 0);

    // get accessors
    float   	kP() { return _kp; }
    float   	kI() { return _ki; }
    float   	kD() { return _kd; }
    float   	filt_hz() { return _filt_hz; }
    float       imax() const { return _imax; }
    float       get_filt_alpha() const;
    float       ff() const { return _ff; }

    // set accessors
    void        kP(const float v) { _kp=v; }
    void        kI(const float v) { _ki=v; }
    void        kD(const float v) { _kd=v; }
    void        imax(const float v) { _imax=fabsf(v); }
    void        filt_hz(const float v);
    void        ff(const float v) { _ff=v; }

    float       get_integrator() const { return _integrator; }
    void        set_integrator(float i) { _integrator = i; }

    // parameter var table
//    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // parameters
    float        _kp=0;
    float        _ki=0;
    float        _kd=0;
    float        _imax=0;
    float        _filt_hz=PID_FILT_HZ_DEFAULT;                   // PID Input filter frequency in Hz
    float        _ff=0;

    // flags
    struct pid_flags {
        bool        _reset_filter : 1;    // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float           _dt;                    // timestep in seconds
    float           _integrator;            // integrator value
    float           _input;                 // last input for derivative
    float           _derivative;            // last derivative for low-pass filter

//    DataFlash_Class::PID_Info        _pid_info;
};
#endif /* __cplusplus */

#endif /* __PID_H */
