/*
 * pid_2d.h
 *
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */
#pragma once

#ifndef __PID_2D_H
#define __PID_2D_H

#include <hal.h>
#include "math/math_inc.h"

#ifdef __cplusplus
// @class	PID_2D
// @brief	PID_2D control class
class PID_2D {
public:

    // Constructor for PID
    PID_2D(float initial_p_x, float initial_p_y, float initial_i_x, float initial_i_y, float initial_d_x, float initial_d_y, float initial_imax, float initial_filt_hz, float initial_filt_d_hz, float dt);

    // set_dt - set time step in seconds
    void        set_dt(float dt);

    /// Overload the function call operator to permit easy initialisation
    void operator() (float initial_p_x, float initial_p_y, float initial_i_x, float initial_i_y, float initial_d_x, float initial_d_y, float initial_imax, float initial_filt_hz, float initial_filt_d_hz, float dt);

    // set_input - set input to PID controller
    //  input is filtered before the PID controllers are run
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input(const Vector2f &input);
    void        set_input(const Vector3f &input) { set_input(Vector2f(input.x, input.y)); }

    // get_pi - get results from pid controller
    Vector2f    get_pid();
    Vector2f    get_p() const;
    Vector2f    get_i();
    Vector2f    get_i_shrink();   // get_i but do not allow integrator to grow (it may shrink)
    Vector2f    get_d();

    // reset_I - reset the integrator
    void        reset_I();

    // reset_filter - input and D term filter will be reset to the next value provided to set_input()
    void        reset_filter();

    // get accessors
    Vector2f       kP() { return _kp; }
    Vector2f       kI() { return _ki; }
    Vector2f       kD() { return _kd; }
    float       imax() const { return _imax; }
    float       filt_hz() const { return _filt_hz; }
    float       get_filt_alpha() const { return _filt_alpha; }
    float       filt_d_hz() const { return _filt_hz; }
    float       get_filt_alpha_D() const { return _filt_alpha_d; }

    // set accessors
    void        kP(const Vector2f v) { _kp=v; }
    void        kI(const Vector2f v) { _ki=v; }
    void        kD(const Vector2f v) { _kd=v; }
    void        imax(const float v) { _imax=fabsf(v); }
    void        filt_hz(const float v);
    void        filt_d_hz(const float v);

    Vector2f    get_integrator() const { return _integrator; }
    void        set_integrator(const Vector2f &i) { _integrator = i; }
    void        set_integrator(const Vector3f &i) { _integrator.x = i.x; _integrator.y = i.y; }

protected:

    // set_input_filter_d - set input to PID controller
    //  only input to the D portion of the controller is filtered
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_d(const Vector2f& input_delta);

    // calc_filt_alpha - recalculate the input filter alpha
    void        calc_filt_alpha();

    // calc_filt_alpha - recalculate the input filter alpha
    void        calc_filt_alpha_d();

    // parameters
    Vector2f        _kp;
    Vector2f        _ki;
    Vector2f        _kd;
    float        _imax;
    float        _filt_hz;                   // PID Input filter frequency in Hz
    float        _filt_d_hz;                 // D term filter frequency in Hz

    // flags
    struct ac_pid_flags {
        bool        _reset_filter : 1;    // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float           _dt;            // timestep in seconds
    float           _filt_alpha;    // input filter alpha
    float           _filt_alpha_d;  // input filter alpha
    Vector2f        _integrator;    // integrator value
    Vector2f        _input;         // last input for derivative
    Vector2f        _derivative;    // last derivative for low-pass filter
};

#endif /* __cplusplus */

#endif /* __PID_H */
