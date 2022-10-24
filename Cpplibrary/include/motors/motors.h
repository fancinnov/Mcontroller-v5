/*
 * motors.h
 *
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */
#pragma once

#ifndef __MOTORS_H
#define __MOTORS_H

#include "common.h"

// offsets for motors in motor_out and _motor_filtered arrays
#define MOTORS_MOT_1 0U
#define MOTORS_MOT_2 1U
#define MOTORS_MOT_3 2U
#define MOTORS_MOT_4 3U
#define MOTORS_MOT_5 4U
#define MOTORS_MOT_6 5U
#define MOTORS_MOT_7 6U
#define MOTORS_MOT_8 7U

#define MOTORS_MAX_NUM_MOTORS 8 //根据实际情况设置电机数,默认情况下Mcontroller 允许最多控制12个电机+舵机

#define MOTORS_YAW_FACTOR_CW   -1.0f//顺时针
#define MOTORS_YAW_FACTOR_CCW   1.0f//逆时针

#define FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define FILT_HZ_MIN      0.01f   // minimum input filter frequency

#define AP_MOTORS_DEFAULT_MID_THROTTLE  500

#define AP_MOTORS_SPIN_WHEN_ARMED       70      // spin motors at this PWM value when armed
#define AP_MOTORS_YAW_HEADROOM_DEFAULT  200.0f
#define AP_MOTORS_THST_EXPO_DEFAULT     0.65f   // set to 0 for linear and 1 for second order approximation
#define AP_MOTORS_THST_HOVER_DEFAULT    0.2f   // the estimated hover throttle, 0 ~ 1
#define AP_MOTORS_THST_HOVER_TC         10.0f   // time constant used to update estimated hover throttle, 0 ~ 1
#define AP_MOTORS_THST_HOVER_MIN        0.2f  // minimum possible hover throttle
#define AP_MOTORS_THST_HOVER_MAX        0.7f // maximum possible hover throttle
#define AP_MOTORS_SPIN_MIN_DEFAULT      0.0f   // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_SPIN_MAX_DEFAULT      1.0f   // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_BAT_VOLT_MAX_DEFAULT  0.0f    // voltage limiting max default
#define AP_MOTORS_BAT_VOLT_MIN_DEFAULT  0.0f    // voltage limiting min default (voltage dropping below this level will have no effect)
#define AP_MOTORS_BAT_CURR_MAX_DEFAULT  0.0f    // current limiting max default
#define AP_MOTORS_BAT_CURR_TC_DEFAULT   5.0f    // Time constant used to limit the maximum current
#define AP_MOTORS_BATT_VOLT_FILT_HZ     0.5f    // battery voltage filtered at 0.5hz

#ifdef __cplusplus
// @class      Motors
class Motors {
public:

    enum motor_frame_class {
        MOTOR_FRAME_UNDEFINED = 0,
        MOTOR_FRAME_QUAD = 1,
        MOTOR_FRAME_HEXA = 2,
        MOTOR_FRAME_OCTA = 3,
        MOTOR_FRAME_OCTAQUAD = 4,
    };
    enum motor_frame_type {
    	MOTOR_FRAME_TYPE_UNDEFINED = 0,
        MOTOR_FRAME_TYPE_X = 1,
        MOTOR_FRAME_TYPE_H = 2,
    };

    enum motor_pwm_type {
    	PWM_TYPE_ESC     = 0,
		PWM_TYPE_BRUSHED = 1,
		PWM_TYPE_SERVO   = 2
    };

    // spool up states
	enum spool_up_down_desired {
		DESIRED_SHUT_DOWN = 0,              // all motors stop
		DESIRED_SPIN_WHEN_ARMED = 1,        // all motors at spin when armed
		DESIRED_THROTTLE_UNLIMITED = 2,     // motors are no longer constrained by start up procedure
	};

    // spool up states
    typedef enum {
        SHUT_DOWN = 0,                      // all motors stop
        SPIN_WHEN_ARMED = 1,                // all motors at spin when armed
        SPOOL_UP = 2,                       // increasing maximum throttle while stabilizing
        THROTTLE_UNLIMITED = 3,             // throttle is no longer constrained by start up procedure
        SPOOL_DOWN = 4,                     // decreasing maximum throttle while stabilizing
    }spool_up_down_mode;

	// structure for holding motor limit flags
	struct Motors_limit {
		uint8_t roll_pitch      : 1; // we have reached roll or pitch limit
		uint8_t yaw             : 1; // we have reached yaw limit
		uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
		uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
	} limit;

    // Constructor
    Motors(float loop_rate);

    // check initialisation succeeded
    bool                initialised_ok() const { return _flags.initialised_ok; }

    void 				set_desired_spool_state(enum spool_up_down_desired spool) { _spool_desired = spool; };

    // arm, disarm or check status status of motors
    bool                get_armed() const { return _flags.armed; }
    void                set_armed(bool arm);

    // set motor interlock status
    void                set_interlock(bool set) { _flags.interlock = set;}

    // get motor interlock status.  true means motors run, false motors don't run
    bool                get_interlock() const { return _flags.interlock; }

    // pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
	void                set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input);
	void 				set_throttle_passthrough_for_motors(float throttle_input);

	// set loop rate. Used to support loop rate as a parameter
	void                set_loop_rate(float loop_rate) { _loop_rate = loop_rate; _dt=1/loop_rate;}

	// reset_filter - input filter will be reset to the next value provided to set_input()
	void                reset_filter() { _reset_filter = true; }

    // set_roll, set_pitch, set_yaw, set_throttle
    void                set_roll(float roll_in) { _roll_in = roll_in; };        // range -1 ~ +1
    void                set_pitch(float pitch_in) { _pitch_in = pitch_in; };    // range -1 ~ +1
    void                set_yaw(float yaw_in) { _yaw_in = yaw_in; };            // range -1 ~ +1
    void                set_throttle(float throttle_in);                        // range 0 ~ 1
    void                set_throttle_avg_max(float throttle_avg_max) { _throttle_avg_max = constrain_float(throttle_avg_max,0.0f,1.0f); };   // range 0 ~ 1
    void                set_throttle_hover(float throttle_hover) { _throttle_hover = throttle_hover; };   // range 0 ~ 1
    void                set_throttle_max(float throttle_max) { _throttle_max = CONSTRAIN(throttle_max,0.0f,1.0f); };   // range 0 ~ 1
    void                set_throttle_min(float throttle_min) { _throttle_min = CONSTRAIN(throttle_min,0.0f,1.0f); };   // range 0 ~ 1
    void                set_throttle_filter_cutoff(float hz) ;
    // update estimated throttle required to hover
	void                update_throttle_hover(float dt);
    // run spool logic
    void                output_logic();

    // accessors for roll, pitch, yaw and throttle inputs to motors
    float               get_roll() const { return _roll_in; }
    float               get_pitch() const { return _pitch_in; }
    float               get_yaw() const { return _yaw_in; }
    float               get_throttle() const { return _throttle_filter; }
    float       		get_throttle_hover() const { return _throttle_hover; }
    // returns maximum thrust in the range 0 to 1
    float               get_throttle_max() const { return _throttle_max; }
    float               get_throttle_min() const { return _throttle_min; }
    float               get_filt_alpha() const;
    motor_frame_class   get_frame_class() const { return _frame_class;}
    motor_frame_type    get_frame_type() const { return _frame_type;}
    motor_pwm_type      get_pwm_type() const { return _pwm_type; }

    // add_motor using raw roll, pitch, throttle and yaw factors
	void                add_motor(uint8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac);

	// configures the motors for the defined frame_class and frame_type
	void                setup_motors(motor_frame_class frame_class, motor_frame_type frame_type, motor_pwm_type pwm_type);
	void                normalise_rpy_factors();
    void                output_to_motors();
    void                output_armed_stabilizing();
    void  				output();
    void 				output_min_throttle();
    float				get_thrust_rpyt_out(uint8_t num){return _thrust_rpyt_out[num];}

protected:

    // flag bitmask
    struct Motors_flags {
        bool armed = false;    // false if disarmed, true if armed
        bool interlock = false;    // true if the motor interlock is enabled (i.e. motors run), false if disabled (motors don't run)
        bool initialised_ok = false;    // true if initialisation was successful
    } _flags;

    // internal variables
    float               _loop_rate;                 // rate in Hz at which output() function is called (normally 400hz)
    float               _dt;                        // _dt=1/_loop_rate
    float               _roll_in;                   // desired roll control from attitude controllers, -1 ~ +1
    float               _pitch_in;                  // desired pitch control from attitude controller, -1 ~ +1
    float               _yaw_in;                    // desired yaw control from attitude controller, -1 ~ +1
    float               _filt_hz;
    bool                _reset_filter;
    float               _roll_filter;
    float               _pitch_filter;
    float               _yaw_filter;
    float               _throttle_filter;           // throttle input filter
    spool_up_down_desired _spool_desired;           // desired spool state

    motor_pwm_type      _pwm_type;                  // PWM output type

	// motor output variables
	bool                motor_enabled[MOTORS_MAX_NUM_MOTORS];    // true if motor is enabled
	float               _roll_factor[MOTORS_MAX_NUM_MOTORS];     // each motors contribution to roll
	float               _pitch_factor[MOTORS_MAX_NUM_MOTORS];    // each motors contribution to pitch
	float               _yaw_factor[MOTORS_MAX_NUM_MOTORS];      // each motors contribution to yaw (normally 1 or -1)
	float               _thrust_rpyt_out[MOTORS_MAX_NUM_MOTORS]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range
	motor_frame_class   _frame_class;                            // most recently requested frame class (i.e. quad, hexa, octa, etc)
	motor_frame_type    _frame_type;                             // most recently requested frame type (i.e. plus, x, v, etc)

	float               _throttle_max;    			   		// max throttle output (0~1)
	float               _throttle_min;       				// min throttle output (0~1)
    float               _throttle_avg_max;                      // last throttle input from set_throttle_avg_max

	// parameters
	float               _yaw_headroom;          // yaw control is given at least this pwm range
	float            	_thrust_curve_expo;     // curve used to linearize pwm to thrust conversion.  set to 0 for linear and 1 for second order approximation
	float            	_spin_min;      // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
	float            	_spin_max;      // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
	float            	_spin_arm;      // throttle out ratio which produces the armed spin rate.  (i.e. 0 ~ 1 ) of the full throttle range
	float            	_batt_voltage_max;      // maximum voltage used to scale lift
	float            	_batt_voltage_min;      // minimum voltage used to scale lift
	float            	_batt_current_max;      // current over which maximum throttle is limited
	float            	_batt_current_time_constant;    // Time constant used to limit the maximum current
	uint8_t             _batt_idx;              // battery index used for compensation
	uint16_t            _pwm_min;               // minimum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's min pwm used)
	uint16_t            _pwm_max;               // maximum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's max pwm used)
	float            	_throttle_hover;        // estimated throttle required to hover throttle in the range 0 ~ 1
	uint8_t             _throttle_hover_learn=0;  // enable/disabled hover thrust learning
	uint8_t             _disarm_disable_pwm;    // disable PWM output while disarmed

	// Maximum lean angle of yaw servo in degrees. This is specific to tricopter
	float            	_yaw_servo_angle_max_deg;

	// time to spool motors to min throttle
	float            	_spool_up_time;

	// scaling for booster motor throttle
	float            	_boost_scale;

	// motor output variables
	int16_t             _throttle_radio_min;        // minimum PWM from RC input's throttle channel (i.e. minimum PWM input from receiver, RC3_MIN)
	int16_t             _throttle_radio_max;        // maximum PWM from RC input's throttle channel (i.e. maximum PWM input from receiver, RC3_MAX)

	// spool variables
	spool_up_down_mode  _spool_mode;         // motor's current spool mode
	float               _spin_up_ratio;      // throttle percentage (0 ~ 1) between zero and throttle_min

	// battery voltage, current and air pressure compensation variables
	float               _batt_voltage_filt;     // filtered battery voltage expressed as a percentage (0 ~ 1.0) of batt_voltage_max
	float               _lift_max;              // maximum lift ratio from battery voltage
	float               _throttle_limit;        // ratio of throttle limit between hover and maximum
	float               _throttle_thrust_max;   // the maximum allowed throttle thrust 0.0 to 1.0 in the range throttle_min to throttle_max
	uint16_t            _disarm_safety_timer;

};
#endif /* __cplusplus */

#endif /* __MOTORS_H */
