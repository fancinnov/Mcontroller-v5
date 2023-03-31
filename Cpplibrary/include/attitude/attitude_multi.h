/*
 * attitude_multi.h
 *
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */
#pragma once

#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "common.h"
#include "motors/motors.h"
#include "pid/pid.h"
#include "pid/p.h"

#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS       radians(40.0f)   // minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS       radians(720.0f)  // maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS        radians(10.0f)   // minimum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS        radians(120.0f)  // maximum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_DS        90      // constraint on yaw angle error in degrees.  This should lead to maximum turn rate of 10deg/sec * Stab Rate P so by default will be 90deg/sec.
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_DSS   1100.0f // default maximum acceleration for roll/pitch axis in degrees/sec/sec
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_DSS    270.0f  // default maximum acceleration for yaw axis in degrees/sec/sec

#define AC_ATTITUDE_RATE_CONTROLLER_TIMEOUT             1.0f    // body-frame rate controller timeout in seconds
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          1.0f    // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         1.0f    // body-frame rate controller maximum output (for yaw axis)

#define AC_ATTITUDE_THRUST_ERROR_ANGLE                  radians(30.0f) // Thrust angle error above which yaw corrections are limited

#define AC_ATTITUDE_400HZ_DT                            0.0025f // delta time in seconds for 400hz update rate

#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT          1       // body-frame rate feedforward enabled by default

#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT      1.0f    // Time constant used to limit lean angle so that vehicle does not lose altitude
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX    0.8f    // Max throttle used to limit lean angle so that vehicle does not lose altitude
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN             5.0f   // Min lean angle so that vehicle can maintain limited control

#define AC_ATTITUDE_CONTROL_MIN_DEFAULT                 0.1f    // minimum throttle mix default
#define AC_ATTITUDE_CONTROL_MAN_DEFAULT                 0.5f    // manual throttle mix default
#define AC_ATTITUDE_CONTROL_MAX_DEFAULT                 0.5f    // maximum throttle mix default
#define AC_ATTITUDE_CONTROL_MAX                         5.0f    // maximum throttle mix default

#define AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT             0.5f    // ratio controlling the max throttle output during competing requests of low throttle from the pilot (or autopilot) and higher throttle for attitude control.  Higher favours Attitude over pilot input
#define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT            0.15f   //  0.15~0.5 Low numbers lead to sharper response, higher numbers to softer response

#ifdef __cplusplus
// @class	Attitude_Multi
// @brief	MultiCopter Attitude control class
class Attitude_Multi {

public:
	Attitude_Multi(Motors &motors,Vector3f &gyro_filt, float dt);

	// pid accessors
	P& get_angle_roll_p() { return _p_angle_roll; }
	P& get_angle_pitch_p() { return _p_angle_pitch; }
	P& get_angle_yaw_p() { return _p_angle_yaw; }
	PID& get_rate_roll_pid() { return _pid_rate_roll; }
	PID& get_rate_pitch_pid() { return _pid_rate_pitch; }
	PID& get_rate_yaw_pid() { return _pid_rate_yaw; }

	void set_rotation_target_to_body(Matrix3f rotation_target_to_body){ _rotation_target_to_body=rotation_target_to_body;}
	void set_rotation_body_to_ned(Matrix3f rotation_body_to_ned){ _rotation_body_to_ned=rotation_body_to_ned;}
	Matrix3f get_rotation_target_to_body(){return _rotation_target_to_body;}
	Matrix3f get_rotation_target_to_ned()    {
		return _rotation_body_to_ned*_rotation_target_to_body;
	}
	Matrix3f get_dcm_rotation_body_to_ned(void) const{return _rotation_body_to_ned;}

	Vector3f get_gyro(void){return _gyro;}//°/s

	// get the roll acceleration limit in degrees/s/s or radians/s/s
	float get_accel_roll_max() const { return _accel_roll_max; }
	float get_accel_roll_max_radss() const { return radians(_accel_roll_max); }

	// Sets the roll acceleration limit in degrees/s/s
	void set_accel_roll_max(float accel_roll_max) { _accel_roll_max = accel_roll_max; }

	// Sets and saves the roll acceleration limit in degrees/s/s
	void save_accel_roll_max(float accel_roll_max) ;

	// get the pitch acceleration limit in degrees/s/s or radians/s/s
	float get_accel_pitch_max() const { return _accel_pitch_max; }
	float get_accel_pitch_max_radss() const { return radians(_accel_pitch_max); }

	// Sets the pitch acceleration limit in degrees/s/s
	void set_accel_pitch_max(float accel_pitch_max) { _accel_pitch_max = accel_pitch_max; }

	// Sets and saves the pitch acceleration limit in degrees/s/s
	void save_accel_pitch_max(float accel_pitch_max) ;

	// get the yaw acceleration limit in degrees/s/s or radians/s/s
	float get_accel_yaw_max() const { return _accel_yaw_max; }
	float get_accel_yaw_max_radss() const { return radians(_accel_yaw_max); }

	// Sets the yaw acceleration limit in degrees/s/s
	void set_accel_yaw_max(float accel_yaw_max) { _accel_yaw_max = accel_yaw_max; }

	// Sets and saves the yaw acceleration limit in degrees/s/s
	void save_accel_yaw_max(float accel_yaw_max) ;

	// set the rate control input smoothing time constant
	void set_input_tc(float input_tc) { _input_tc = constrain_float(input_tc, 0.0f, 1.0f); }

	// Ensure attitude controller have zero errors to relax rate controller output
	void relax_attitude_controllers();

	// reset rate controller I terms
	void reset_rate_controller_I_terms();

	// Sets attitude target to vehicle attitude
	void set_attitude_target_to_current_attitude() { _attitude_target_quat.from_rotation_matrix(get_rotation_target_to_ned()); }

	// Sets yaw target to vehicle heading
	void set_yaw_target_to_current_heading() { shift_ef_yaw_target(degrees(ahrs_yaw_rad() - _attitude_target_euler_angle.z)); }

	// Shifts earth frame yaw target by yaw_shift_cd. yaw_shift_cd should be in degrees and is added to the current target heading
	void shift_ef_yaw_target(float yaw_shift_cd);

	// handle reset of attitude from EKF since the last iteration
	void inertial_frame_reset();

	// Command a Quaternion attitude with feedforward and smoothing
	void input_quaternion(Quaternion attitude_desired_quat);

	// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
	void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);

	// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
	void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw);

	// Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
	void input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds);

	// Command an angular velocity with angular velocity feedforward and smoothing
	void input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

	// Command an angular velocity with angular velocity feedforward and smoothing
	void input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

	// Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
	void input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

	// Command an angular step (i.e change) in body frame angle
	void input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd);

	// Convert a 321-intrinsic euler angle derivative to an angular velocity vector
	void euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads);

	// Convert an angular velocity vector to a 321-intrinsic euler angle derivative
	// Returns false if the vehicle is pitched 90 degrees up or down
	bool ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads);

	// Specifies whether the attitude controller should use the square root controller in the attitude correction.
	// This is used during Autotune to ensure the P term is tuned without being influenced by the acceleration limit of the square root controller.
	void use_sqrt_controller(bool use_sqrt_cont) { _use_sqrt_controller = use_sqrt_cont; }

	// Return 321-intrinsic euler angles in centidegrees representing the rotation from NED earth frame to the
	// attitude controller's target attitude.
	// **NOTE** Using vector3f*deg(1) is more efficient than deg(vector3f) or deg(vector3d) because it gives the
	// same result with the fewest multiplications. Even though it may look like a bug, it is intentional. See issue 4895.
	Vector3f get_att_target_euler_d() const { return _attitude_target_euler_angle*degrees(1); }

	// Return the angle between the target thrust vector and the current thrust vector.
	float get_att_error_angle_deg() const { return degrees(_thrust_error_angle); }

	// Set x-axis angular velocity in degrees/s
	void rate_bf_roll_target(float rate_cds) { _rate_target_ang_vel.x = radians(rate_cds); }

	// Set y-axis angular velocity in degrees/s
	void rate_bf_pitch_target(float rate_cds) { _rate_target_ang_vel.y = radians(rate_cds); }

	// Set z-axis angular velocity in degrees/s
	void rate_bf_yaw_target(float rate_cds) { _rate_target_ang_vel.z = radians(rate_cds); }

	// Return roll rate step size in radians/s that results in maximum output after 4 time steps
	float max_rate_step_bf_roll();

	// Return pitch rate step size in radians/s that results in maximum output after 4 time steps
	float max_rate_step_bf_pitch();

	// Return yaw rate step size in radians/s that results in maximum output after 4 time steps
	float max_rate_step_bf_yaw();

	// Return roll step size in radians that results in maximum output after 4 time steps
	float max_angle_step_bf_roll() { return max_rate_step_bf_roll()/_p_angle_roll.kP(); }

	// Return pitch step size in radians that results in maximum output after 4 time steps
	float max_angle_step_bf_pitch() { return max_rate_step_bf_pitch()/_p_angle_pitch.kP(); }

	// Return yaw step size in radians that results in maximum output after 4 time steps
	float max_angle_step_bf_yaw() { return max_rate_step_bf_yaw()/_p_angle_yaw.kP(); }

	// Return angular velocity in radians used in the angular velocity controller
	Vector3f rate_bf_targets() const { return _rate_target_ang_vel; }

	// Enable or disable body-frame feed forward
	void bf_feedforward(bool enable_or_disable) { _rate_bf_ff_enabled = enable_or_disable; }

	// Enable or disable body-frame feed forward and save
	void bf_feedforward_save(bool enable_or_disable);

	// Return body-frame feed forward setting
	bool get_bf_feedforward() { return _rate_bf_ff_enabled; }

	// Enable or disable body-frame feed forward
	void accel_limiting(bool enable_or_disable);

	// Update Alt_Hold angle maximum
	void update_althold_lean_angle_max(float throttle_in);

	// Set output throttle
	void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff);

	// Set output throttle and disable stabilization
	void set_throttle_out_unstabilized(float throttle_in, bool reset_attitude_control, float filt_cutoff);

	// get throttle passed into attitude controller (i.e. throttle_in provided to set_throttle_out)
	float get_throttle_in() const { return _throttle_in; }

	// Return throttle increase applied for tilt compensation
	float angle_boost() const { return _angle_boost; }

	// Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
	float get_althold_lean_angle_max() const;

	// Return configured tilt angle limit in degrees
	float lean_angle_max() const { return _angle_max; }
	void set_lean_angle_max(float degree){_angle_max=degree;}

	// Proportional controller with piecewise sqrt sections to constrain second derivative
	static float sqrt_controller(float error, float p, float second_ord_lim, float dt);

	// Inverse proportional controller with piecewise sqrt sections to constrain second derivative
	static float stopping_point(float first_ord_mag, float p, float second_ord_lim);

	// calculates the velocity correction from an angle error. The angular velocity has acceleration and
	// deceleration limits including basic jerk limiting using smoothing_gain
	static float input_shaping_angle(float error_angle, float smoothing_gain, float accel_max, float target_ang_vel, float dt);

	// limits the acceleration and deceleration of a velocity request
	static float input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt);

	// calculates the expected angular velocity correction from an angle error based on the AC_AttitudeControl settings.
	// This function can be used to predict the delay associated with angle requests.
	void input_shaping_rate_predictor(Vector2f error_angle, Vector2f& target_ang_vel, float dt) const;

	// translates body frame acceleration limits to the euler axis
	void ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max, float ang_vel_pitch_max, float ang_vel_yaw_max) const;

	// translates body frame acceleration limits to the euler axis
	Vector3f euler_accel_limit(Vector3f euler_rad, Vector3f euler_accel);

	// thrust_heading_rotation_angles - calculates two ordered rotations to move the att_from_quat quaternion to the att_to_quat quaternion.
	// The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
	void thrust_heading_rotation_angles(Quaternion& att_to_quat, const Quaternion& att_from_quat, Vector3f& att_diff_angle, float& thrust_vec_dot);

	// Calculates the body frame angular velocities to follow the target attitude
	void attitude_controller_run_quat();

	// return true if the rpy mix is at lowest value
	bool is_throttle_mix_min() const { return (_throttle_rpy_mix < 1.25f*_thr_mix_min); }

	// enable use of flybass passthrough on heli
	void use_flybar_passthrough(bool passthrough, bool tail_passthrough);

	// use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage on heli
	void use_leaky_i(bool leaky_i);

	// set_hover_roll_scalar - scales Hover Roll Trim parameter. To be used by vehicle code according to vehicle condition.
	void set_hover_roll_trim_scalar(float scalar);

	// passthrough_bf_roll_pitch_rate_yaw - roll and pitch are passed through directly, body-frame rate target for yaw
	void passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf_cds);

	// enable inverted flight on backends that support it
	void set_inverted_flight(bool inverted);

    // calculate total body frame throttle required to produce the given earth frame throttle
	float get_throttle_boosted(float throttle_in);

	// set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
	//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
	//  has no effect when throttle is above hover throttle
	void set_throttle_mix_min() { _throttle_rpy_mix_desired = _thr_mix_min; }
	void set_throttle_mix_man() { _throttle_rpy_mix_desired = _thr_mix_man; }
	void set_throttle_mix_max() { _throttle_rpy_mix_desired = _thr_mix_max; }
	void set_throttle_mix_value(float value) { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
	float get_throttle_mix(void) const { return _throttle_rpy_mix; }

	// run lowest level body-frame rate controller and send outputs to the motors
	void rate_controller_run();

	//
	// UGV steering controller
	//

	// return a steering servo output given a desired lateral acceleration rate in m/s/s.
	// positive lateral acceleration is to the right.  dt should normally be the main loop rate.
	// return value is normally in range -1.0 to +1.0 but can be higher or lower
	float get_steering_out_lat_accel(float desired_accel, bool motor_limit_left, bool motor_limit_right, float dt);

	// return a steering servo output given a heading in radians
	// return value is normally in range -1.0 to +1.0 but can be higher or lower
	float get_steering_out_heading(float heading_rad, float rate_max, bool motor_limit_left, bool motor_limit_right, float dt);

	// return a steering servo output given a desired yaw rate in radians/sec.
	// positive yaw is to the right
	// return value is normally in range -1.0 to +1.0 but can be higher or lower
	float get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right, float dt);

	// get latest desired turn rate in rad/sec recorded during calls to get_steering_out_rate.  For reporting purposes only
	float get_desired_turn_rate() const;

	// get latest desired lateral acceleration in m/s/s recorded during calls to get_steering_out_lat_accel.  For reporting purposes only
	float get_desired_lat_accel() const;

	// get actual lateral acceleration in m/s/s.  returns true on success.  For reporting purposes only
	bool get_lat_accel(float &lat_accel) const;

	//
	// UGV throttle / speed controller
	//

	// set limits used by throttle controller
	//   forward/back acceleration max in m/s/s
	//   forward/back deceleartion max in m/s/s
	void set_throttle_limits(float throttle_accel_max, float throttle_decel_max);

	// return a throttle output from -1 to +1 given a desired speed in m/s (use negative speeds to travel backwards)
	//   desired_speed argument should already have been passed through get_desired_speed_accel_limited function
	//   motor_limit should be true if motors have hit their upper or lower limits
	//   cruise speed should be in m/s, cruise throttle should be a number from -1 to +1
	float get_throttle_out_speed(float desired_speed, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt);

	// return a throttle output from -1 to +1 to perform a controlled stop.  stopped is set to true once stop has been completed
	float get_throttle_out_stop(bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt, bool &stopped);

	// for balancebot
	// return a throttle output from -1 to +1 given a desired pitch angle
	// desired_pitch is in radians
	float get_throttle_out_from_pitch(float desired_pitch, bool armed, float dt);

	// low level control accessors for reporting and logging
	P& get_steering_angle_p() { return _steer_angle_p; }
	PID& get_steering_rate_pid() { return _steer_rate_pid; }
	PID& get_throttle_speed_pid() { return _throttle_speed_pid; }
	PID& get_pitch_to_throttle_pid() { return _pitch_to_throttle_pid; }

	// get forward speed in m/s (earth-frame horizontal velocity but only along vehicle x-axis).  returns true on success
	bool get_forward_speed(float &speed) const;

	// get throttle/speed controller maximum acceleration (also used for deceleration)
	float get_accel_max() const { return MAX(_throttle_accel_max, 0.0f); }

	// get throttle/speed controller maximum deceleration
	float get_decel_max() const;

	// check if speed controller active
	bool speed_control_active() const;

	// get latest desired speed recorded during call to get_throttle_out_speed.  For reporting purposes only
	float get_desired_speed() const;

	// get acceleration limited desired speed
	float get_desired_speed_accel_limited(float desired_speed, float dt) const;

	// get minimum stopping distance (in meters) given a speed (in m/s)
	float get_stopping_distance(float speed) const;

protected:

	// Update rate_target_ang_vel using attitude_error_rot_vec_rad
	Vector3f update_ang_vel_target_from_att_error(Vector3f attitude_error_rot_vec_rad);

	// Run the roll angular velocity PID controller and return the output
	float rate_target_to_motor_roll(float rate_actual_rads, float rate_target_rads);

	// Run the pitch angular velocity PID controller and return the output
	float rate_target_to_motor_pitch(float rate_actual_rads, float rate_target_rads);

	// Run the yaw angular velocity PID controller and return the output
	float rate_target_to_motor_yaw(float rate_actual_rads, float rate_target_rads);

	// Return angle in radians to be added to roll angle. Used by heli to counteract
	// tail rotor thrust in hover. Overloaded by AC_Attitude_Heli to return angle.
	float get_roll_trim_rad() { return 0;}

	// Return the yaw slew rate limit in radians/s
	float get_slew_yaw_rads() { return radians(_slew_yaw); }

	// update_throttle_rpy_mix - updates thr_low_comp value towards the target
	void update_throttle_rpy_mix();

	// get maximum value throttle can be raised to based on throttle vs attitude prioritisation
	float get_throttle_avg_max(float throttle_in);

	// Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
	float            _slew_yaw;

	Matrix3f _rotation_body_to_ned;
	Matrix3f _rotation_target_to_body;

	float    _angle_max;

	// Maximum angular velocity (in degrees/second) for earth-frame roll, pitch and yaw axis
	float            _ang_vel_roll_max;
	float            _ang_vel_pitch_max;
	float            _ang_vel_yaw_max;

	// Maximum rotation acceleration for earth-frame roll axis
	float            _accel_roll_max;

	// Maximum rotation acceleration for earth-frame pitch axis
	float            _accel_pitch_max;

	// Maximum rotation acceleration for earth-frame yaw axis
	float            _accel_yaw_max;

	// Enable/Disable body frame rate feed forward
	bool             _rate_bf_ff_enabled;

	// Enable/Disable angle boost
	bool             _angle_boost_enabled;

	// angle controller P objects
	P                _p_angle_roll;
	P                _p_angle_pitch;
	P                _p_angle_yaw;

	// Angle limit time constant (to maintain altitude)
	float            _angle_limit_tc;

	// rate controller input smoothing time constant
	float            _input_tc;

	// Intersampling period in seconds
	float               _dt;

	// This represents a 321-intrinsic rotation in NED frame to the target (setpoint)
	// attitude used in the attitude controller, in radians.
	Vector3f            _attitude_target_euler_angle;

	// This represents the angular velocity of the target (setpoint) attitude used in
	// the attitude controller as 321-intrinsic euler angle derivatives, in radians per
	// second.
	Vector3f            _attitude_target_euler_rate;

	// This represents a quaternion rotation in NED frame to the target (setpoint)
	// attitude used in the attitude controller.
	Quaternion          _attitude_target_quat;

	// This represents the angular velocity of the target (setpoint) attitude used in
	// the attitude controller as an angular velocity vector, in radians per second in
	// the target attitude frame.
	Vector3f            _attitude_target_ang_vel;

	// This represents the angular velocity in radians per second in the body frame, used in the angular
	// velocity controller.
	Vector3f            _rate_target_ang_vel;

	// This represents a quaternion attitude error in the body frame, used for inertial frame reset handling.
	Quaternion          _attitude_ang_error;

	// The angle between the target thrust vector and the current thrust vector.
	float               _thrust_error_angle;

	// throttle provided as input to attitude controller.  This does not include angle boost.
	float               _throttle_in = 0.0f;

	// This represents the throttle increase applied for tilt compensation.
	// Used only for logging.
	float               _angle_boost;

	// Specifies whether the attitude controller should use the square root controller in the attitude correction.
	// This is used during Autotune to ensure the P term is tuned without being influenced by the acceleration limit of the square root controller.
	bool                _use_sqrt_controller;

	// Filtered Alt_Hold lean angle max - used to limit lean angle when throttle is saturated using Alt_Hold
	float               _althold_lean_angle_max = 0.0f;

	// desired throttle_low_comp value, actual throttle_low_comp is slewed towards this value over 1~2 seconds
	float               _throttle_rpy_mix_desired;

	// mix between throttle and hover throttle for 0 to 1 and ratio above hover throttle for >1
	float               _throttle_rpy_mix;

	PID                _pid_rate_roll;
	PID                _pid_rate_pitch;
	PID                _pid_rate_yaw;

	float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
	float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
	float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

	Motors&            _motors;
	Vector3f&          _gyro;

	// UGV parameters
	P     _steer_angle_p;        // steering angle controller
	PID   _steer_rate_pid;       // steering rate controller
	PID   _throttle_speed_pid;   // throttle speed controller
	PID   _pitch_to_throttle_pid;// balancebot pitch controller

	float _throttle_accel_max;   // speed/throttle control acceleration (and deceleration) maximum in m/s/s.  0 to disable limits
	float _throttle_decel_max;    // speed/throttle control deceleration maximum in m/s/s. 0 to use ATC_ACCEL_MAX for deceleration
	int8_t  _brake_enable;         // speed control brake enable/disable. if set to 1 a reversed output to the motors to slow the vehicle.
	float _stop_speed;           // speed control stop speed.  Motor outputs to zero once vehicle speed falls below this value
	float _steer_accel_max;      // steering angle acceleration max in deg/s/s
	float _steer_rate_max;       // steering rate control maximum rate in deg/s

	// steering control
	uint32_t _steer_lat_accel_last_ms;  // system time of last call to lateral acceleration controller (i.e. get_steering_out_lat_accel)
	uint32_t _steer_turn_last_ms;   // system time of last call to steering rate controller
	float    _desired_lat_accel;    // desired lateral acceleration (in m/s/s) from latest call to get_steering_out_lat_accel (for reporting purposes)
	float    _desired_turn_rate;    // desired turn rate (in radians/sec) either from external caller or from lateral acceleration controller

	// throttle control
	uint32_t _speed_last_ms;        // system time of last call to get_throttle_out_speed
	float    _desired_speed;        // last recorded desired speed
	uint32_t _stop_last_ms;         // system time the vehicle was at a complete stop
	bool     _throttle_limit_low;   // throttle output was limited from going too low (used to reduce i-term buildup)
	bool     _throttle_limit_high;  // throttle output was limited from going too high (used to reduce i-term buildup)

	// balancebot pitch control
	uint32_t _balance_last_ms = 0;

};
#endif /* __cplusplus */

#endif /* __ATTITUDE_H */
