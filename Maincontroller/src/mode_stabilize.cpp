/*
 * mode_stabilize.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

bool mode_stabilize_init(void){
	// if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
	if (motors->get_armed() && ap->land_complete && !has_manual_throttle() &&
			(get_pilot_desired_throttle(get_channel_throttle(), 0.0f) > get_non_takeoff_throttle())) {
		Buzzer_set_ring_type(BUZZER_ERROR);
		return false;
	}
	// set target altitude to zero for reporting
	pos_control->set_alt_target(0);
	set_manual_throttle(true);//设置为手动油门
	Buzzer_set_ring_type(BUZZER_MODE_SWITCH);
	usb_printf("switch mode stabilize!\n");
	return true;
}

static int16_t esc_counter=0, esc_delay=0;
static float target_yaw=0.0f;
void mode_stabilize(void){
	float target_roll, target_pitch;
	float target_yaw_rate;
	float pilot_throttle_scaled;

	// if not armed set throttle to zero and exit immediately
	if (!motors->get_armed() || ap->throttle_zero || !motors->get_interlock()) {
		zero_throttle_and_relax_ac();
		attitude->rate_controller_run();
		target_yaw=ahrs_yaw_deg();
		motors->output();
		robot_state=STATE_STOP;
		/*
		 * 以下为电调校准模式说明:
		 * (1) 在channel8电机硬件锁定状态下, 将油门推到最高, 偏航推到最左, 持续5s进入电调校准模式;
		 * (2) 进入电调校准模式后提示音“嘟嘟嘟...”响起。此时将偏航回中, channel8解锁电机硬件, Mcontroller M1~M8插口会产生PWM输出;
		 * (3) 在电调校准模式中, Mcontroller M1~M8口输出的PWM波脉宽直接由油门推杆控制, 即最大油门对应最大脉宽, 最小油门对应最小脉宽。
		 * (4) 电调校准模式默认持续时间为20s, 即进入电调校准模式20s后自动退出电调校准模式;
		 * (5) 在电调校准模式中, 将偏航推到最右可以立即退出电调校准模式;
		 * */
		float throttle=get_channel_throttle();
		float tmp = get_channel_yaw();
		if (tmp < -0.9) { //full left
			// wait for 5s to enter esc correct mode
			if( throttle > 0.9 && esc_counter < 2000 && !motors->get_interlock()) {
				esc_counter++;
			}
			if (esc_counter == 2000) {
				esc_delay=8000; //设置电调校准持续时间为20s, 20s后自动退出电调校准模式
				Buzzer_set_ring_type(BUZZER_ESC);
			}
		}else if(tmp>0.5){	// Yaw is right to reset esc counter
			esc_counter = 0;
		}else{				// Yaw is centered to delay 20s to reset esc counter
			// correct the esc
			if (esc_counter == 2000) {
				Buzzer_set_ring_type(BUZZER_ESC);
				motors->set_throttle_passthrough_for_motors(throttle);//只校准当前机型使能的电机
			}
			if(esc_delay>0){
				esc_delay--;
			}else{
				esc_counter = 0;
				esc_delay = 0;
			}
		}
		return;
	}
	robot_state=STATE_FLYING;
	// clear landing flag
	set_land_complete(false);

	motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

	// convert pilot input to lean angles
	get_pilot_desired_lean_angles(target_roll, target_pitch, param->angle_max.value, param->angle_max.value);

	// get pilot's desired yaw rate
	target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());

	// get pilot's desired throttle
	pilot_throttle_scaled = get_pilot_desired_throttle(get_channel_throttle(),0.0f);

	// call attitude controller
	target_yaw+=target_yaw_rate*_dt;
	attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);

	// output pilot's throttle
	attitude->set_throttle_out(pilot_throttle_scaled, true, param->throttle_filt.value);

	attitude->rate_controller_run();
	motors->output();
}
