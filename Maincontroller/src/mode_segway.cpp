/*
 * mode_segway.cpp
 *
 */
#include "maincontroller.h"

/*
 * 注意本demo文件都是新定义的函数，在工程里面使用前都要在hal.h里面声明一下,具体方式可以咨询我们
 * */

//默认遥控器为左手油门
static float channel_roll = 0.0; //滚转角通道控制左右 y
static float channel_yaw = 0.0; //偏航角控制旋转 rotate
static float channel_pitch = 0.0;  //俯仰角通道控制前后 x
static float channel_throttle=0.0;//油门通道控制是否启动电机
static float target_yaw=0.0f;
static float target_pitch=0.0f;
static float target_roll=0.0f;
static float control_yaw = 0.0f;
static float control_pitch = 0.0f;
static float control_left = 0.0f;
static float control_right = 0.0f;

//初始化函数
bool mode_segway_init(void){
	//设置码盘引脚读取模式
	set_gpio1_mode(GPIO_MODE_IT_RISING_FALLING);//左侧编码器A相，中断输入双边延触发
	set_gpio2_mode(GPIO_MODE_IT_RISING_FALLING);//左侧编码器B相，中断输入双边延触发

	set_gpio5_mode(GPIO_MODE_IT_RISING_FALLING);//右侧编码器A相，中断输入双边延触发
	set_gpio6_mode(GPIO_MODE_IT_RISING_FALLING);//右侧编码器B相，中断输入双边延触发

	//这里用了SDK里的P-PID串级控制环,外环角度环为P控制,内环角速度环为PID控制,也可以自己写控制环,demo有PID的自定义例程
	attitude->get_steering_angle_p().kP(AR_ATTCONTROL_STEER_ANG_P);//设置偏航控制外环的P参数,可以自己调整
	attitude->get_steering_rate_pid().kP(AR_ATTCONTROL_STEER_RATE_P);//设置偏航控制外环的P参数,可以自己调整
	attitude->get_steering_rate_pid().kI(AR_ATTCONTROL_STEER_RATE_I);//设置偏航控制内环的I参数,可以自己调整
	attitude->get_steering_rate_pid().kD(AR_ATTCONTROL_STEER_RATE_D);//设置偏航控制内环的D参数,可以自己调整
	attitude->get_steering_rate_pid().set_dt(_dt);//设置PID周期,即PID函数的调用频率
	attitude->get_steering_rate_pid().filt_hz(AR_ATTCONTROL_STEER_RATE_FILT);//设置偏航控制内环的PID低通滤波截止频率,可以自己调整
	attitude->get_steering_rate_pid().ff(AR_ATTCONTROL_STEER_RATE_FF);//PID前馈系数,补偿输出值为：前馈系数*目标值,可以自己调整

	attitude->get_pitch_to_throttle_pid().kP(AR_ATTCONTROL_PITCH_THR_P);//设置俯仰控制外环的P参数,可以自己调整
	attitude->get_pitch_to_throttle_pid().kI(AR_ATTCONTROL_PITCH_THR_I);//设置俯仰控制内环的I参数,可以自己调整
	attitude->get_pitch_to_throttle_pid().kD(AR_ATTCONTROL_PITCH_THR_D);//设置俯仰控制内环的D参数,可以自己调整
	attitude->get_pitch_to_throttle_pid().set_dt(_dt);//设置PID周期,即PID函数的调用频率
	attitude->get_pitch_to_throttle_pid().filt_hz(AR_ATTCONTROL_PITCH_THR_FILT);//设置俯仰控制内环的PID低通滤波截止频率,可以自己调整

	channel_roll = 0.0;
	channel_yaw = 0.0;
	channel_pitch = 0.0;
	channel_throttle=0.0;

	target_yaw = ahrs_yaw_deg();//重置目标偏航角

	Buzzer_set_ring_type(BUZZER_MODE_SWITCH);
	usb_printf("switch mode segway!\n");
	return true;
}

//编码器计数函数，把它添加进freertos.c的gpio回调函数里,函数内容需要自己按实际情况调整,我们采用四倍频计数方法
static int32_t left_count = 0;//左编码器
static int32_t right_count = 0;//右编码器
static int32_t left_count_last = 0;//左编码器
static int32_t right_count_last = 0;//右编码器
void check_wheel_count_gpio1(void){
	if(read_gpio1()){//A相上升沿触发
		 if(read_gpio2()){
			 left_count--;//B相位于高电平 向下计数
		 }else{
			 left_count++;//B相位于低电平 向上计数
		 }
	}else{//A相下降沿触发
		if(read_gpio2()){
			left_count++;//B相位于高电平 向上计数
		}else{
			left_count--;//B相位于低电平 向下计数
		}
	}
}
void check_wheel_count_gpio2(void){
	if(read_gpio2()){//B相上升沿触发
		 if(read_gpio1()){
			 left_count++;//A相位于高电平 向上计数
		 }else{
			 left_count--;//A相位于低电平 向下计数
		 }
	}else{//B相下降沿触发
		if(read_gpio1()){
			left_count--;//A相位于高电平 向下计数
		}else{
			left_count++;//A相位于低电平 向上计数
		}
	}
}
void check_wheel_count_gpio5(void){
	if(read_gpio5()){//A相上升沿触发
		 if(read_gpio6()){
			 right_count--;//B相位于高电平 向下计数
		 }else{
			 right_count++;//B相位于低电平 向上计数
		 }
	}else{//A相下降沿触发
		if(read_gpio6()){
			right_count++;//B相位于高电平 向上计数
		}else{
			right_count--;//B相位于低电平 向下计数
		}
	}
}
void check_wheel_count_gpio6(void){
	if(read_gpio5()){//B相上升沿触发
		 if(read_gpio6()){
			 right_count++;//A相位于高电平 向上计数
		 }else{
			 right_count--;//A相位于低电平 向下计数
		 }
	}else{//B相下降沿触发
		if(read_gpio6()){
			right_count--;//A相位于高电平 向下计数
		}else{
			right_count++;//A相位于低电平 向上计数
		}
	}
}

//通过编码器计数来计车轮速度,要自己按实际情况调整
//注意！硬件解锁后才能读取编码器
static float vel_left=0.0f;//左轮转速
static float vel_right=0.0f;//右轮转速
static void cal_vel(void){
	usb_printf("count:%d|%d\n",left_count,right_count);//在usb口打印,看看计数是否正确
	float	delta_num_left = (float)(left_count - left_count_last); //编码器数据
	float	delta_num_right = (float)(right_count - right_count_last);
	vel_left = delta_num_left * 20 / 24480 / 2.5 * 1000;//速度计算,具体算法按照自己的编码器电机参数、车轮直径调整,默认本函数以400hz频率运行,周期为2.5ms
	vel_right = delta_num_right * 20 / 24480 / 2.5 * 1000;
	left_count_last = left_count;
	right_count_last = right_count;
	usb_printf("vel:%f|%f\n",vel_left,vel_right);//在usb口打印,看看测速是否正确
}

void mode_segway(void){//平衡车控制主函数,默认以400hz频率运行,周期0.0025s
	robot_state=STATE_DRIVE;
	//(1)首先获取遥控器通道值
	channel_roll = get_channel_roll(); //range(-1,1)
	channel_yaw = get_channel_yaw(); //range(-1,1)
	channel_pitch = get_channel_pitch(); //range(-1,1)
	channel_throttle=get_channel_throttle();  //range(0-1)

	//(2)获取编码器测速
	cal_vel();

	//(3)判断是否有输出
	if(get_soft_armed()&&(channel_throttle>0.1)){//手势解锁后,油门推起来才允许电机输出
		//获取目标偏航角
		float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());//获取摇杆设置的目标偏航角速度
		target_yaw+=target_yaw_rate*_dt;
		//这里用了SDK里的P-PID串级控制环,外环角度环为P控制,内环角速度环为PID控制,也可以自己写控制环,demo有PID的自定义例程
		control_yaw = attitude->get_steering_out_heading(target_yaw*DEG_TO_RAD, M_PI/2, false, false, _dt);//输出归一化偏航控制量-1~1
		//获取目标俯仰角degree,这里用遥控器手动控制平衡车俯仰角
		get_pilot_desired_lean_angles(target_roll, target_pitch, param->angle_max.value, param->angle_max.value);

		//留个作业：这里可以构建一个P-PID控制环,利用编码器数据实现位置控制,位置控制的输出是下面控制环的目标俯仰角,如何自定义P和PID可以参考demo_pid.cpp。

		//用SDK里面的PID控制环控制俯仰，也就是前后移动,输入目标俯仰角rad,输出油门控制量
		control_pitch = attitude->get_throttle_out_from_pitch(target_pitch*DEG_TO_RAD, true, _dt);//输出归一化偏航控制量-1~1

		//注意这里每个变量的正负号都和电机安装方式有关，自己调整
		control_left=control_pitch+control_yaw;
		control_right=control_pitch-control_yaw;

		//限幅
		control_left=constrain_float(control_left, -1.0, 1.0);
		control_right=constrain_float(control_right, -1.0, 1.0);

		//电机输出,我们假设用两个pwm口控制一个有刷电机的正反转,实际情况如有不同可以跟我们反馈
		//用M3，M4控制左侧电机；用M7，M8控制右侧电机；
		//默认PWM频率为400hz,即周期为2500us,所以最大输出脉宽就是2500
		if(control_left){//正转
			Motor_Set_Value(3, (uint16_t)(control_left*PWM_BRUSH_MAX));
			Motor_Set_Value(4, 0);
		}else{//反转
			Motor_Set_Value(3, 0);
			Motor_Set_Value(4, (uint16_t)(-control_left*PWM_BRUSH_MAX));
		}
		if(control_right){//正转
			Motor_Set_Value(7, (uint16_t)(control_right*PWM_BRUSH_MAX));
			Motor_Set_Value(8, 0);
		}else{//反转
			Motor_Set_Value(7, 0);
			Motor_Set_Value(8, (uint16_t)(-control_right*PWM_BRUSH_MAX));
		}

	}else{
		target_yaw = ahrs_yaw_deg();//重置目标偏航角
		attitude->get_steering_rate_pid().reset_I();//重置PID的积分
		control_pitch = attitude->get_throttle_out_from_pitch(0.0f, false, _dt);//重置油门
		Motor_Set_Value(3, 0);
		Motor_Set_Value(4, 0);
		Motor_Set_Value(7, 0);
		Motor_Set_Value(8, 0);
	}

}
