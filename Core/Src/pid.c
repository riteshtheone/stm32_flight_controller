/*
 * pid.c
 *
 *  Created on: Jul 28, 2024
 *      Author: Ritesh Kumar
 */

#include "pid.h"

static PID_Constant pid_const = {
		.roll  = {0.4f, 0.002f, 10.0f, 400.0f},
		.pitch = {0.4f, 0.002f, 10.0f, 400.0f},
		.yaw   = {4.0f, 0.008f, 0.0f, 400.0f}
};

static float pid_error_temp, pid_i_mem_roll, pid_i_mem_pitch, pid_i_mem_yaw ;
static float pid_last_roll_d_error, pid_last_pitch_d_error, pid_last_yaw_d_error;

void flush_pid(void) {
	pid_i_mem_roll = 0;
	pid_last_roll_d_error = 0;
	pid_i_mem_pitch = 0;
	pid_last_pitch_d_error = 0;
	pid_i_mem_yaw = 0;
	pid_last_yaw_d_error = 0;
}

void calculate_pid(PID_Controller *pid, float roll_rate, float pitch_rate, float yaw_rate) {

	//Roll calculations
	pid_error_temp = roll_rate - pid->setpoint.roll;
	pid_i_mem_roll += pid_const.roll.i_gain * pid_error_temp;
	if (pid_i_mem_roll > pid_const.roll.max) pid_i_mem_roll = pid_const.roll.max;
	else if (pid_i_mem_roll < pid_const.roll.max * -1) pid_i_mem_roll = pid_const.roll.max * -1;

	pid->output.roll = pid_const.roll.p_gain * pid_error_temp + pid_i_mem_roll + pid_const.roll.d_gain * (pid_error_temp - pid_last_roll_d_error);
	if (pid->output.roll > pid_const.roll.max) pid->output.roll = pid_const.roll.max;
	else if (pid->output.roll < pid_const.roll.max * -1) pid->output.roll = pid_const.roll.max * -1;

	pid_last_roll_d_error = pid_error_temp;

	//Pitch calculations
	pid_error_temp = pitch_rate - pid->setpoint.pitch;
	pid_i_mem_pitch += pid_const.pitch.i_gain * pid_error_temp;
	if (pid_i_mem_pitch > pid_const.pitch.max) pid_i_mem_pitch = pid_const.pitch.max;
	else if (pid_i_mem_pitch < pid_const.pitch.max * -1) pid_i_mem_pitch = pid_const.pitch.max * -1;

	pid->output.pitch = pid_const.pitch.p_gain * pid_error_temp + pid_i_mem_pitch + pid_const.pitch.d_gain * (pid_error_temp - pid_last_pitch_d_error);
	if (pid->output.pitch > pid_const.pitch.max) pid->output.pitch = pid_const.pitch.max;
	else if (pid->output.pitch < pid_const.pitch.max * -1) pid->output.pitch = pid_const.pitch.max * -1;

	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = yaw_rate - pid->setpoint.yaw;
	pid_i_mem_yaw += pid_const.yaw.i_gain * pid_error_temp;
	if (pid_i_mem_yaw > pid_const.yaw.max) pid_i_mem_yaw = pid_const.yaw.max;
	else if (pid_i_mem_yaw < pid_const.yaw.max * -1) pid_i_mem_yaw = pid_const.yaw.max * -1;

	pid->output.yaw = pid_const.yaw.p_gain * pid_error_temp + pid_i_mem_yaw + pid_const.yaw.d_gain * (pid_error_temp - pid_last_yaw_d_error);
	if (pid->output.yaw > pid_const.yaw.max) pid->output.yaw = pid_const.yaw.max;
	else if (pid->output.yaw < pid_const.yaw.max * -1) pid->output.yaw = pid_const.yaw.max * -1;

	pid_last_yaw_d_error = pid_error_temp;
}
