/*
 * pid.h
 *
 *  Created on: Jul 28, 2024
 *      Author: Ritesh Kumar
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    float p_gain, i_gain, d_gain, max;
} PID_Gains;

typedef struct {
	PID_Gains roll, pitch, yaw;
} PID_Constant;

typedef struct {
	float roll, pitch, yaw;
} PID_Params;

typedef struct {
	PID_Params output, setpoint;
} PID_Controller;

void calculate_pid(PID_Controller *pid, float roll_rate, float pitch_rate, float yaw_rate);
void flush_pid(void);

#endif /* INC_PID_H_ */
