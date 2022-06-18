/*
 * PI Loop.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: Logan Rosenmayer
 */
#include "PI_Loop.h"
#define MAX_ERROR
#define KI_FOC 10.0f
#define KP_FOC 0.5f
#define time_between_update_foc 0.001f
#define MAX_V 1000
#define INTERRORQ_MAX MAX_V/KI_FOC
static float interrorq = 0;
static float interrord = 0;
static float errorq_prev = 0;
static float errord_prev = 0;
static uint32_t last_tic = 0;
float slewed_target = 0;
float slewed_iq = 0, slewed_id = 0;

void update_foc_pi(float *target_iq, float *target_id, float *cur_iq,
		float *cur_id, int *Vq, int *Vd) {
	slew(0.5, *target_iq, &slewed_iq);
	slew(0.5, *target_id, &slewed_id);
	float errorq = slewed_iq - *cur_iq;
	float errord = slewed_id - *cur_id;
	interrorq += errorq * time_between_update_foc;
	interrord += errord * time_between_update_foc;
	errorq_prev = errorq; //update previous error term
	errord_prev = errord; //update previous error term
	if (interrorq > INTERRORQ_MAX) {
		interrorq = INTERRORQ_MAX;
	} else if (interrorq < -INTERRORQ_MAX) {
		interrorq = -INTERRORQ_MAX;
	}
	if (interrord > INTERRORQ_MAX) {
		interrord = INTERRORQ_MAX;
	} else if (interrord < -INTERRORQ_MAX) {
		interrord = -INTERRORQ_MAX;
	}
	//TODO: pwm saturates at 100% but drivers should not operate 100% so some protection must be added
	*Vq = (int) (interrorq * KI_FOC + errorq * KP_FOC);
	*Vd = (int) (interrord * KI_FOC + errord * KP_FOC);
	if (*Vq > MAX_V) {
		*Vq = MAX_V;
	} else if (*Vq < -MAX_V) {
		*Vq = -MAX_V;
	}
	if (*Vd > MAX_V) {
		*Vd = MAX_V;
	} else if (*Vd < -MAX_V) {
		*Vd = -MAX_V;
	}
}

#define KI_SPEED 0.01f
#define KP_SPEED 2.0f
#define KD_SPEED 0.003f
#define IQ_MAX 50
static float interrorspeed = 0;
static float errorspeed_prev = 0;
static float speed_diff = 0;
int max_speed_int = IQ_MAX / KI_SPEED;

void update_speed_pi(float *target_speed, float *cur_speed, float *Iq) {
	float output, speed_error_der;
	slew(10, *target_speed, &slewed_target);
	float error_speed = slewed_target - *cur_speed;
	interrorspeed += error_speed * 0.01;
	if (interrorspeed > max_speed_int) {
		interrorspeed = max_speed_int;
	} else if (interrorspeed < -max_speed_int) {
		interrorspeed = -max_speed_int;
	}
	output = (interrorspeed * KI_SPEED + error_speed * KP_SPEED);
	if (output > IQ_MAX) {
		*Iq = IQ_MAX;
	} else if (output < -IQ_MAX) {
		*Iq = -IQ_MAX;
	} else {
		*Iq = output;
	}
}

#define KI_ID 1.0f
#define KP_ID 0.5f
#define ID_MAX 60
int interrorvs = 0;
int max_interrorvs = ID_MAX / KI_ID;
void update_vs_pi(int target_vout, int cur_vq, int cur_vd, float *Id) {
	float output;
	int error_vs = sqrt(cur_vq * cur_vq + cur_vd * cur_vd) - target_vout;
	interrorvs += error_vs * 0.001;
	if (interrorvs > max_interrorvs) {
		interrorvs = max_interrorvs;
	} else if (interrorvs < 0) {
		interrorvs = 0;
	}
	output = (interrorvs * KI_ID + error_vs * KP_ID);
	if (output > ID_MAX) {
		*Id = ID_MAX;
	} else if (output < 0) {
		*Id = 0;
	} else {
		*Id = output;
	}
}
void slew(float max_change, float pre_slewed, float *curr_val) {
	float diff = pre_slewed - *curr_val;
	if (diff > max_change) {
		*curr_val += max_change;
	} else if (diff < -max_change) {
		*curr_val -= max_change;
	} else {
		*curr_val = pre_slewed;
	}
}
/*
 Pi_loop(float maxval,float minval,float kp_in,float ki_in,int ticks_per_second_in){
 max_val = maxval;
 min_val = minval;
 kp = kp_in;
 ki = ki_in;
 ticks_per_second = ticks_per_second_in;
 }
 void target(float newtarget){
 target = newtarget;
 }
 float get_val(void){
 return(curr_val);
 }
 void update_loop(float measured){
 long time_delta=HAL_GetTick()-last_tick;
 last_tick+=delta;
 float error = target-measured;
 error_intgral = error*time_delta/TICKS_PER_SECOND;
 curr_val=error_intgral*kp+error*ki;
 if(curr_val>max_val){
 curr_val=max_val;
 }
 if(curr_val<min_val){
 curr_val=min_val;
 }
 }
 */
