/*
 * PI Loop.hpp
 *
 *  Created on: Oct 1, 2019
 *      Author: MidwestMotorBoy
 */

#ifndef PI_LOOP_H_
#define PI_LOOP_H_
#include "stm32h7xx_hal.h"

void update_foc_pi(float *target_iq, float *target_id,float *cur_iq,float *cur_id,int *Vq,int *Vd);
void slew(float max_change,float pre_slewed,float *curr_val);
void update_vs_pi(int target_vout, int cur_vq, int cur_vd,float *Id);

#endif /* PI_LOOP_H_ */
