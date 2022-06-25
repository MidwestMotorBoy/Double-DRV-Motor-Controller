/*
 * FOC.h
 *
 *  Created on: Oct 1, 2019
 *      Author: MidwestMotorBoy
 */

#ifndef FOC_H_
#define FOC_H_

#include "stm32h7xx_hal.h"


int rawdata_to_angle(int rawdata);
void parkclark(int Ia,int Ib,int Ic,int theta,int *Iq,int *Id );
void inv_parkclark(int *Va,int *Vb,int *Vc,int theta,int Vq,int Vd );
int sin_lut(int angle);
int cos_lut(int angle);
int find_min_voltage(int32_t Va, int32_t Vb, int32_t Vc);
#endif /* FOC_H_ */
