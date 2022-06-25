/*
 * pos_offset_cal.c
 *
 *  Created on: May 19, 2022
 *      Author: MidwestMotorBoy
 */

/*********************************************************************
 * Includes
 * ******************************************************************/
#include "pos_offset_cal.h"
#include "FOC.h"
#include "drv8350s.h"
#include "controller_cfg.h"
#include <math.h>
/*********************************************************************
 * Defines
 * ******************************************************************/
#define LOOPS_TO_RUN 5
#define LOOP_SIZE 1024 //Size of sin lut
#define NUMBER_OF_STEPS_PER_LOOP 6
#define WAIT_TIME 300 //ms
#define E_DEGREE_CHANGE_IN_LOOP 360.0f
#define E_DEGREE_CHANGE_IN_STEP E_DEGREE_CHANGE_IN_LOOP/NUMBER_OF_STEPS_PER_LOOP
#define ZERO_POS 90.0f

/*********************************************************************
 * GLOBAL VARIABLES
 * ******************************************************************/


/*********************************************************************
 * Functions Implimentation
 * ******************************************************************/

SENSOR_OFFSET_E get_pos_offset(TIM_TypeDef *timer, SPI_HandleTypeDef *encoder_spi){
	SENSOR_OFFSET_E ret_val;
	int Va, Vb, Vc, min_val;
	int32_t tim_period = timer->ARR;
	uint32_t samples = LOOPS_TO_RUN * NUMBER_OF_STEPS_PER_LOOP;
	float positions_forward[LOOPS_TO_RUN][NUMBER_OF_STEPS_PER_LOOP];
	float positions_backwards[LOOPS_TO_RUN][NUMBER_OF_STEPS_PER_LOOP];
	int32_t locations[NUMBER_OF_STEPS_PER_LOOP];
	float avg_pos[NUMBER_OF_STEPS_PER_LOOP] = {0};
	uint16_t position;
	int request_pos = 0xffff;

	for(int x=0; x < NUMBER_OF_STEPS_PER_LOOP; x++){
		locations[x] = (LOOP_SIZE * x) / NUMBER_OF_STEPS_PER_LOOP;
	}

	HAL_SPI_TransmitReceive(encoder_spi, (uint8_t*) &request_pos, (uint8_t*) &position, 1, 100);
	for(int32_t loop_cnt=0; loop_cnt < LOOPS_TO_RUN; loop_cnt++){
		for(int32_t step_cnt=0; step_cnt < NUMBER_OF_STEPS_PER_LOOP; step_cnt++){
			inv_parkclark(&Va, &Vb, &Vc, locations[step_cnt], POS_OFFSET_TEST_V, 0);
			min_val = find_min_voltage(Va, Vb, Vc);
			Va -= min_val;
			Vb -= min_val;
			Vc -= min_val;
			timer->CCR1 = (Va*tim_period)/DC_BUS_V;
			timer->CCR2 = (Vb*tim_period)/DC_BUS_V;
			timer->CCR3 = (Vc*tim_period)/DC_BUS_V;
			HAL_Delay(WAIT_TIME);
			HAL_SPI_TransmitReceive(encoder_spi, (uint8_t*) &request_pos, (uint8_t*) &position, 1, 100);
			positions_forward[loop_cnt][step_cnt] = (position >> 4) * 0.087890625f;
		}
	}
	timer->CCR1 = 0;
	timer->CCR2 = 0;
	timer->CCR3 = 0;
	HAL_Delay(1000);
	for(int32_t loop_cnt=0; loop_cnt < LOOPS_TO_RUN; loop_cnt++){
		for(int32_t step_cnt = NUMBER_OF_STEPS_PER_LOOP - 1; step_cnt >= 0; step_cnt -= 1){
			inv_parkclark(&Va, &Vb, &Vc, locations[step_cnt], POS_OFFSET_TEST_V, 0);
			min_val = find_min_voltage(Va, Vb, Vc);
			Va -= min_val;
			Vb -= min_val;
			Vc -= min_val;
			timer->CCR1 = (Va*tim_period)/DC_BUS_V;
			timer->CCR2 = (Vb*tim_period)/DC_BUS_V;
			timer->CCR3 = (Vc*tim_period)/DC_BUS_V;
			HAL_Delay(WAIT_TIME);
			HAL_SPI_TransmitReceive(encoder_spi, (uint8_t*) &request_pos, (uint8_t*) &position, 1, 100);
			positions_backwards[loop_cnt][step_cnt] = (position >> 4) * 0.087890625f;
		}
	}
	timer->CCR1 = 0;
	timer->CCR2 = 0;
	timer->CCR3 = 0;
	HAL_Delay(1000);

	//Lets do a bunch of math (Kill me)
	int sign = 1;
	ret_val.direction = DIR_CW;
	ret_val.pole_pairs = 0;
	ret_val.offset = 0;
	float offset_mdeg = 0.0f;
	float step_size_mech = positions_forward[0][1] - positions_forward[0][0];

	if(fabs(step_size_mech) > E_DEGREE_CHANGE_IN_STEP*2){
		if(step_size_mech < 0){
		step_size_mech += 360;
		}else{
		step_size_mech -= 360;
		}
	}
	if(step_size_mech < 0){
		step_size_mech=-step_size_mech;
		ret_val.direction = DIR_CCW;
		sign= -1;
	}
	ret_val.pole_pairs = (int)E_DEGREE_CHANGE_IN_STEP/step_size_mech+0.5;
	float mech_degrees_in_loop = E_DEGREE_CHANGE_IN_LOOP/ret_val.pole_pairs;
	for(int32_t loop_cnt=0; loop_cnt < LOOPS_TO_RUN; loop_cnt++){
		for(int32_t step_cnt=0; step_cnt < NUMBER_OF_STEPS_PER_LOOP; step_cnt++){
			avg_pos[step_cnt] += fmod(positions_forward[loop_cnt][step_cnt],mech_degrees_in_loop);
			avg_pos[step_cnt] += fmod(positions_backwards[loop_cnt][step_cnt],mech_degrees_in_loop);
		}
	}
	for(int32_t step_cnt=0; step_cnt < NUMBER_OF_STEPS_PER_LOOP; step_cnt++){
		avg_pos[step_cnt] /= LOOPS_TO_RUN*2;
	}
	float normalized_offset =0;
	for(int32_t idx=0; idx < NUMBER_OF_STEPS_PER_LOOP; idx++){
		normalized_offset += fmod(avg_pos[idx] - sign * mech_degrees_in_loop * idx / NUMBER_OF_STEPS_PER_LOOP, mech_degrees_in_loop);
	}
	normalized_offset /= NUMBER_OF_STEPS_PER_LOOP;
	offset_mdeg = normalized_offset - ZERO_POS / ret_val.pole_pairs;
	//ret_val.fixedpt_offset = (int)(LOOP_SIZE * offset_mdeg / E_DEGREE_CHANGE_IN_LOOP + ret_val.sign * 0.5);
	if(offset_mdeg < 0){
		offset_mdeg += mech_degrees_in_loop;
	}
	ret_val.offset = offset_mdeg;
	return(ret_val);
}

