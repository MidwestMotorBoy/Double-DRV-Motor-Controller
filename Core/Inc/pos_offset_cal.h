/*
 * pos_offset_cal.h
 *
 *  Created on: May 19, 2022
 *      Author: MidwestMotorBoy
 */
#include "stm32h7xx_hal.h"
#include "MA732.h"

#ifndef INC_POS_OFFSET_CAL_H_
#define INC_POS_OFFSET_CAL_H_

typedef struct{
	MA732_ROT_DIR direction;
	int pole_pairs;
	float offset;
}SENSOR_OFFSET_E;

SENSOR_OFFSET_E get_pos_offset(TIM_TypeDef *timer, SPI_HandleTypeDef *encoder_spi);

#endif /* INC_POS_OFFSET_CAL_H_ */
