/*
 * pos_offset_cal.h
 *
 *  Created on: May 19, 2022
 *      Author: LoganRosenmayer
 */
#include "stm32h7xx_hal.h"

#ifndef INC_POS_OFFSET_CAL_H_
#define INC_POS_OFFSET_CAL_H_

typedef struct{
	int sign;
	int pole_pairs;
	int fixedpt_offset;
}SENSOR_OFFSET_E;

SENSOR_OFFSET_E get_pos_offset(TIM_TypeDef *timer, SPI_HandleTypeDef *encoder_spi);

#endif /* INC_POS_OFFSET_CAL_H_ */
