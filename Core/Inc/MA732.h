/*
 * MA732.h
 *
 *  Created on: Jun 17, 2022
 *      Author: MidwestMotorBoy
 */

#ifndef INC_MA732_H_
#define INC_MA732_H_

#include "stm32h7xx_hal.h"

typedef enum{
	ZERO_OFFSET_LSB_REG,
	ZERO_OFFSET_MSB_REG,
	BIAS_CURRENT_REG,
	TRIM_DIR_EN_REG,
	PULSES_IDX_SET_REG,
	MAG_THRESH_HI_LOW_REG,
	ROT_DIR_REG = 0x9,
	FILTER_WINDOW_REG = 0xE,
	ABZ_HYSTERESIS_REG = 0x10,
	MAG_ACT_HI_LOW_REG = 0x1B
}MA732_REGS_E;

typedef enum{
DIR_CW = 0x00,
DIR_CCW = 0x80,
DIR_ERROR = 0xff
}MA732_ROT_DIR;

typedef enum{
time_const_64us = 		51,
time_const_128us = 		68,
time_const_256us = 		85,
time_const_512us = 		102,
time_const_1024us = 	119,
time_const_2048us = 	136,
time_const_4096us = 	153,
time_const_8192us = 	170,
time_const_16384us =	187
}MA732_FW_TIM_CONST_E;


void reset_offset_and_dir(SPI_HandleTypeDef *encoder);
void set_offset(SPI_HandleTypeDef *encoder, float offset);
void set_dir(SPI_HandleTypeDef *encoder, MA732_ROT_DIR dir);
void set_filter_window(SPI_HandleTypeDef *encoder,MA732_FW_TIM_CONST_E FW);
void write_to_reg_ma732(SPI_HandleTypeDef *encoder, MA732_REGS_E reg, uint8_t value);
uint8_t read_reg_ma732(SPI_HandleTypeDef *encoder, MA732_REGS_E reg);
float read_offset(SPI_HandleTypeDef *encoder);
MA732_ROT_DIR read_rot_dir(SPI_HandleTypeDef *encoder);

#endif /* INC_MA732_H_ */
