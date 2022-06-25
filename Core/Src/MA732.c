/*
 * MA732.c
 *
 *  Created on: Jun 17, 2022
 *      Author: MidwestMotorBoy
 */
#include "MA732.h"

#define FULL_ROT_16_BIT 65536
#define DEGREES_IN_FULL_ROT 360.0f
#define WRITE_REQUEST_COMMAND 0x8000
#define READ_REQUEST_COMMAND 0x4000
#define REG_ADDRESS_SHIFT 8
#define MSB_MASK 0xff00
#define LSB_MASK 0x00ff
#define ZERO_DEGREES 0.0f
/***********************************************************************************/

void reset_offset_and_dir(SPI_HandleTypeDef *encoder){
	set_offset(encoder, ZERO_DEGREES);
	set_dir(encoder, DIR_CW);
}
/***********************************************************************************/

void set_offset(SPI_HandleTypeDef *encoder, float offset){
	uint16_t zero_offset = (FULL_ROT_16_BIT - ((int32_t)( offset / DEGREES_IN_FULL_ROT * FULL_ROT_16_BIT))) % FULL_ROT_16_BIT;
	write_to_reg_ma732(encoder, ZERO_OFFSET_MSB_REG, (uint8_t) ((MSB_MASK & zero_offset)>>8) );
	HAL_Delay(20);
	write_to_reg_ma732(encoder, ZERO_OFFSET_LSB_REG, (uint8_t) (LSB_MASK & zero_offset));
	HAL_Delay(20);
}
/***********************************************************************************/

float read_offset(SPI_HandleTypeDef *encoder){
	 int16_t zero_offset = read_reg_ma732(encoder, ZERO_OFFSET_LSB_REG);
	 zero_offset |= read_reg_ma732(encoder, ZERO_OFFSET_MSB_REG) << 8;
	 float offset = fmod(((float)(FULL_ROT_16_BIT - zero_offset)) / FULL_ROT_16_BIT * DEGREES_IN_FULL_ROT,360.0f);
	 return(offset);
}
/***********************************************************************************/

void set_dir(SPI_HandleTypeDef *encoder,MA732_ROT_DIR dir){
	write_to_reg_ma732(encoder, ROT_DIR_REG, (uint8_t) dir);
	HAL_Delay(20);
}
/***********************************************************************************/

MA732_ROT_DIR read_rot_dir(SPI_HandleTypeDef *encoder){
	uint8_t rot_dir = read_reg_ma732(encoder, ROT_DIR_REG);
	MA732_ROT_DIR ret_val;
	switch(rot_dir){
	case DIR_CW:
		ret_val = DIR_CW;
		break;
	case DIR_CCW:
		ret_val = DIR_CCW;
		break;
	default:
		ret_val = DIR_ERROR;
		break;
	}
	return(ret_val);
}
/***********************************************************************************/

void set_filter_window(SPI_HandleTypeDef *encoder,MA732_FW_TIM_CONST_E FW){
	//TODO
}
/***********************************************************************************/

void write_to_reg_ma732(SPI_HandleTypeDef *encoder, MA732_REGS_E reg, uint8_t value){
	uint16_t data_packet = WRITE_REQUEST_COMMAND | (reg << REG_ADDRESS_SHIFT) | value;
	uint16_t reg_value;
	HAL_SPI_Transmit(encoder, (uint8_t*) &data_packet, 1, 100);
	HAL_SPI_Receive(encoder, (uint8_t*) &reg_value, 1, 100);

}
/***********************************************************************************/

uint8_t read_reg_ma732(SPI_HandleTypeDef *encoder, MA732_REGS_E reg){
	uint16_t data_packet = READ_REQUEST_COMMAND | (reg << REG_ADDRESS_SHIFT);
	uint16_t reg_value;
	HAL_SPI_Transmit(encoder, (uint8_t*) &data_packet, 1, 100);
	HAL_SPI_Receive(encoder, (uint8_t*) &reg_value, 1, 100);
	return((uint8_t) (reg_value>>8));
}
