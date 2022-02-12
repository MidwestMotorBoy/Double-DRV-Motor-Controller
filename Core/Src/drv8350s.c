#include "drv8350s.h"

void write_to_reg(uint16_t reg,uint16_t data){
	uint16_t message = (reg&0x000f)<<11;
	message|= data&0x07ff;
}
uint16_t read_reg_IT(uint16_t reg){
	uint16_t data=0;
	uint16_t message = (reg&0x000f)<<11+0x8000;
	message|= data&0x07ff;
//	HAL_SPI_TransmitReceive(&hspi2, &message, &data, 1, 100);
}
