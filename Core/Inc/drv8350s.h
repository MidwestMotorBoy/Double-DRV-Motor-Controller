#include "stm32h7xx_hal.h"


struct drv8350{
	SPI_HandleTypeDef spi;
	GPIO_TypeDef *GPIOx_en;
	uint16_t en_Pin;
	GPIO_TypeDef *GPIOx_enhb;
	uint16_t en_Pinhb;
	GPIO_TypeDef *GPIOx_fault;
	uint16_t fault_Pin;
	TIM_HandleTypeDef tim;
};
void write_to_reg(uint16_t reg,uint16_t data);
uint16_t read_reg_IT(uint16_t reg);
