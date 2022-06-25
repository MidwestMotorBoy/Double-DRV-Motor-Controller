/*
 * DRV8350.c
 *
 *  Created on: Oct 1, 2021
 *      Author: MidwestMotorBoy
 */
#include "drv8350s.h"

void write_to_reg(SPI_HandleTypeDef* spi_handler, uint16_t reg, uint16_t data){
	uint16_t message = (reg&0x000f)<<11;
	message|= data&0x07ff;
	HAL_SPI_Transmit(spi_handler, (uint8_t*) &message, 1, 100);
}

uint16_t read_reg(SPI_HandleTypeDef* spi_handler, uint16_t reg){
	uint16_t reg_val;
	uint16_t message = ((reg&0x000f)<<11)+0x8000;
	HAL_SPI_TransmitReceive(spi_handler, &message, (uint8_t*) &reg_val, 1, 100);
	return(reg_val);
}

uint16_t read_fault_reg(SPI_HandleTypeDef* spi_handler){
	return(read_reg(spi_handler,FAULT_STATUS) & 0xfff);
}

uint16_t read_vgs_reg(SPI_HandleTypeDef* spi_handler){
	return(read_reg(spi_handler,VGS_STATUS) & 0xfff);
}

void set_driver_ctrl_reg(SPI_HandleTypeDef* spi_handler, uint_fast16_t ocp_act, uint_fast16_t dis_gduv, uint_fast16_t dis_gdf, uint_fast16_t otw_rep,	uint_fast16_t pwm_mode,
		uint_fast16_t pwm1_mode, uint_fast16_t pwm1_dir, uint_fast16_t coast, uint_fast16_t brake){
	uint16_t data = (ocp_act<<10) | (dis_gduv<<9) | (otw_rep<<8) | (otw_rep<<7) | (pwm_mode<<5) | (pwm1_mode<<4) |
			(pwm1_dir<<3) | (coast<<2) | (brake<<1);
	write_to_reg(spi_handler, DRIVER_CTRL, data);
}

uint16_t read_driver_ctrl_reg(SPI_HandleTypeDef* spi_handler){
	return(read_reg(spi_handler,DRIVER_CTRL) & 0xfff);
}

void set_hs_driver_reg(SPI_HandleTypeDef* spi_handler, uint_fast16_t lock, uint_fast16_t idrvp_hs, uint_fast16_t idrvn_hs){
	uint16_t data = (lock<<8) | (idrvp_hs<<4) | idrvn_hs;
	write_to_reg(spi_handler, HS_GATE_DRV, data);
}

uint16_t read_hs_driver_reg(SPI_HandleTypeDef* spi_handler){
	return(read_reg(spi_handler,HS_GATE_DRV));
}

void set_ls_driver_ctrl_reg(SPI_HandleTypeDef* spi_handler, uint_fast16_t cbc, uint_fast16_t tdrive, uint_fast16_t idrvp_ls, uint_fast16_t idrvn_ls){
	uint16_t data = (cbc<<10)| (tdrive<<8) | (idrvp_ls<<4) | idrvn_ls;
	write_to_reg(spi_handler, LS_GATE_DRV, data);
}

uint16_t read_ls_driver_reg(SPI_HandleTypeDef* spi_handler){
	return(read_reg(spi_handler,LS_GATE_DRV));
}

void set_ocp_ctrl_reg(SPI_HandleTypeDef* spi_handler, uint_fast16_t retry_time, uint_fast16_t dead_time, uint_fast16_t mode, uint_fast16_t deglitch, uint_fast16_t vds_lvl){
	uint16_t data = (retry_time<<10) | (dead_time<<8) | (mode<<6) | (deglitch<<4) | vds_lvl;
	write_to_reg(spi_handler, OCP_CTRL, data);
}

uint16_t read_ocp_reg(SPI_HandleTypeDef* spi_handler){
	return(read_reg(spi_handler,OCP_CTRL));
}

void read_all_regs(SPI_HandleTypeDef* spi_handler,uint16_t * reg_data){
	reg_data[0] = read_fault_reg(spi_handler);
	reg_data[1] = read_vgs_reg(spi_handler);
	reg_data[2] = read_driver_ctrl_reg(spi_handler);
	reg_data[3] = read_hs_driver_reg(spi_handler);
	reg_data[4] = read_ls_driver_reg(spi_handler);
	reg_data[5] = read_ocp_reg(spi_handler);
}

void clear_flt(SPI_HandleTypeDef* spi_handler){
	uint_fast16_t temp = read_driver_ctrl_reg(spi_handler)|0x01;
	write_to_reg(spi_handler, DRIVER_CTRL, temp);
}
