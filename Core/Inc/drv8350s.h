/*
 * DRV8350.h
 *
 *  Created on: Oct 1, 2021
 *      Author: MidwestMotorBoy
 */
#include "stm32h7xx_hal.h"

enum regs{FAULT_STATUS, VGS_STATUS, DRIVER_CTRL, HS_GATE_DRV, LS_GATE_DRV, OCP_CTRL};
// DRIVER CTRL REG ENUMS
enum OCP_ACT{SHUTDOWN_BRIDGE, SHUTDOWN_ALL_BRIDGES};
enum DIS_GDUV{UVLO_EN, UVLO_DIS};
enum DIS_GDF{GATE_DRV_FAULT_EN, GATE_DRV_FAULT_DIS};
enum OTW_REP{OTW_REPORT_EN, OTW_REPORT_DIS};
enum PWM_MODE{PWM_6X, PWM_3X, PWM_1X, PWM_IND};
enum PWM1_COM{SYNC_RECT, ASYNC_RECT};
enum PWM1_DIR{CW, CCW};
enum COAST{COAST_DIS, COAST_EN};
enum BRAKE{NO_TPS, TPS};

//GATE DRIVER HS/LS REGS ENUMS
enum LOCK{LOCK_REGS=0x6,UNLOCK_REGS=0x3};
enum CBC{RETRY_ONLY,RETRY_OR_PWM};
enum TDRIVE{PK_CUR_500NS, PK_CUR_1000NS, PK_CUR_2000NS, PK_CUR_4000NS};
enum I_DRIVEP{IDRIVEP_50MA, IDRIVEP_100MA=0x2, IDRIVEP_150MA, IDRIVEP_300MA, IDRIVEP_350MA, IDRIVEP_400MA, IDRIVEP_450MA, IDRIVEP_550MA,
	IDRIVEP_600MA, IDRIVEP_650MA, IDRIVEP_700MA, IDRIVEP_850MA, IDRIVEP_900MA, IDRIVEP_950MA, IDRIVEP_1000MA};
enum I_DRIVEN{IDRIVEN_100MA, IDRIVEN_200MA=0x2, IDRIVEN_300MA, IDRIVEN_600MA, IDRIVEN_700MA, IDRIVEN_800MA, IDRIVEN_900MA, IDRIVEN_1100MA,
	IDRIVEN_1200MA, IDRIVEN_1300MA, IDRIVEN_1400MA, IDRIVEN_1700MA, IDRIVEN_1800MA, IDRIVEN_1900MA, IDRIVEN_2000MA};

//OPC CTRL REG ENUMS
enum tretry{RETRY_8MS, RETRY_50US};
enum deadtime{DEAD_TIME_50NS, DEAD_TIME_100NS, DEAD_TIME_200NS, DEAD_TIME_400NS};
enum mode{OC_LATCHED, OC_AUTO_RTY, OC_RPT_NO_FLT, OC_NO_RPT_NO_FLT};
enum OCP_DEG{OCP_DEG_1US, OCP_DEG_2US, OCP_DEG_4US, OCP_DEG_8US};
enum VDS_LVL{VDS_LVL_0_06V, VDS_LVL_0_07V, VDS_LVL_0_08V, VDS_LVL_0_09V, VDS_LVL_0_1V, VDS_LVL_0_2V, VDS_LVL_0_3V, VDS_LVL_0_4V,
			 VDS_LVL_0_5V, VDS_LVL_0_6V, VDS_LVL_0_7V, VDS_LVL_0_8V, VDS_LVL_0_9V, VDS_LVL_1_0V, VDS_LVL_1_5V, VDS_LVL_2_0V};


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

void write_to_reg(SPI_HandleTypeDef* spi_handler, uint16_t reg, uint16_t data);
uint16_t read_reg(SPI_HandleTypeDef* spi_handler, uint16_t reg);
uint16_t read_fault_reg(SPI_HandleTypeDef* spi_handler);
uint16_t read_vgs_reg(SPI_HandleTypeDef* spi_handler);
void set_driver_ctrl_reg(SPI_HandleTypeDef* spi_handler, uint_fast16_t ocp_act, uint_fast16_t dis_gduv, uint_fast16_t dis_gdf, uint_fast16_t otw_rep,	uint_fast16_t pwm_mode,
		uint_fast16_t pwm1_mode, uint_fast16_t pwm1_dir, uint_fast16_t coast, uint_fast16_t brake);
uint16_t read_driver_ctrl_reg(SPI_HandleTypeDef* spi_handler);
void set_hs_driver_reg(SPI_HandleTypeDef* spi_handler, uint_fast16_t lock, uint_fast16_t idrvp_hs, uint_fast16_t idrvn_hs);
uint16_t read_hs_driver_reg(SPI_HandleTypeDef* spi_handler);
void set_ls_driver_ctrl_reg(SPI_HandleTypeDef* spi_handler, uint_fast16_t cbc, uint_fast16_t tdrive, uint_fast16_t idrvp_ls, uint_fast16_t idrvn_ls);
uint16_t read_ls_driver_reg(SPI_HandleTypeDef* spi_handler);
void set_ocp_ctrl_reg(SPI_HandleTypeDef* spi_handler, uint_fast16_t retry_time, uint_fast16_t dead_time, uint_fast16_t mode, uint_fast16_t deglitch, uint_fast16_t vds_lvl);
uint16_t read_ocp_reg(SPI_HandleTypeDef* spi_handler);
void clear_flt(SPI_HandleTypeDef* spi_handler);
