/*
*
 *  Created on: Jun 3, 2018
 *      Author: Joshua
 */
// **************************************************************************
#include "DRV8323.h"
#include "main.h"
/*Configure Registers For DRV8323*/

#define DRV8323regDrvCtrl  	0x100//default 0x100
#define DRV8323regGateDrvHS 0x3bf//default 0x3BF
#define DRV8323regGateDrvLS 0x6FF//default 0x6ff
#define DRV8323regOcpCtrl  	0x15A//default 0x159
#define DRV8323regCsaCtrl  	0x163//ovc disabled //default 0x163 //683

extern SPI_HandleTypeDef hspi1;

uint16_t DRV8323_readSpi(uint8_t regAdr)
{

	uint16_t controlword = 0x8000 | (regAdr & 0x7) << 11; //MSbit =1 for read, address is 3 bits (MSbit is always 0), data is 11 bits
	uint16_t recbuff = 0x0;

	HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT, SPIx_NSS_PIN, DISABLE);
	//HAL_Delay(0.5);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)(&controlword), (uint8_t*)(&recbuff), 1, 100);
	//HAL_Delay(0.5);
	HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT, SPIx_NSS_PIN, ENABLE);
	return (0x7FF&recbuff);
}

void DRV8323_writeSpi(uint8_t regAdr, uint16_t regVal)
{
	uint16_t controlword = (regAdr & 0x7) << 11 | (regVal & 0x7ff); //MSbit =0 for write, address is 3 bits (MSbit is always 0), data is 11 bits

	HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT, SPIx_NSS_PIN, DISABLE);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)(&controlword), 1, 1000);
	HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT, SPIx_NSS_PIN, ENABLE);
	return;
}

char DRV8323_Config(void)
{
    volatile uint16_t temp;
    char status = 1;
    //turn on DRV8323//
    HAL_GPIO_WritePin(DRV8323_ENABLE_PIN_PORT, DRV8323_ENABLE_PIN,ENABLE);// Disable the DRV83238
	//In TI sample firmware http://www.ti.com/tool/tida-00774, DRV8323regGateDrvHS is written first, and it is written twice
	//In http://www.ti.com/tool/boostxl-drv8323rs, the 5 control register are just written once in order.
	DRV8323_writeSpi(0x02, DRV8323regDrvCtrl);
	HAL_Delay(0.5);
	DRV8323_writeSpi(ADR_GATE_DRV_HS, DRV8323regGateDrvHS);//0x3BF
	HAL_Delay(0.5);
	DRV8323_writeSpi(ADR_GATE_DRV_LS,DRV8323regGateDrvLS);//0x6FF
	HAL_Delay(0.5);
	DRV8323_writeSpi(ADR_OCP_CTRL, DRV8323regOcpCtrl);//0x1E0
	HAL_Delay(0.5);
	DRV8323_writeSpi(ADR_CSA_CTRL, DRV8323regCsaCtrl);//0x003
	HAL_Delay(0.5);

    temp = DRV8323_readSpi(ADR_FAULT_STAT);

    temp = DRV8323_readSpi(ADR_VGS_STAT);


    temp = DRV8323_readSpi(ADR_DRV_CTRL);
    if(temp!=DRV8323regDrvCtrl)status&=0;

    temp = DRV8323_readSpi(ADR_GATE_DRV_HS);
    if(temp!=DRV8323regGateDrvHS)status&=0;

    temp = DRV8323_readSpi(ADR_GATE_DRV_LS);
    if(temp!=ADR_GATE_DRV_LS)status&=0;

    temp = DRV8323_readSpi(ADR_OCP_CTRL);
    if(temp!=ADR_OCP_CTRL)status&=0;

    temp = DRV8323_readSpi(ADR_CSA_CTRL);
    if(temp!=ADR_CSA_CTRL)status&=0;

	return status;
}

