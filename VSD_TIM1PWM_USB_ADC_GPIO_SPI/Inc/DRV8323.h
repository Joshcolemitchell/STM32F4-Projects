
/*
 *
 *  Created on: Jun 3, 2018
 *      Author: Joshua
 */

#ifndef DRV8323_H_
#define DRV8323_H_

#include "stdbool.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

/*Configure Registers Values For DRV8323 */
#define DRV8323regDrvCtrl  	(0x1000)
#define DRV8323regGateDrvHS (0x3BF0)
#define DRV8323regGateDrvLS (0x6FF0)
#define DRV8323regOcpCtrl  	(0x1600)
#define DRV8323regCsaCtrl  	(0x6830)
/*Configure Register Addresses For DRV8323 */
#define ADR_FAULT_STAT      (0x00)
#define ADR_VGS_STAT        (0x01)
#define ADR_DRV_CTRL        (0x02)
#define ADR_GATE_DRV_HS     (0x03)
#define ADR_GATE_DRV_LS     (0x04)
#define ADR_OCP_CTRL        (0x05)
#define ADR_CSA_CTRL        (0x06)

uint16_t DRV8323_readSpi(uint8_t regAdr);
void DRV8323_writeSpi(uint8_t regAdr, uint16_t regVal);
char DRV8323_Config(void);


#endif //_DRV8323_H_


