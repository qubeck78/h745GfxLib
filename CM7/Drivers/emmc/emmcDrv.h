/*
 * emmcDrv.h
 *
 *  Created on: Jan 3, 2025
 *      Author: qubec
 */

#ifndef EMMC_EMMCDRV_H_
#define EMMC_EMMCDRV_H_

#include "stm32h7xx_hal.h"

#define MMC_TRANSFER_OK          0U
#define MMC_TRANSFER_BUSY        1U

#define MMC_WRITE_TIMEOUT        10000U

#define MMC_READ_TIMEOUT         10000U

#define MMC_DEFAULT_BLOCK_SIZE   512

uint32_t emmcDrvInit( MMC_HandleTypeDef *phmmc );




#endif /* EMMC_EMMCDRV_H_ */
