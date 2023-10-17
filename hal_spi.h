/*
 * hal_spi.h
 *
 *  Created on: Jan 31, 2023
 *      Author: Admin
 */

#ifndef HAL_SPI_H_
#define HAL_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "fsl_lpspi_freertos_store.h"
#include "fsl_common.h"

#ifndef PACKED
#define PACKED __attribute((packed))
#endif

typedef enum PACKED
{
  HAL_SPI_SS_NONE = (uint32_t)(1 << 0),
  HAL_SPI_SS_0 = (uint32_t)(1 << 1),
  HAL_SPI_SS_1 = (uint32_t)(1 << 2),
  HAL_SPI_SS_2 = (uint32_t)(1 << 3),
  HAL_SPI_SS_3 = (uint32_t)(1 << 4),
  HAL_SPI_SS_BOTH = (uint32_t)(1 << 5),
}HAL_SPI_SS_t;

typedef enum PACKED
{
  HAL_SPI_ACTIV_LVL_LOW = 0,
  HAL_SPI_ACTIV_LVL_HIGH,
}HAL_SPI_LVL_t;

typedef enum PACKED
{
  HAL_SPI_MODE_0,
  HAL_SPI_MODE_1,
  HAL_SPI_MODE_2,
  HAL_SPI_MODE_3,
}HAL_SPI_MODE_t;

void HAL_SPI_Open(HAL_SPI_ID_t spiId, uint32_t speed, uint8_t txBitLen, HAL_SPI_LVL_t lvl, HAL_SPI_MODE_t mode);
void HAL_SPI_SetQUADMode(HAL_SPI_ID_t spi);
void HAL_SPI_SetNORMMode(HAL_SPI_ID_t spi);
void HAL_SPI_SetOMode(HAL_SPI_ID_t spi);
void HAL_SPI_SetIMode(HAL_SPI_ID_t spi);
void HAL_SPI_DisableIT(HAL_SPI_ID_t spi);
void HAL_SPI_EnableIT(HAL_SPI_ID_t spi);
status_t HAL_SPI_WriteByte(HAL_SPI_ID_t spi, uint8_t data);
uint8_t HAL_SPI_ReadByte(HAL_SPI_ID_t spiId);
void HAL_SPI_TRIG(HAL_SPI_ID_t spi);
bool HAL_SPI_EnableChip(HAL_SPI_ID_t spi, HAL_SPI_SS_t ssId);
bool HAL_SPI_DisableChip(HAL_SPI_ID_t spi, HAL_SPI_SS_t ssId);
int32_t HAL_SPI_ReadWrite_IT(HAL_SPI_ID_t spiId, uint8_t *tx, uint8_t *rx, uint32_t length);
uint32_t HAL_SPI_RegisterCallBack(HAL_SPI_ID_t spiId, HAL_SPI_Cb const *cb, void *const param);


#ifdef __cplusplus
}
#endif

#endif /* HAL_SPI_H_ */
