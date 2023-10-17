#ifndef FREERTOS_SPI_STORE_H_
#define FREERTOS_SPI_STORE_H_

#include "fsl_lpspi.h"

typedef void (* HAL_SPI_Cb) (  /*!< Callback function pointer */
    void      *pCBParam,         /*!< Client supplied callback param */
    uint32_t   Event,            /*!< Event ID specific to the Driver/Service */
    void      *pArg);            /*!< Pointer to the event specific argument */
//@formatter:off
typedef enum
{
  HAL_SPI_ID_0 = 0,
  HAL_SPI_ID_1 = 1,
  HAL_SPI_ID_2 = 2,
  HAL_SPI_ID_3 = 3,
  HAL_SPI_ID_4 = 4,
  HAL_SPI_ID_5 = 5,
  HAL_SPI_ID_6 = 6,
  HAL_SPI_NUMBER = 7
} HAL_SPI_ID_t;
//@formatter:on

typedef struct _fsl_lpspi_freertos_store_t{
  volatile bool transmitDone;
  lpspi_transfer_t buffer;
  LPSPI_Type *bs;
  lpspi_master_config_t mconfig;
  uint32_t srcClock_Hz;
  HAL_SPI_Cb SpiCallback;
  void *SpiCBParam;
}fsl_lpspi_freertos_store_t;





#if defined(__cplusplus)
extern "C" {
#endif




#if defined(__cplusplus)
}
#endif

#endif /* FREERTOS_SPI_STORAGE_H_ */
