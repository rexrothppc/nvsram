/**
 * @file nuc972_hal_spi.c
 * @author Oleksandr Kishchuk
 * @brief
 * @version 0.1
 * @date 2022-09-16
 *
 * @copyright Copyright (c) 2022 "Tiras - 12"
 *
 */
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "MIMXRT1024.h"
#include "fsl_lpspi_freertos.h"
#include "fsl_gpio.h"

#include "hal_spi.h"
#include "pin_mux.h"
#include "DebugPrintf.h"

#include "fsl_lpspi_freertos_store.h"

static volatile uint32_t actSPI[HAL_SPI_NUMBER] = { HAL_SPI_SS_NONE, HAL_SPI_SS_NONE, HAL_SPI_SS_NONE, HAL_SPI_SS_NONE, HAL_SPI_SS_NONE, HAL_SPI_SS_NONE, HAL_SPI_SS_NONE };
/*FUNCTIONS DECLARATION*********************************************************/

//#define EXAMPLE_LPSPI_DEALY_COUNT 0xfffffU
//
///* Select USB1 PLL PFD0 (720 MHz) as lpspi clock source */
//#define EXAMPLE_LPSPI_CLOCK_SOURCE_SELECT (1U)
///* Clock divider for master lpspi clock source */
//#define EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER (7U)
#define LPSPI_MASTER_CLK_FREQ 80000000U//(CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER + 1U))

lpspi_rtos_handle_t g_freertos_hspi[HAL_SPI_NUMBER];

void HAL_SPI_Open(HAL_SPI_ID_t spiId, uint32_t speed, uint8_t txBitLen, HAL_SPI_LVL_t lvl, HAL_SPI_MODE_t mode)
  {

    assert(spiId < HAL_SPI_NUMBER);

    LPSPI_MasterGetDefaultConfig(&g_freertos_hspi[spiId].store.mconfig);

    g_freertos_hspi[spiId].store.mconfig.baudRate = 500000U;
    g_freertos_hspi[spiId].store.mconfig.bitsPerFrame = 8U;
    g_freertos_hspi[spiId].store.srcClock_Hz = LPSPI_MASTER_CLK_FREQ;
    g_freertos_hspi[spiId].store.transmitDone = true;
    g_freertos_hspi[spiId].store.mconfig.cpha = 0U;
    g_freertos_hspi[spiId].store.mconfig.cpol = 0U;
    g_freertos_hspi[spiId].store.mconfig.whichPcs = kLPSPI_Pcs0;
    g_freertos_hspi[spiId].store.mconfig.pinCfg = kLPSPI_SdiInSdoOut;

    g_freertos_hspi[spiId].store.mconfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

    switch (spiId)
      {
      case HAL_SPI_ID_1: //W5500
        g_freertos_hspi[spiId].store.mconfig.baudRate = speed;
        g_freertos_hspi[spiId].store.mconfig.bitsPerFrame = 8;
        g_freertos_hspi[spiId].store.mconfig.cpha = 0U;
        g_freertos_hspi[spiId].store.mconfig.cpol = 0U;
        g_freertos_hspi[spiId].store.mconfig.whichPcs = kLPSPI_Pcs0;
        g_freertos_hspi[spiId].store.mconfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

        g_freertos_hspi[spiId].store.mconfig.pcsToSckDelayInNanoSec = 5; //2.5
        g_freertos_hspi[spiId].store.mconfig.lastSckToPcsDelayInNanoSec = 3;
        g_freertos_hspi[spiId].store.mconfig.betweenTransferDelayInNanoSec = 3;

        g_freertos_hspi[spiId].store.bs = LPSPI1;
        g_freertos_hspi[spiId].store.buffer.configFlags = kLPSPI_MasterPcs0;// | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
        break;
      case HAL_SPI_ID_2: //M74HC595RM13TR
        g_freertos_hspi[spiId].store.mconfig.baudRate = speed;
        g_freertos_hspi[spiId].store.mconfig.bitsPerFrame = 8;
        g_freertos_hspi[spiId].store.mconfig.cpha = 0U;
        g_freertos_hspi[spiId].store.mconfig.cpol = 0U;
        g_freertos_hspi[spiId].store.mconfig.whichPcs = kLPSPI_Pcs0;
        g_freertos_hspi[spiId].store.mconfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

        g_freertos_hspi[spiId].store.mconfig.pcsToSckDelayInNanoSec = 2;//1000000000 / masterConfig.baudRate * 2;
        g_freertos_hspi[spiId].store.mconfig.lastSckToPcsDelayInNanoSec = 2;//1000000000 / masterConfig.baudRate * 2;
        g_freertos_hspi[spiId].store.mconfig.betweenTransferDelayInNanoSec = 2;//1000000000 / masterConfig.baudRate * 2;

        g_freertos_hspi[spiId].store.bs = LPSPI2;
        g_freertos_hspi[spiId].store.buffer.configFlags = kLPSPI_MasterPcs0;// | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
        break;
      case HAL_SPI_ID_3: //IS66/67WVS1M8ALL
        g_freertos_hspi[spiId].store.mconfig.baudRate = speed;
        g_freertos_hspi[spiId].store.mconfig.bitsPerFrame = 8;
        g_freertos_hspi[spiId].store.mconfig.cpha = 0U;
        g_freertos_hspi[spiId].store.mconfig.cpol = 0U;
        g_freertos_hspi[spiId].store.mconfig.whichPcs = kLPSPI_Pcs0;
        g_freertos_hspi[spiId].store.mconfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

        g_freertos_hspi[spiId].store.mconfig.pcsToSckDelayInNanoSec = 0; //2.5
        g_freertos_hspi[spiId].store.mconfig.lastSckToPcsDelayInNanoSec = 0;
        g_freertos_hspi[spiId].store.mconfig.betweenTransferDelayInNanoSec = 0;

        g_freertos_hspi[spiId].store.bs = LPSPI3;
        g_freertos_hspi[spiId].store.buffer.configFlags = kLPSPI_MasterPcs0;// | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
        break;
      case HAL_SPI_ID_4: //ST7586
        g_freertos_hspi[spiId].store.mconfig.baudRate = speed;
        g_freertos_hspi[spiId].store.mconfig.bitsPerFrame = 8;
        g_freertos_hspi[spiId].store.mconfig.cpha = 0U;
        g_freertos_hspi[spiId].store.mconfig.cpol = 0U;
        g_freertos_hspi[spiId].store.mconfig.whichPcs = kLPSPI_Pcs0;
        g_freertos_hspi[spiId].store.mconfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

        g_freertos_hspi[spiId].store.mconfig.pcsToSckDelayInNanoSec = 48;//20;
        g_freertos_hspi[spiId].store.mconfig.lastSckToPcsDelayInNanoSec = 48;//20;
        g_freertos_hspi[spiId].store.mconfig.betweenTransferDelayInNanoSec = 48;//20;

        g_freertos_hspi[spiId].store.bs = LPSPI4;
        g_freertos_hspi[spiId].store.buffer.configFlags = kLPSPI_MasterPcs0;// | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
        break;
      case HAL_SPI_NUMBER:
      default:
        return;
      }

    assert(
      kStatus_Success
        == LPSPI_RTOS_Init(&g_freertos_hspi[spiId], g_freertos_hspi[spiId].store.bs, &g_freertos_hspi[spiId].store.mconfig, g_freertos_hspi[spiId].store.srcClock_Hz));

  }

void HAL_SPI_SetQUADMode(HAL_SPI_ID_t spi)
  {
    switch (spi)
      {
      case HAL_SPI_ID_0:
        break;
      case HAL_SPI_ID_1:
      case HAL_SPI_NUMBER:
      default:
        return;
      }
  }

void HAL_SPI_SetNORMMode(HAL_SPI_ID_t spi)
  {
    switch (spi)
      {
      case HAL_SPI_ID_0:
        break;
      case HAL_SPI_ID_1:
      case HAL_SPI_NUMBER:
      default:
        return;
      }
  }

void HAL_SPI_SetOMode(HAL_SPI_ID_t spi)
  {

  }

void HAL_SPI_SetIMode(HAL_SPI_ID_t spi)
  {

  }

void HAL_SPI_EnableIT(HAL_SPI_ID_t spi)
  {
    if (spi == HAL_SPI_ID_3)
      {
        NVIC_SetPriority(LPSPI1_IRQn, 7);
      }
    else if (spi == HAL_SPI_ID_1)
      {
        NVIC_SetPriority(LPSPI2_IRQn, 8);
      }
    else if (spi == HAL_SPI_ID_2)
      {
        NVIC_SetPriority(LPSPI3_IRQn, 9);
      }
    else if (spi == HAL_SPI_ID_4)
      {
        NVIC_SetPriority(LPSPI3_IRQn, 10);
      }
  }

void HAL_SPI_DisableIT(HAL_SPI_ID_t spi)
  {

  }

status_t HAL_SPI_WriteByte(HAL_SPI_ID_t spiId, uint8_t data)
  {
    uint8_t data_rx;
    assert(spiId == HAL_SPI_ID_1 || spiId == HAL_SPI_ID_2 || spiId == HAL_SPI_ID_3 || spiId == HAL_SPI_ID_4);

    g_freertos_hspi[spiId].store.buffer.txData = &data;
    g_freertos_hspi[spiId].store.buffer.rxData = &data_rx;
    g_freertos_hspi[spiId].store.buffer.dataSize = 1;

    switch (spiId)
      {
      case HAL_SPI_ID_1:
      case HAL_SPI_ID_2:
      case HAL_SPI_ID_3:
      case HAL_SPI_ID_4:
        g_freertos_hspi[spiId].store.buffer.configFlags = kLPSPI_MasterPcs0;
        break;
      default:
        g_freertos_hspi[spiId].store.buffer.configFlags = kLPSPI_MasterPcs0/* | kLPSPI_MasterPcsContinuous | kLPSPI_SlaveByteSwap*/;
        break;
      }

    g_freertos_hspi[spiId].store.transmitDone = false;

    while (kStatus_Success != LPSPI_RTOS_Transfer(&g_freertos_hspi[spiId], &g_freertos_hspi[spiId].store.buffer))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    return kStatus_Success;
  }

uint8_t HAL_SPI_ReadByte(HAL_SPI_ID_t spiId)
  {
    uint8_t data = 0x00;
    assert(spiId == HAL_SPI_ID_1 || spiId == HAL_SPI_ID_2 || spiId == HAL_SPI_ID_3 || spiId == HAL_SPI_ID_4);

    g_freertos_hspi[spiId].store.buffer.txData = NULL;
    g_freertos_hspi[spiId].store.buffer.rxData = &data;
    g_freertos_hspi[spiId].store.buffer.dataSize = 1;

    while (kStatus_Success != LPSPI_RTOS_Transfer(&g_freertos_hspi[spiId], &g_freertos_hspi[spiId].store.buffer))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    return data;
  }

void HAL_SPI_TRIG(HAL_SPI_ID_t spi)
  {

  }

bool HAL_SPI_EnableChip(HAL_SPI_ID_t spiIdx, HAL_SPI_SS_t ssId)
  {
    assert(spiIdx == HAL_SPI_ID_1 || spiIdx == HAL_SPI_ID_2 || spiIdx == HAL_SPI_ID_3 || spiIdx == HAL_SPI_ID_4);
    assert(ssId == HAL_SPI_SS_0);

    if ((actSPI[spiIdx] & ssId) == ssId)
      return true;

    assert(actSPI[spiIdx] == HAL_SPI_SS_NONE);

    if (spiIdx == HAL_SPI_ID_3 && ssId == HAL_SPI_SS_0)
      {
        GPIO_PinWrite(IS66WVS1M8BLL104NLI_SPI3_SS0_PORT, IS66WVS1M8BLL104NLI_SPI3_SS0_PIN, 0);
        actSPI[spiIdx] |= ssId;
      }
    else if (spiIdx == HAL_SPI_ID_2 && ssId == HAL_SPI_SS_0)
      {
        GPIO_PinWrite(M74HC595RM13TR_SPI2_SS0_PORT, M74HC595RM13TR_SPI2_SS0_PIN, 0);
        actSPI[spiIdx] |= ssId;
      }
    else if (spiIdx == HAL_SPI_ID_4 && ssId == HAL_SPI_SS_0)
      {
        GPIO_PinWrite(ST7586S_LCD_CS_PORT, ST7586S_LCD_CS_PIN, 0);
        actSPI[spiIdx] |= ssId;
      }
    else if (spiIdx == HAL_SPI_ID_1 && ssId == HAL_SPI_SS_0)
      {
        GPIO_PinWrite(W5500_SPI1_SS0_PORT, W5500_SPI1_SS0_PIN, 0);
        actSPI[spiIdx] |= ssId;
      }
    else
      {
        DebugPrintf("ERROR: spiIdx = %d, ssId = %d, does not feet this is requirements.\n", spiIdx, ssId);
        assert(false);
        return false;
      }

    SDK_DelayAtLeastUs(10, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

    return true;
  }

bool HAL_SPI_DisableChip(HAL_SPI_ID_t spiIdx, HAL_SPI_SS_t ssId)
  {

    assert(spiIdx == HAL_SPI_ID_1 || spiIdx == HAL_SPI_ID_2 || spiIdx == HAL_SPI_ID_3 || spiIdx == HAL_SPI_ID_4);
    assert(ssId == HAL_SPI_SS_0);

    if (spiIdx == HAL_SPI_ID_3 && ssId == HAL_SPI_SS_0)
      {
        GPIO_PinWrite(IS66WVS1M8BLL104NLI_SPI3_SS0_PORT, IS66WVS1M8BLL104NLI_SPI3_SS0_PIN, 1);
        actSPI[spiIdx] &= ~ssId;
      }
    else if (spiIdx == HAL_SPI_ID_2 && ssId == HAL_SPI_SS_0)
      {
        GPIO_PinWrite(M74HC595RM13TR_SPI2_SS0_PORT, M74HC595RM13TR_SPI2_SS0_PIN, 1);
        actSPI[spiIdx] &= ~ssId;
      }
    else if (spiIdx == HAL_SPI_ID_4 && ssId == HAL_SPI_SS_0)
      {
        GPIO_PinWrite(ST7586S_LCD_CS_PORT, ST7586S_LCD_CS_PIN, 1);
        actSPI[spiIdx] &= ~ssId;
      }
    else if (spiIdx == HAL_SPI_ID_1 && ssId == HAL_SPI_SS_0)
      {
        GPIO_PinWrite(W5500_SPI1_SS0_PORT, W5500_SPI1_SS0_PIN, 1);
        actSPI[spiIdx] |= ssId;
      }
    else
      {
        DebugPrintf("ERROR: spiIdx = %d, ssId = %d, does not feet this is requirements.\n", spiIdx, ssId);
        assert(false);
        return false;
      }

    SDK_DelayAtLeastUs(3, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    return true;
  }

status_t HAL_SPI_ReadWrite_IT(HAL_SPI_ID_t spiId, uint8_t *tx, uint8_t *rx, uint32_t length)
  {
    status_t status = kStatus_Fail;

    assert(spiId == HAL_SPI_ID_2 || spiId == HAL_SPI_ID_3 || spiId == HAL_SPI_ID_4);

    if (g_freertos_hspi[spiId].store.transmitDone == false)
      return kStatus_Busy;

    if (spiId < HAL_SPI_NUMBER && (tx != NULL || rx != NULL) && length > 0)
      {

        g_freertos_hspi[spiId].store.buffer.txData = tx;
        g_freertos_hspi[spiId].store.buffer.rxData = rx;
        g_freertos_hspi[spiId].store.buffer.dataSize = length;

        g_freertos_hspi[spiId].store.transmitDone = false;
        return LPSPI_RTOS_Transfer(&g_freertos_hspi[spiId], &(g_freertos_hspi[spiId].store.buffer));
      }

    return status;
  }

uint32_t HAL_SPI_RegisterCallBack(HAL_SPI_ID_t spiId, HAL_SPI_Cb const *cb, void *const param)
  {
    assert(spiId == HAL_SPI_ID_2 || spiId == HAL_SPI_ID_3 || spiId == HAL_SPI_ID_4);

    g_freertos_hspi[spiId].store.SpiCallback = (HAL_SPI_Cb) cb;
    g_freertos_hspi[spiId].store.SpiCBParam = param;

    return 0;
  }

