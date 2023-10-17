//https://github.com/gpb01/NVSRAM
//https://ww1.microchip.com/downloads/en/DeviceDoc/25156A.pdf
/*
 NVSRAM.h - Library for Microchip 23LCV512 (64 KB) and 23LCV1024 (128 KB)
 battery-backed SPI RAM memories.

 Copyright (c) 2020 Guglielmo Braguglia.  All right reserved.

 ---------------------------------------------------------------------

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"

#include "hal_spi.h"
#include "fsl_lpspi_freertos_store.h"
#include "fsl_lpspi_freertos.h"
#include "nvsram_is66wvs1m8bll104.h"
#include "DebugPrintf.h"

#ifndef min
#define min(x, y)             (((x) > 0) && ((y) > 0) ? ((size_t)(x) < (size_t)(y) ? (size_t)(x) : (size_t)(y)) : 0)
#endif

#define IS66WVS1MJ8B_MANUFID 0x9D
#define IS66WVS1MJ8B_KGD_PASS 0x5D

#define IS66WVS1MJ8B_WRITE  0x02
#define IS66WVS1MJ8B_FREAD  0x0B
#define IS66WVS1MJ8B_ID     0x9F
#define IS66WVS1MJ8B_BURST  0xC0
#define IS66WVS1MJ8B_RESET  0x99

#define IS66WVS1MJ8B_USE_PAGE32

#ifdef IS66WVS1MJ8B_USE_PAGE32
#define IS66WVS1MJ8B_PAGE_SIZE   32 // 32
#else
#define IS66WVS1MJ8B_PAGE_SIZE   1024 // 32
#endif

#define IS66WVS1MJ8B_MAX  (1024U * 1024)

#ifndef PACKED
#define PACKED __attribute__ ((__packed__))
#endif

enum
  PACKED
    {
      KGD_FAIL = 0x55,
  KGD_PASS = 0x5D
} ENUM_KGD;

typedef union PACKED _ChipId_t
{
  uint8_t id_byte[8];
  uint64_t id_64 PACKED;
  struct PACKED
  {
    uint64_t MANUFACTURE :8 PACKED;
    uint64_t KGD :8 PACKED;
    uint64_t DEV_DENSITY :3 PACKED;
    uint64_t RESERVED :45 PACKED;

  };
} ChipId_t;

static union PACKED u_32_bit
{
  uint8_t addr_byte[4];
  uint32_t addr_32 PACKED;
} mem_addr;

typedef union PACKED
{
  uint8_t BYTE;
  struct PACKED
  {
    unsigned RESERV :6 PACKED;
    unsigned MODE :2 PACKED;
  };
} status_reg_t;

static uint8_t IS66WVS1MJ8B_fast_read_byte(uint32_t addr);
static void IS66WVS1MJ8B_set_burst_length_to_32bytes(void);
static uint16_t IS66WVS1MJ8B_calc_crc16(uint32_t startAddr, uint32_t len);

//============================================================
//Port procedures
SemaphoreHandle_t gChip_mtx = NULL;

static HAL_SPI_ID_t gSpiHndl = HAL_SPI_ID_3;
static HAL_SPI_SS_t gSpiSs = HAL_SPI_SS_0;

static inline bool EnableChip(void)
  {
    bool res = HAL_SPI_EnableChip(gSpiHndl, gSpiSs);
    return res;
  }

static inline bool DisableChip(void)
  {
    return HAL_SPI_DisableChip(gSpiHndl, gSpiSs);
  }

static inline void SysncTake(void)
  {
    xSemaphoreTake(gChip_mtx, portMAX_DELAY);
  }

static inline void SyncGive(void)
  {
    xSemaphoreGive(gChip_mtx);
  }
//============================================================
void IS66WVS1MJ8B_GetId(ChipId_t *pID)
  {
    uint8_t txBuff[3];
    uint32_t address = 0;

    assert(pID != NULL);

    SysncTake();
    memset(pID, 0, sizeof(ChipId_t));
    EnableChip();

    HAL_SPI_WriteByte(gSpiHndl, IS66WVS1MJ8B_ID);

    txBuff[0] = (address >> 16) & 0xFF;
    txBuff[1] = (address >> 8) & 0xFF;
    txBuff[2] = address & 0xFF;

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(gSpiHndl, txBuff, NULL, 3))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(gSpiHndl, NULL, (uint8_t*) pID, sizeof(ChipId_t)))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    DisableChip();
    SyncGive();
  }

bool IS66WVS1MJ8B_init(void)
  {
    ChipId_t chipId;
    uint32_t safetyCounter = 0;
    int32_t successCounter = 0;

    assert(gChip_mtx == NULL);
    gChip_mtx = xSemaphoreCreateMutex();
    assert(gChip_mtx != NULL);

    safetyCounter = 0;
    while (safetyCounter++ < 1000)
      {
        IS66WVS1MJ8B_GetId(&chipId); //page 25

        if (chipId.MANUFACTURE == IS66WVS1MJ8B_MANUFID && chipId.KGD == IS66WVS1MJ8B_KGD_PASS) //(9Dh) = ISSI
          {
            if (successCounter++ > 100)
              {
#ifdef IS66WVS1MJ8B_USE_PAGE32
                IS66WVS1MJ8B_set_burst_length_to_32bytes();
#endif
                DebugPrintfln("INFO: IS66WVS1MJ8B (page %d ), manufacture = %d, kgd = %d RAM_OK!", IS66WVS1MJ8B_PAGE_SIZE, chipId.MANUFACTURE, chipId.KGD);
                return true;
              }
          }
        else
          {
            DebugPrintfln("ERROR: IS66WVS1MJ8B test failure, success counter  %d!", successCounter);
            return false;
          }
      };

    return false;
  }

static void IS66WVS1MJ8B_set_burst_length_to_32bytes(void) //default 1024, now 32
  {
    EnableChip();
    HAL_SPI_WriteByte(gSpiHndl, IS66WVS1MJ8B_BURST);
    DisableChip();
  }

uint32_t IS66WVS1MJ8B_size(void)
  {
    return IS66WVS1MJ8B_MAX;
  }

void IS66WVS1MJ8B_erase(void)
  {
    uint32_t addr = 0;
    int cnt = 0;
    uint8_t *buf = pvPortMalloc(128);
    assert(buf != NULL);
    memset(buf, 0, sizeof buf);
    int len = IS66WVS1MJ8B_MAX;
    while (len > 0)
      {
        cnt = min(128, len);
        IS66WVS1MJ8B_Write(addr, buf, cnt);
        addr += cnt;
        len -= cnt;
      }
    vPortFree(buf);
  }

static uint8_t IS66WVS1MJ8B_fast_read_byte(uint32_t addr)
  {
    uint8_t value;
    mem_addr.addr_32 = (addr & IS66WVS1MJ8B_MAX);
    EnableChip();
    HAL_SPI_WriteByte(gSpiHndl, IS66WVS1MJ8B_FREAD);
    HAL_SPI_WriteByte(gSpiHndl, mem_addr.addr_byte[2]);
    HAL_SPI_WriteByte(gSpiHndl, mem_addr.addr_byte[1]);
    HAL_SPI_WriteByte(gSpiHndl, mem_addr.addr_byte[0]);
    HAL_SPI_WriteByte(gSpiHndl, 0); //dummy
    value = HAL_SPI_ReadByte(gSpiHndl);
    DisableChip();
    return value;
  }

static int IS66WVS1MJ8B_buf_read(uint32_t address, uint8_t *buf, int len)
  {
    uint8_t txBuff[4];

    if (len == 0)
      return 0;

    EnableChip();

    HAL_SPI_WriteByte(gSpiHndl, IS66WVS1MJ8B_FREAD);

    txBuff[0] = (address >> 16) & 0xFF;
    txBuff[1] = (address >> 8) & 0xFF;
    txBuff[2] = address & 0xFF;
    txBuff[3] = 0;

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(gSpiHndl, txBuff, NULL, 4))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(gSpiHndl, NULL, buf, len))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    DisableChip();

    return len;
  }

static int IS66WVS1MJ8B_buf_write(uint32_t address, const uint8_t *buf, int len)
  {
    uint8_t txBuff[3];

    EnableChip();
    HAL_SPI_WriteByte(gSpiHndl, IS66WVS1MJ8B_WRITE);

    txBuff[0] = (address >> 16) & 0xFF;
    txBuff[1] = (address >> 8) & 0xFF;
    txBuff[2] = address & 0xFF;

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(gSpiHndl, txBuff, NULL, 3))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(gSpiHndl, (uint8_t*) buf, NULL, len))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    DisableChip();

    return len;
  }

int IS66WVS1MJ8B_Write(uint32_t address, const uint8_t *data, int len)
  {
    uint8_t page[IS66WVS1MJ8B_PAGE_SIZE] = { 0 };
    uint32_t block_address = 0;
    uint32_t in_block_address = 0;
    uint32_t temporaryLength = 0;
    const uint8_t *src_ptr = NULL;

    assert(data != NULL);
    assert(len > 0);
    assert(address + len < IS66WVS1MJ8B_MAX);
    assert(gChip_mtx != NULL);

    SysncTake();
    while (len)
      {

        block_address = address & ~(IS66WVS1MJ8B_PAGE_SIZE - 1);
        in_block_address = address & (IS66WVS1MJ8B_PAGE_SIZE - 1);
        src_ptr = (const uint8_t*) data;

        if (in_block_address == 0 && len >= IS66WVS1MJ8B_PAGE_SIZE)
          {
            len -= IS66WVS1MJ8B_PAGE_SIZE;
            address += IS66WVS1MJ8B_PAGE_SIZE;
            data += IS66WVS1MJ8B_PAGE_SIZE;
          }
        else
          {
            temporaryLength = (sizeof(page) - in_block_address) < len ? (sizeof(page) - in_block_address) : len;

            IS66WVS1MJ8B_buf_read(block_address, (uint8_t*) page, in_block_address);
            memcpy(page + in_block_address, data, temporaryLength);
            IS66WVS1MJ8B_buf_read(block_address + in_block_address + temporaryLength, (uint8_t*) (page + in_block_address + temporaryLength),
              sizeof(page) - in_block_address - temporaryLength);
            len -= temporaryLength;
            address += temporaryLength;
            data += temporaryLength;
            src_ptr = (const uint8_t*) page;
          }

        IS66WVS1MJ8B_buf_write(block_address, src_ptr, IS66WVS1MJ8B_PAGE_SIZE);
        src_ptr += IS66WVS1MJ8B_PAGE_SIZE;
      }

    SyncGive();
    return len;
  }

int IS66WVS1MJ8B_Read(uint32_t address, uint8_t *data, int len)
  {
    uint8_t page[IS66WVS1MJ8B_PAGE_SIZE] = { 0 };
    uint32_t block_address = 0;
    uint32_t in_block_address = 0;
    uint32_t temporaryLength = 0;
    int pos = 0;

    assert(data != NULL);
    assert(len > 0);
    assert(address + len < IS66WVS1MJ8B_MAX);
    assert(gChip_mtx != NULL);

    SysncTake();
    pos = 0;
    while (len)
      {
        block_address = address & ~(IS66WVS1MJ8B_PAGE_SIZE - 1);
        in_block_address = address & (IS66WVS1MJ8B_PAGE_SIZE - 1);

        temporaryLength = (sizeof(page) - in_block_address) < len ? (sizeof(page) - in_block_address) : len;

        IS66WVS1MJ8B_buf_read(block_address, (uint8_t*) &page, IS66WVS1MJ8B_PAGE_SIZE);

        memcpy(&data[pos], &page[in_block_address], temporaryLength);
        pos += temporaryLength;
        len -= temporaryLength;
        address += temporaryLength;
      }
    SyncGive();
    return len;
  }

//===========================================================================================
//just for test
uint16_t IS66WVS1MJ8B_crc(uint32_t addr, uint32_t lng)
  {
    return IS66WVS1MJ8B_calc_crc16((addr & IS66WVS1MJ8B_MAX), (lng & IS66WVS1MJ8B_MAX));
  }

static uint16_t IS66WVS1MJ8B_crc16_update(uint16_t crc, uint8_t a)
  {
    int i;
    crc ^= a;
    for (i = 0; i < 8; ++i)
      {
        if (crc & 1)
          crc = (crc >> 1) ^ 0xA001;
        else
          crc = (crc >> 1);
      }
    return crc;
  }

static uint16_t IS66WVS1MJ8B_calc_crc16(uint32_t startAddr, uint32_t len)
  {
    uint8_t ch;
    uint16_t crc;
    uint32_t i;

    SysncTake();
    crc = 0;
    for (i = 0; i < len; i++)
      {
        ch = IS66WVS1MJ8B_fast_read_byte(startAddr + i);
        crc = IS66WVS1MJ8B_crc16_update(crc, ch);
      }
    SyncGive();
    return (crc);
  }

