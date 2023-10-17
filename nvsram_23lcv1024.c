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
#include "nvsram_23lcv1024.h"

extern lpspi_rtos_handle_t g_freertos_hspi[HAL_SPI_NUMBER];
extern SemaphoreHandle_t gSpi2_mtx;
//#define MC23LCV512
#define MC23LCV1024

/*
 =====================================================================
 */

#if defined( MC23LCV512 ) && defined( MC23LCV1024 )
#error "You can't define both NVSRAM_512 and NVSRAM_1024"
#endif
#if !defined( MC23LCV512 ) && !defined( MC23LCV1024 )
#error "You must define NVSRAM_512 or NVSRAM_1024"
#endif
#if defined( MC23LCV512 )
#define NVSRAM_MAX   0xFFFF
#pragma message "INFO   : You are compiling for 23LCV512 memory"
#pragma message "WARNING: source code not converted and tested"
#elif defined( MC23LCV1024 )
#define MC23LCVxxx_MAX   0x0001FFFF
#pragma message "INFO: You are compiling for 23LCV1024 memory"
#endif

static uint16_t MC23LCVxxx_calc_crc16(uint32_t startAddr, uint32_t len);
static bool MC23LCVxxx_check_status(void);
static void MC23LCVxxx_set_seq_mode(void);

#define MC23LCVxxx_READ  0x03
#define MC23LCVxxx_WRITE 0x02
#define MC23LCVxxx_RDMR  0x05
#define MC23LCVxxx_WRMR  0x01

#define HIGH 1
#define LOW 0

#ifndef PACKED
#define PACKED __attribute__ ((__packed__))
#endif

static union u_32_bit
{
  uint8_t addr_byte[4];
  uint32_t addr_32;
} mem_addr;

typedef union PACKED
{
  uint8_t BYTE;
  struct PACKED
  {
    unsigned RESERV :6;
    unsigned MODE :2;
  };
} status_reg_t;

bool MC23LCVxxx_init(void)
  {
    bool res = false;
    xSemaphoreTake(gSpi2_mtx, portMAX_DELAY);
    HAL_SPI_EnableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    MC23LCVxxx_set_seq_mode();
    res = MC23LCVxxx_check_status();
    HAL_SPI_DisableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    xSemaphoreGive(gSpi2_mtx);
    return res;
  }

static uint8_t MC23LCVxxx_read_status_reg(void)
  {
    uint8_t result;
    HAL_SPI_EnableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, MC23LCVxxx_RDMR);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, 0x00);
    result = HAL_SPI_ReadByte(HAL_SPI_ID_2);
    HAL_SPI_DisableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    return result;
  }

static void MC23LCVxxx_WriteStatusReg(uint8_t u8StatValue)
  {
    HAL_SPI_EnableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, MC23LCVxxx_WRMR);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, u8StatValue);
    HAL_SPI_DisableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
  }

static bool MC23LCVxxx_check_status(void)
  {
    bool result = false;
    status_reg_t ReturnValue;
    uint32_t i = 0;
    do
      {
        ReturnValue.BYTE = MC23LCVxxx_read_status_reg();
        vTaskDelay(pdMS_TO_TICKS(1));
      } while ((ReturnValue.BYTE == 0xFF) && (i++ < 0xffffffff)); // check the BUSY bit

    if (i < 0xffffffff)
      {
        if (ReturnValue.MODE != 1)
          {
            MC23LCVxxx_WriteStatusReg(0x40);
          }
        result = true;
      }
    else
      {
        result = false;
      }
    return result;
  }

static void MC23LCVxxx_set_seq_mode(void)
  {
    HAL_SPI_EnableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, MC23LCVxxx_WRMR);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, 0x40);
    HAL_SPI_DisableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
  }

uint32_t MC23LCVxxx_size(void)
  {
    return MC23LCVxxx_MAX;
  }

#if defined( MC23LCVxxx_512 )

      // *** For 23LCV512 64K x 8bit memory ***

      // erase function

      void MC23LCVxxx_erase ( void ) {
         uint16_t cnt;
         //
         MC23LCVxxx_cs ( ssPin, LOW );
         spi_wr_byte ( MC23LCVxxx_WRITE );
         spi_transfer16 ( 0x0000 );
         for ( cnt = 0; cnt < MC23LCVxxx_MAX; cnt++ )
            spi_wr_byte ( 0x00 );
         MC23LCVxxx_cs ( ssPin, HIGH );
      }

      // read/write/update functions

      uint8_t MC23LCVxxx_read ( uint16_t addr ) {
         uint8_t value;
         //
         MC23LCVxxx_cs ( ssPin, LOW );
         spi_wr_byte ( MC23LCVxxx_READ );
         spi_transfer16 ( addr );
         value = spi_wr_byte ( 0x00 );
         MC23LCVxxx_cs ( ssPin, HIGH );
         return value;
      }

      void MC23LCVxxx_write ( uint16_t addr, uint8_t value ) {
         MC23LCVxxx_cs ( ssPin, LOW );
         spi_wr_byte ( MC23LCVxxx_WRITE );
         spi_transfer16 ( addr );
         spi_wr_byte ( value );
         MC23LCVxxx_cs ( ssPin, HIGH );
      }

      void MC23LCVxxx_update ( uint16_t addr, uint8_t value ) {
        MC23LCVxxx_write ( addr, value );
      }

      // get/put functions

      int MC23LCVxxx_get(uint32_t addr, uint8_t *buf, int len) {
         MC23LCVxxx_cs ( ssPin, LOW );
         spi_wr_byte ( MC23LCVxxx_READ );
         spi_transfer16 ( addr );
         spi_wr_buf((void*) buf, len);
         MC23LCVxxx_cs ( ssPin, HIGH );
         return t;
      }

      int MC23LCVxxx_put(uint32_t addr, uint8_t *buf, int len) {
         MC23LCVxxx_cs ( ssPin, LOW );
         spi_wr_byte ( MC23LCVxxx_WRITE );
         spi_transfer16 ( addr );
         spi_wr_buf((void*) buf, len);
         MC23LCVxxx_cs ( ssPin, HIGH );
         return t;
      }

      // crc function

      uint16_t MC23LCVxxx_crc ( uint16_t addr, uint16_t lng ) {
         return calc_crc16 ( addr, lng );
      }

#elif defined( MC23LCV1024 )

// *** For 23LCV1024 128K x 8bit memory ***
#ifndef min
#define min(x, y)             (((x) > 0) && ((y) > 0) ? ((size_t)(x) < (size_t)(y) ? (size_t)(x) : (size_t)(y)) : 0)
#endif

void MC23LCVxxx_erase(void)
  {
    uint32_t addr = 0;
    int cnt = 0;
    uint8_t *buf = pvPortMalloc(128);
    assert(buf != NULL);
    memset(buf, 0, sizeof buf);
    int len = MC23LCVxxx_MAX;
    while (len > 0)
      {
        cnt = min(128, len);
       MC23LCVxxx_write(addr, buf, cnt);
        addr += cnt;
        len -= cnt;
      }
    vPortFree(buf);
  }

// read/write/update functions

static uint8_t MC23LCVxxx_read_byte(uint32_t addr)
  {
    uint8_t value;
    mem_addr.addr_32 = (addr & MC23LCVxxx_MAX);
    HAL_SPI_EnableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, MC23LCVxxx_READ);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, mem_addr.addr_byte[2]);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, mem_addr.addr_byte[1]);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, mem_addr.addr_byte[0]);
    value = HAL_SPI_ReadByte(HAL_SPI_ID_2);
    HAL_SPI_DisableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    return value;
  }

__attribute ((unused))
static void MC23LCVxxx_write_byte(uint32_t addr, uint8_t value)
  {
    mem_addr.addr_32 = (addr & MC23LCVxxx_MAX);
    HAL_SPI_EnableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, MC23LCVxxx_WRITE);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, mem_addr.addr_byte[2]);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, mem_addr.addr_byte[1]);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, mem_addr.addr_byte[0]);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, value);
    HAL_SPI_DisableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
  }

// get/put functions

int MC23LCVxxx_read(uint32_t address, uint8_t *buf, int len)
  {
    uint8_t txBuff[3];

    xSemaphoreTake(gSpi2_mtx, portMAX_DELAY);

    HAL_SPI_EnableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, MC23LCVxxx_READ);

    txBuff[0] = (address >> 16) & 0xFF;
    txBuff[1] = (address >> 8) & 0xFF;
    txBuff[2] = address & 0xFF;

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(HAL_SPI_ID_2, txBuff, NULL, 3))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(HAL_SPI_ID_2, NULL, buf, len))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    HAL_SPI_DisableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);

    xSemaphoreGive(gSpi2_mtx);
    return len;
  }

int MC23LCVxxx_write(uint32_t address, uint8_t *buf, int len)
  {
    uint8_t txBuff[3];
    assert(buf != NULL);
    assert(address + len < MC23LCVxxx_MAX);

    xSemaphoreTake(gSpi2_mtx, portMAX_DELAY);

    HAL_SPI_EnableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);
    HAL_SPI_WriteByte(HAL_SPI_ID_2, MC23LCVxxx_WRITE);

    txBuff[0] = (address >> 16) & 0xFF;
    txBuff[1] = (address >> 8) & 0xFF;
    txBuff[2] = address & 0xFF;

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(HAL_SPI_ID_2, txBuff, NULL, 3))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    while (kStatus_Success != HAL_SPI_ReadWrite_IT(HAL_SPI_ID_2, (uint8_t*) buf, NULL, len))
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

    HAL_SPI_DisableChip(HAL_SPI_ID_2, HAL_SPI_SS_2);

    xSemaphoreGive(gSpi2_mtx);

    return len;
  }

uint16_t MC23LCVxxx_crc(uint32_t addr, uint32_t lng)
  {
    return MC23LCVxxx_calc_crc16((addr & MC23LCVxxx_MAX), (lng & MC23LCVxxx_MAX));
  }

#endif

static uint16_t MC23LCVxxx_crc16_update(uint16_t crc, uint8_t a)
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

#if defined( MC23LCV512 )

      // *** For 23LCV512 64K x 8bit memory ***

static uint16_t calc_crc16 ( uint16_t startAddr, uint16_t len ) {
         uint8_t  ch;
         uint16_t crc;
         uint16_t i;
         //
         crc = 0;
         for ( i = 0; i < len; i++ ) {
            ch = read ( startAddr + i );
            crc = _crc16_update(crc, ch);
         }
         return ( crc );
      }

#elif defined( MC23LCV1024 )

// *** For 23LCV1024 128K x 8bit memory ***

static uint16_t MC23LCVxxx_calc_crc16(uint32_t startAddr, uint32_t len)
  {
    uint8_t ch;
    uint16_t crc;
    uint32_t i;

    xSemaphoreTake(gSpi2_mtx, portMAX_DELAY);
    crc = 0;
    for (i = 0; i < len; i++)
      {
        ch = MC23LCVxxx_read_byte(startAddr + i);
        crc = MC23LCVxxx_crc16_update(crc, ch);
      }
    xSemaphoreGive(gSpi2_mtx);
    return (crc);
  }

#endif

