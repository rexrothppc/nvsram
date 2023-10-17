#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nvsram.h"
#include "ugfx_assert.h"
#include "gpio.h"

void NVSRAM_test1(void)
  {
    int stn1;
    int stn2;
    int stn3;
    int stn;
    int i;
    bool st = false;
    char hello[256];
    const char txt1[] = "HELLO world\r\n";
    const char txt2[] = "HELLO fucken world\r\n";
    const char txt3[] = "Людина в світ зерно нести повинна , як зерно носить людям колосок.Ми є. Були. І будем ми! Й Вітчизна наша з нами.";
    stn1 = strlen(txt1);
    stn2 = strlen(txt2);
    stn3 = strlen(txt3);
    strcpy(hello, txt1);
    strcpy(hello + stn1 + 1, txt2);
    strcpy(hello + stn1 + stn2 + 2, txt3);

    stn = stn1 + stn2 + stn3 + 3;
    for (i = 0; (i * stn + stn) < NVSRAM_size(); i++)
      {
        NVSRAM_write(i * stn, (const uint8_t*) hello, stn);
      }

    while (1)
      {
        for (i = 0; (i * stn + stn) < NVSRAM_size(); i++)
          {
            NVSRAM_read(i * stn, (uint8_t*) hello, stn1);
            assert(memcmp(hello, txt1, stn1) == 0);

            NVSRAM_read(i * stn + stn1 + 1, (uint8_t*) hello, stn2);
            assert(memcmp(hello, txt2, stn2) == 0);

            NVSRAM_read(i * stn + stn1 + stn2 + 2, (uint8_t*) hello, stn3);
            assert(memcmp(hello, txt3, stn3) == 0);
          }

        if (st)
          {
            GPIO_SetBit(GPIOC, BIT11);
          }
        else
          {
            GPIO_ClrBit(GPIOC, BIT11);
          }
        st ^= 1;
      }
  }
