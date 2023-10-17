#ifndef NVSRAM_H_
#define NVSRAM_H_

#ifdef __cplusplus
extern "C" {
#endif




//#define MC23LCV1024 1
#define IS66WVS1MJ8B 1

#ifdef MC23LCV1024
#include "nvsram_23lcv1024.h"
#define NVSRAM_init    MC23LCVxxx_init
#define NVSRAM_size  MC23LCVxxx_size
#define NVSRAM_erase   MC23LCVxxx_erase
#define NVSRAM_read    MC23LCVxxx_read
#define NVSRAM_write   MC23LCVxxx_write
#define NVSRAM_crc     MC23LCVxxx_crc

#else
#include "nvsram_is66wvs1m8bll104.h"
#define NVSRAM_init    IS66WVS1MJ8B_init
#define NVSRAM_size    IS66WVS1MJ8B_size
#define NVSRAM_erase   IS66WVS1MJ8B_erase
#define NVSRAM_read    IS66WVS1MJ8B_Read
#define NVSRAM_write   IS66WVS1MJ8B_Write
#define NVSRAM_crc     IS66WVS1MJ8B_crc
#endif

void NVSRAM_test1(void);

#ifdef __cplusplus
}
#endif

#endif /* NVSRAM_H_ */
