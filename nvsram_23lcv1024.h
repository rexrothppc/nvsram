#ifndef NVSRAM_23LCV1024_H_
#define NVSRAM_23LCV1024_H_


#ifdef __cplusplus
extern "C" {
#endif

bool MC23LCVxxx_init(void);
uint32_t MC23LCVxxx_size(void);
void MC23LCVxxx_erase(void);
int MC23LCVxxx_read(uint32_t address, uint8_t *buf, int len);
int MC23LCVxxx_write(uint32_t address, uint8_t *buf, int len);
uint16_t MC23LCVxxx_crc(uint32_t addr, uint32_t lng);


#ifdef __cplusplus
}
#endif


#endif /* NVSRAM_23LCV1024_H_ */
