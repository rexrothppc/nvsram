#ifndef IS66WVS1M8BLL104_H_
#define IS66WVS1M8BLL104_H_


#ifdef __cplusplus
extern "C" {
#endif



bool IS66WVS1MJ8B_init(void);
uint32_t IS66WVS1MJ8B_length(void);
void IS66WVS1MJ8B_erase(void);
int IS66WVS1MJ8B_Read(uint32_t address, uint8_t *data, int len);
int IS66WVS1MJ8B_Write(uint32_t address, const uint8_t *data, int len);
uint16_t IS66WVS1MJ8B_crc(uint32_t addr, uint32_t lng);



#ifdef __cplusplus
}
#endif


#endif /* IS66WVS1M8BLL104_H_ */
