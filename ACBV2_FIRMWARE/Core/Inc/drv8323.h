#ifndef DRV8323_H
#define DRV8323_H

#ifdef __cplusplus
extern "C" {
#endif
#include"main.h"
void DRV8323_Init(void);
uint16_t DRV8323_ReadReg(uint8_t reg_addr);
DRV8323_WriteReg(uint8_t reg_addr, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif
