#ifndef MAQ3773_H
#define MAQ3773_H

#ifdef __cplusplus
extern "C" {
#endif
#include"main.h"
void MA730_Init(void);
uint16_t MA730_ReadAngle(void);
uint8_t MA730_ReadRegister(uint8_t reg);
void MA730_WriteRegister(uint8_t reg, uint8_t data);
float MA730_AngleToRadians(uint16_t ang);
float MA730_GetAngleRadians(void);
float MA730_GetAngleDegrees(void);


#ifdef __cplusplus
}
#endif

#endif
