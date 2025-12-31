#ifndef __DRV_8323_SPI_H
#define __DRV_8323_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  SPI_HandleTypeDef *hspi;
} DRV8323_HandleTypeDef;



#ifdef __cplusplus
}
#endif

#endif
