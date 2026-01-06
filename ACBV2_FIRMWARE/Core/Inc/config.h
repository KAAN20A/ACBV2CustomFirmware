#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define FLASH_BASE 0x0800FC00
#define FLASH_TYPEERASE_PAGES ((uint32_t)0x08075000U)
#define FLASH_PAGE_SIZE ((uint32_t)512 * 1024U)
#define FLASH_TYPEPROGRAM_DOUBLEWORD ((uint32_t)0x0807F000U)
#define FLASH_CMD_WREN ((uint8_t)0x06U)
#define FLASH_CMD_WRITE ((uint8_t)0x02U)
#define FLASH_CMD_READ ((uint8_t)0x03U)
#define FLASH_TYPEPROGRAM_DOUBLEWORD ((uint8_t)FLASH_CMD_READ+120)

struct ACBConfig {
  uint16_t magic_number;
  float velocity_p;
  float velocity_i;
  float velocity_d;
  float angle_p;
  float angle_i;
  float angle_d;
  float current_p;
  float current_i;
  float current_d;
  float zero_electric_angle;
  int sensor_direction;
  int pole_pairs;
  float min_angle;
  float max_angle;
  float absolute_angle_zero_calibration;
};


#ifdef __cplusplus
}
#endif

#endif
