#include "drv8323.h

#define DRV_CS_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define DRV_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

void DRV8323_Init(void) {

  DRV_CS_HIGH();
  HAL_Delay(10);

  uint16_t ctrlRegValue = 0;
  ctrlRegValue |= (0 << 9);
  ctrlRegValue |= (0 << 8);
  ctrlRegValue |= (0 << 7);
  ctrlRegValue |= (1 << 5);
  DRV8323_WriteReg(0x02, ctrlRegValue);

  uint16_t hsRegValue = (3 << 8) | (7 << 4) | (7);
  DRV8323_WriteReg(0x03, hsRegValue);

  uint16_t lsRegValue = (0 << 9) | (0 << 7) | (7 << 4) | (7);
  DRV8323_WriteReg(0x04, lsRegValue);

  uint16_t ocpValue = (0 << 9) | (1 << 7) | (1 << 4) | (0xA);
  DRV8323_WriteReg(0x05, ocpValue);

  uint16_t csaValue = 0;
  DRV8323_WriteReg(0x06, csaValue);

  HAL_Delay(1);

  DRV8323_WriteReg(0x02, ctrlRegValue | (1 << 3));
}

uint16_t DRV8323_ReadReg(uint8_t reg_addr) {
  uint16_t cmd = 0;
  uint16_t data16;

  cmd = (1U << 15) | ((uint16_t)(reg_addr & 0x0F) << 11);

  tx_spi_buf[0] = (uint8_t)(cmd >> 8);
  tx_spi_buf[1] = (uint8_t)(cmd & 0xFF);

  DRV_CS_LOW();

  HAL_Delay(20);
  DRV_CS_HIGH();

  data16 = ((uint16_t)rx_spi_buf[0] << 8) | rx_spi_buf[1];

  return (data16 & 0x07FF);
}

void DRV8323_WriteReg(uint8_t reg_addr, uint16_t value) {
  uint16_t cmd;

  cmd = ((uint16_t)(reg_addr & 0x0F) << 11) | (value & 0x07FF);

  tx_spi_buf[0] = (uint8_t)(cmd >> 8);
  tx_spi_buf[1] = (uint8_t)(cmd & 0xFF);

  DRV_CS_LOW();

  HAL_Delay(20);

  DRV_CS_HIGH();
}
"
