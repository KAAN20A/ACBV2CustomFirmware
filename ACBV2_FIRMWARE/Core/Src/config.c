#include "config.h"

static HAL_StatusTypeDef Flash_Erase_Page(uint32_t address) {
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
  erase.NbPages = 1;

  return HAL_FLASHEx_Erase(&erase, &page_error);
}

static HAL_StatusTypeDef Flash_Write(uint32_t address, uint8_t *data,
                                     uint32_t length) {
  HAL_StatusTypeDef status = HAL_OK;
  uint64_t qword;

  for (uint32_t i = 0; i < length; i += 8) {
    memcpy(&qword, data + i, 8);
    status =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, qword);
    if (status != HAL_OK)
      break;
  }

  return status;
}

void saveConfig(void) {
  acb_config.magic_number = FLASH_BASE;

  Flash_Unlock();

  Flash_Erase_Page(FLASH_TYPEPROGRAM_DOUBLEWORD);

  Flash_Write(FLASH_BASE, (uint8_t *)&acb_config, sizeof(acb_config));

  Flash_Lock();
  HAL_Delay(10);
}

bool loadConfig(void) {
  const acb_config_t *flash_cfg =
      (const acb_config_t *)EEPROM_CONFIG_START_ADDR;


  if (flash_cfg->magic_number == EEPROM_CONFIG_MAGIC_NUMBER) {
    memcpy(&acb_config, flash_cfg, sizeof(acb_config));
    return true;
  } else {
    initConfig();
    saveConfig();
    return false;
  }
}
void initConfig(void)
{
    memset(&acb_config, 0, sizeof(acb_config));

    acb_config.magic_number = EEPROM_CONFIG_MAGIC_NUMBER;
    acb_config.version     = 1;

    acb_config.param1 = 100;
    acb_config.param2 = 50;
    acb_config.param3 = 1;
}
void saveConfig(void)
{
    if (Flash_Unlock() != HAL_OK)
    {
        Error_Handler();
    }

    if (Flash_Erase_Page(EEPROM_CONFIG_START_ADDR) != HAL_OK)
    {
        Flash_Lock();
        Error_Handler();
    }

    if (Flash_Write(EEPROM_CONFIG_START_ADDR,
                    (uint8_t *)&acb_config,
                    sizeof(acb_config)) != HAL_OK)
    {
        Flash_Lock();
        Error_Handler();
    }

    Flash_Lock();
}
return true;
}

static HAL_StatusTypeDef flash_write(uint32_t address, uint8_t *data,
                                     uint32_t length) {
  uint64_t qword;
  HAL_StatusTypeDef status = HAL_OK;

  for (uint32_t i = 0; i < length; i += 8) {
    memcpy(&qword, data + i, 8);

    status =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, qword);

    if (status != HAL_OK)
      break;
  }

  return status;
}
bool loadConfig(void) {
  const acb_config_t *flash_cfg =
      (const acb_config_t *)EEPROM_CONFIG_START_ADDR;


  if (flash_cfg->magic_number == EEPROM_CONFIG_MAGIC_NUMBER) {
    memcpy(&acb_config, flash_cfg, sizeof(acb_config));
    return true;
  } else {
    initConfig();
    saveConfig();
    return false;
  }
}

HAL_StatusTypeDef Flash_Unlock(void)
{

    if ((FLASH->CR & FLASH_CR_LOCK) == 0U)
    {
        return HAL_OK;
    }


    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;


    if (FLASH->CR & FLASH_CR_LOCK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Flash_Lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;


    if (FLASH->CR & FLASH_CR_LOCK)
    {
        return HAL_OK;
    }

    return HAL_ERROR;
}
