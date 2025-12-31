struct ACBConfig {
  uint16_t magic_number;  // Magic number to verify valid data

  // PID Configuration
  float velocity_p;
  float velocity_i;
  float velocity_d;
  float angle_p;
  float angle_i;
  float angle_d;
  float current_p;
  float current_i;
  float current_d;

  // Sensor Calibration Values
  float zero_electric_angle;
  int sensor_direction;

  // Motor Configuration
  int pole_pairs;

  // Angle Limits Configuration
  float min_angle;  // Minimum allowed angle in degrees
  float max_angle;  // Maximum allowed angle in degrees

  // Absolute Angle Calibration
  float absolute_angle_zero_calibration;  // Absolute angle at zero position during calibration

  // Add more configuration variables here as needed
  // Example:
  // float motor_voltage_limit;
  // uint16_t encoder_ppr;
  // bool debug_mode;
};


void saveConfig(void)
{
    acb_config.magic_number = EEPROM_CONFIG_MAGIC_NUMBER;

    HAL_I2C_Mem_Write(&hi2c1,
                      EEPROM_I2C_ADDRESS,
                      EEPROM_CONFIG_START_ADDR,
                      I2C_MEMADD_SIZE_16BIT,
                      (uint8_t*)&acb_config,
                      sizeof(acb_config),
                      100);

    HAL_Delay(10); // EEPROM page write süresi
}

bool loadConfig(void)
{
    HAL_I2C_Mem_Read(&hi2c1,
                     EEPROM_I2C_ADDRESS,
                     EEPROM_CONFIG_START_ADDR,
                     I2C_MEMADD_SIZE_16BIT,
                     (uint8_t*)&acb_config,
                     sizeof(acb_config),
                     100);

    if(acb_config.magic_number != EEPROM_CONFIG_MAGIC_NUMBER)
    {
        initConfig();
        saveConfig();
        return false;
    }

    return true;
}
void eeprom_write_enable()
{
    uint8_t cmd = EEPROM_CMD_WREN;
    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    EEPROM_CS_HIGH();
}
void saveConfig(void)
{
    acb_config.magic_number = EEPROM_CONFIG_MAGIC_NUMBER;

    uint8_t cmd[3];
    cmd[0] = EEPROM_CMD_WRITE;
    cmd[1] = 0x00;
    cmd[2] = 0x00;

    eeprom_write_enable();

    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 3, 100);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&acb_config, sizeof(acb_config), 100);
    EEPROM_CS_HIGH();

    HAL_Delay(5);
}
bool loadConfig(void)
{
    uint8_t cmd[3];
    cmd[0] = EEPROM_CMD_READ;
    cmd[1] = 0x00;
    cmd[2] = 0x00;

    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 3, 100);
    HAL_SPI_Receive(&hspi1, (uint8_t*)&acb_config, sizeof(acb_config), 100);
    EEPROM_CS_HIGH();

    if(acb_config.magic_number != EEPROM_CONFIG_MAGIC_NUMBER)
    {
        initConfig();
        saveConfig();
        return false;
    }

    return true;
}
