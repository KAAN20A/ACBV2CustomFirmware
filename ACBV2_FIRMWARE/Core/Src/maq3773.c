void MA730_Init(void)
{
    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

    // Low / High sensitivity
    MA730_WriteRegister(0x06,
        (0b00000000 << 5) | (0b00000111 << 2)
    );

    // Filter window
    MA730_WriteRegister(0x0E, 85);

    // Hysteresis
    MA730_WriteRegister(0x10, 156);
}

uint16_t MA730_ReadAngle(void)
{
    uint16_t angle = 0;

    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_SPI_TransmitReceive(&hspi1,
                            (uint8_t *)&(uint16_t){0x0000},
                            (uint8_t *)&angle,
                            1,
                            10);

    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    return angle;
}

uint8_t MA730_ReadRegister(uint8_t reg)
{
    uint16_t rx = 0;
    uint16_t read_cmd = 0x4000 | ((reg & 0x1F) << 8);
    uint8_t result;

    // --- 1. Frame: Read Request ---
    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_SPI_TransmitReceive(&hspi1,
                            (uint8_t *)&read_cmd,
                            (uint8_t *)&rx,
                            1,
                            10);

    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // --- 2. Frame: Data Read ---
    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    uint16_t dummy = 0x0000;
    HAL_SPI_TransmitReceive(&hspi1,
                            (uint8_t *)&dummy,
                            (uint8_t *)&rx,
                            1,
                            10);

    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    result = (rx >> 8) & 0xFF;
    return result;
}

void MA730_WriteRegister(uint8_t reg, uint8_t data)
{
    uint16_t tx;
    uint16_t rx;

    // ---- Frame 1 : Write Command ----
    tx = 0x8000 | ((reg & 0x1F) << 8) | data;

    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_SPI_TransmitReceive(&hspi1,
                            (uint8_t *)&tx,
                            (uint8_t *)&rx,
                            1,
                            10);

    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(20);

    // ---- Frame 2 : Dummy Readback ----
    uint16_t dummy = 0x0000;

    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_SPI_TransmitReceive(&hspi1,
                            (uint8_t *)&dummy,
                            (uint8_t *)&rx,
                            1,
                            10);

    HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

    // İstersen hata kontrolü yapılır:
    /*
    if((rx >> 8) != data)
    {
        printf("MA730 WRITE ERROR! got=%02X expected=%02X\r\n",
               (rx >> 8), data);
    }
    */
}

float MA730_AngleToRadians(uint16_t ang)
{
    return ((float)ang) * 2.0f * M_PI / 65536.0f;
}

float MA730_GetAngleRadians(void)
{
    return MA730_AngleToRadians(MA730_ReadAngle());
}

float MA730_GetAngleDegrees(void)
{
    return MA730_GetAngleRadians() * 180.0f / M_PI;
}
