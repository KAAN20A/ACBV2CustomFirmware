#include"foc.h"
void Motor_Init(void)
{

    HAL_Delay(1000);


    uint8_t msg[] = "---------- ACB BOOTING ----------\r\n";
    HAL_UART_Transmit(&huart1, msg, strlen((char *)msg), HAL_MAX_DELAY);


    IOSetup();


    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_spi_buf, rx_spi_buf, BUFFER_SIZE);
    HAL_SPI_TransmitReceive_DMA(&hspi2, tx_spi_buf, rx_spi_buf, BUFFER_SIZE);


    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1N);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2N);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3N);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    MA730_Init();
    DRV8323_Init();

}


void Read_Config(void)
{
    ACBConfig *flashConfig = loadconfig();
}
