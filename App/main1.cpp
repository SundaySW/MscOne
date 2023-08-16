#include <main.h>
#include <i2c.h>
#include <dac.h>
#include <fdcan.h>
#include <adc.h>
#include "stm32g4xx_hal.h"
#include "MscOne.hpp"
#include "tim.h"

extern "C"
{
    void initDevice()
    {
        if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
            Error_Handler();
        if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
            Error_Handler();
        MscOne::getInstance().initPerf(&hadc1, &hadc2, &hi2c2, &hdac1, &htim2, &htim4);
        MscOne::getInstance().Start();
        HAL_TIM_Base_Start_IT(&htim1);
//        HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
    }
    void device_main_loop()
    {
        MscOne::getInstance().Poll();
    }

    void OnSysTickTimer()
    {
        MscOne::getInstance().OnTimerINT(1);
    }

    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        if (hi2c == &hi2c2)
            MscOne::getInstance().getI2CMaster().WriteHandler();
    }
    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        if (hi2c == &hi2c2)
            MscOne::getInstance().getI2CMaster().ReadHandler();
    }

    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
    {
        if (hi2c == & hi2c2)
            MscOne::getInstance().getI2CMaster().ErrorHandler();
    }

    void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        if (hi2c == &hi2c2)
            MscOne::getInstance().getI2CMaster().ReadHandler();
    }

    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        if (hi2c == &hi2c2)
            MscOne::getInstance().getI2CMaster().WriteHandler();
    }

    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
    {
        MscOne::getInstance().processADCCallback(hadc);
    }

    void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
    {
        MscOne::getInstance().processTimCallBack(htim);
    }

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM1){
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
    }

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
        if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
        {
            /* Retrieve Rx messages from RX FIFO0 */
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
                Error_Handler();
            MscOne::getInstance().Port.OnRX(RxHeader, RxData);
        }
    }
}
