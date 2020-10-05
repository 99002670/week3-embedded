/*
 * STM32F407xx_gpio_driver.c
 *
 *  Created on: Sep 30, 2020
 *      Author: milind
 */
#include "STM32F407xx_gpio_driver.h"

void GPIO_PeriphClkCtrl(GPIO_REGDEF_t *pGPIOx, uint8_t EnOrDis)
{
    if(EnOrDis == ENABLE){
        if(pGPIOx == GPIOA){
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB){
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC){
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD){
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE){
            GPIOE_PCLK_EN();
        }
        else if(pGPIOx == GPIOF){
            GPIOF_PCLK_EN();
        }
        else if(pGPIOx == GPIOG){
            GPIOG_PCLK_EN();
        }
        else if(pGPIOx == GPIOH){
            GPIOH_PCLK_EN();
        }
        else if(pGPIOx == GPIOI){
            GPIOI_PCLK_EN();
        }
    }
    else{
        //Disable
    }
}

void GPIO_Init(GPIO_Handle_t *GPIO_Handle)
{
    //Init mode
    uint32_t temp = 0;
    temp = (GPIO_Handle->GPIO_PinConfig.PinMode << (2 * GPIO_Handle->GPIO_PinConfig.PinNumber));
    GPIO_Handle->pGPIOx->MODER &= ~(0x3 << (2 * GPIO_Handle->GPIO_PinConfig.PinNumber));
    GPIO_Handle->pGPIOx->MODER |= temp;

    //Init speed
    temp = 0;
    temp = (GPIO_Handle->GPIO_PinConfig.PinSpeed << (2 * GPIO_Handle->GPIO_PinConfig.PinNumber));
    GPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * GPIO_Handle->GPIO_PinConfig.PinNumber));
    GPIO_Handle->pGPIOx->OSPEEDR |= temp;

    //Init pull-up and pull-down
    temp = 0;
    temp = (GPIO_Handle->GPIO_PinConfig.PinPuPdCtrl << (2 * GPIO_Handle->GPIO_PinConfig.PinNumber));
    GPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << (2 * GPIO_Handle->GPIO_PinConfig.PinNumber));
    GPIO_Handle->pGPIOx->PUPDR |= temp;

    //Init output type
    temp = 0;
    temp = (GPIO_Handle->GPIO_PinConfig.PinOpType << GPIO_Handle->GPIO_PinConfig.PinNumber);
    GPIO_Handle->pGPIOx->OTYPER &= ~(0x1 << (2 * GPIO_Handle->GPIO_PinConfig.PinNumber));
    GPIO_Handle->pGPIOx->OTYPER |= temp;

    //Init alternating fuction
    temp = 0;
    if(GPIO_Handle->GPIO_PinConfig.PinMode == GPIO_MODE_AFN)
    {
        uint8_t temp1 = 0;
        uint8_t temp2 = 0;
        temp1 = GPIO_Handle->GPIO_PinConfig.PinNumber / 8;
        temp2 = GPIO_Handle->GPIO_PinConfig.PinNumber % 8;
        GPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        GPIO_Handle->pGPIOx->AFR[temp1] |= GPIO_Handle->GPIO_PinConfig.PinAltFunMode << (4 * temp2);
    }
}

void GPIO_Dinit(GPIO_REGDEF_t *pGPIOx)
{
    if(pGPIOx == GPIOA){
        GPIOA_REG_RESET();
    }
    else if(pGPIOx == GPIOB){
        GPIOB_REG_RESET();
    }
    else if(pGPIOx == GPIOC){
        GPIOC_REG_RESET();
    }
    else if(pGPIOx == GPIOD){
        GPIOD_REG_RESET();
    }
    else if(pGPIOx == GPIOE){
        GPIOE_REG_RESET();
    }
    else if(pGPIOx == GPIOF){
        GPIOF_REG_RESET();
    }
    else if(pGPIOx == GPIOG){
        GPIOG_REG_RESET();
    }
    else if(pGPIOx == GPIOH){
        GPIOH_REG_RESET();
    }
    else if(pGPIOx == GPIOI){
        GPIOI_REG_RESET();
    }
}

uint8_t GPIO_ReadFromInputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value = 0;
    value = (uint8_t)(pGPIOx->IDR >> PinNumber) & (0x000000001);
    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_REGDEF_t *pGPIOx)
{
    uint16_t value = 0;
    value = (uint16_t)(pGPIOx->IDR);
    return value;
}

void GPIO_WriteToOutputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
    if(value == GPIO_PIN_SET){
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else{
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_REGDEF_t *pGPIOx, uint16_t value)
{
        pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}
