/*
 * STM32F407xx_gpio_driver.h
 *
 *  Created on: Sep 30, 2020
 *      Author: milind
 */
#ifndef __STM32F407XX_GPIO_DRIVER_H__
#define __STM32F407XX_GPIO_DRIVER_H__

#include "STM32F407xx.h"

typedef struct
{
  uint8_t PinNumber;
  uint8_t PinMode;
  uint8_t PinSpeed;
  uint8_t PinOpType;
  uint8_t PinPuPdCtrl;
  uint8_t PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct
{
  GPIO_REGDEF_t *pGPIOx;
  GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

//GPIO_PinNumber of pGPIOx
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15

//Possible modes
#define GPIO_MODE_IN         0
#define GPIO_MODE_OUT        1
#define GPIO_MODE_AFN        2
#define GPIO_MODE_ANALOG     3

//Possible speeds
#define GPIO_SPEED_LOW       0
#define GPIO_SPEED_MEDIUM    1
#define GPIO_SPEED_HIGH      2
#define GPIO_SPEED_VERYHIGH  3

//Possible output types
#define GPIO_OP_TYPE_PP      0
#define GPIO_OP_TYPE_OD      1

//Possible PUPD modes
#define GPIO_NO_PUPD         0
#define GPIO_PU              1
#define GPIO_PD              2


void GPIO_Init(GPIO_Handle_t *GPIO_Handle);
void GPIO_Dinit(GPIO_REGDEF_t *pGPIOx);

void GPIO_PeriphClkCtrl(GPIO_REGDEF_t *pGPIOx, uint8_t EnOrDis);

uint8_t GPIO_ReadFromInputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_REGDEF_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_REGDEF_t *pGPIOx, uint16_t Value);

void GPIO_ToggleOutputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber);

#endif
