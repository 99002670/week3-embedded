/*
 * STM32F407xx.h
 *
 *  Created on: Sep 29, 2020
 *      Author: milind
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

// Base address for memory
#define FLASH_BASE_ADDR 	0x08000000UL
#define SRAM1_BASE_ADDR 	0x20000000UL
#define SRAM2_BASE_ADR		0x2001C000UL
#define SRAM				SRAM1_BASE_ADDR
#define ROM_BASE_ADDR	 	0x1FFF0000UL

//Base address for bus
#define APB1_BASE_ADDR		0x40000000UL
#define APB2_BASE_ADDR		0x40010000UL
#define AHB1_BASE_ADDR		0x40020000UL
#define AHB2_BASE_ADDR		0x50000000UL

//Base address of peripherals hanging on AHB1
#define GPIOA_BASE_ADDR		(AHB1_BASE_ADDR + 0x0000UL)
#define GPIOB_BASE_ADDR		(AHB1_BASE_ADDR + 0x0400UL)
#define GPIOC_BASE_ADDR		(AHB1_BASE_ADDR + 0x0800UL)
#define GPIOD_BASE_ADDR 	(AHB1_BASE_ADDR + 0x0C00UL)
#define GPIOE_BASE_ADDR		(AHB1_BASE_ADDR + 0x1000UL)
#define GPIOF_BASE_ADDR		(AHB1_BASE_ADDR + 0x1400UL)
#define GPIOG_BASE_ADDR		(AHB1_BASE_ADDR + 0x1800UL)
#define GPIOH_BASE_ADDR		(AHB1_BASE_ADDR + 0x1C00UL)
#define GPIOI_BASE_ADDR		(AHB1_BASE_ADDR + 0x2000UL)
#define RCC_BASE_ADDR		(0x40023800)

//Base address of peripherals hanging on APB1
#define I2C1_BASE_ADDR		0x40005400
#define I2C2_BASE_ADDR		0x40005800
#define I2C3_BASE_ADDR		0x40005C00
#define SPI2_BASE_ADDR		0x40003800
#define SPI3_BASE_ADDR		0x40003C00
#define USART2_BASE_ADDR	0x40004400
#define USART3_BASE_ADDR	0x40004800

//Base address of peripherals hanging on APB2
#define SPI1_BASE_ADDR		0x40013000
#define USART1_BASE_ADDR	0x40011000
#define USART6_BASE_ADDR	0x40011400
#define EXTI_BASE_ADDR		0x40013C00
#define SYSCFG_BASE_ADDR	0x40013800

//Register structure of GPIO peripheral
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_REGDEF_t;

//GPIO peripheral definition
#define GPIOA			((GPIO_REGDEF_t*) GPIOA_BASE_ADDR)
#define GPIOB			((GPIO_REGDEF_t*) GPIOB_BASE_ADDR)
#define GPIOC			((GPIO_REGDEF_t*) GPIOC_BASE_ADDR)
#define GPIOD			((GPIO_REGDEF_t*) GPIOD_BASE_ADDR)
#define GPIOE			((GPIO_REGDEF_t*) GPIOE_BASE_ADDR)
#define GPIOF			((GPIO_REGDEF_t*) GPIOF_BASE_ADDR)
#define GPIOG			((GPIO_REGDEF_t*) GPIOG_BASE_ADDR)
#define GPIOH			((GPIO_REGDEF_t*) GPIOH_BASE_ADDR)
#define GPIOI			((GPIO_REGDEF_t*) GPIOI_BASE_ADDR)

//Register structure of RCC
typedef struct
{
		volatile uint32_t RCC_CR;
		volatile uint32_t RCC_PLLCFGR;
		volatile uint32_t RCC_CFGR;
		volatile uint32_t RCC_CIR;
		volatile uint32_t RCC_AHB1RSTR;
		volatile uint32_t RCC_AHB2RSTR;
		volatile uint32_t RCC_AHB3RSTR;
		volatile uint32_t RCC_RESERVED0;
		volatile uint32_t RCC_APB1RSTR;
		volatile uint32_t RCC_APB2RSTR;
		volatile uint32_t RCC_RESERVED1[2];
		volatile uint32_t RCC_AHB1ENR;
		volatile uint32_t RCC_AHB2ENR;
		volatile uint32_t RCC_AHB3ENR;
		volatile uint32_t RCC_RESERVED2;
		volatile uint32_t RCC_APB1ENR;
		volatile uint32_t RCC_APB2ENR;
		volatile uint32_t RCC_RESERVED3[2];
		volatile uint32_t RCC_AHB1LPENR;
		volatile uint32_t RCC_AHB2LPENR;
		volatile uint32_t RCC_AHB3LPENR;
		volatile uint32_t RCC_RESERVED4;
		volatile uint32_t RCC_APB1LPENR;
		volatile uint32_t RCC_APB2LPENR;
		volatile uint32_t RCC_RESERVED5[2];
		volatile uint32_t RCC_BDCR;
		volatile uint32_t RCC_CSR;
		volatile uint32_t RCC_RESERVED6[2];
		volatile uint32_t RCC_SSCGR;
		volatile uint32_t RCC_PLLI2SCFGR;
}RCC_REGDEF_t;

#define RCC				((RCC_REGDEF_t*)RCC_BASE_ADDR)

//CLOCK ENABLING
#define GPIOA_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN() (RCC->RCC_AHB1ENR |= (1<<8))

//CLOCK DISABLING
#define GPIOB_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOC_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOA_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DIS() (RCC->RCC_AHB1ENR &= ~(1<<8))

//Some important MACROS
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#define GPIOA_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &= (1<<0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<1)); (RCC->RCC_AHB1RSTR &= (1<<1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<2)); (RCC->RCC_AHB1RSTR &= (1<<2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<3)); (RCC->RCC_AHB1RSTR &= (1<<3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<4)); (RCC->RCC_AHB1RSTR &= (1<<4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<5)); (RCC->RCC_AHB1RSTR &= (1<<5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<6)); (RCC->RCC_AHB1RSTR &= (1<<6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<7)); (RCC->RCC_AHB1RSTR &= (1<<7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &= (1<<8));}while(0)

//Register structure of I2C peripheral
typedef struct
{
	volatile uint32_t I2C_CR1;
	volatile uint32_t I2C_CR2;
	volatile uint32_t I2C_OAR1;
	volatile uint32_t I2C_OAR2;
	volatile uint32_t I2C_DR;
	volatile uint32_t I2C_SR1;
	volatile uint32_t I2C_SR2;
	volatile uint32_t I2C_CCR;
	volatile uint32_t I2C_TRISE;
	volatile uint32_t I2C_FLTR;
}I2C_REGDEF_t;

//Register structure of SPI peripheral
typedef struct
{
	volatile uint32_t SPI_CR1;
	volatile uint32_t SPI_CR2;
	volatile uint32_t SPI_SR;
	volatile uint32_t SPI_DR;
	volatile uint32_t SPI_CRCPR;
	volatile uint32_t SPI_RXCRCR;
	volatile uint32_t SPI_TXCRCR;
	volatile uint32_t SPI_I2SCFGR;
	volatile uint32_t SPI_I2SPR;
}SPI_REGDEF_t;

//Register structure of USART peripheral
typedef struct
{
	volatile uint32_t USART_SR;
	volatile uint32_t USART_DR;
	volatile uint32_t USART_BRR;
	volatile uint32_t USART_CR1;
	volatile uint32_t USART_CR2;
	volatile uint32_t USART_CR3;
	volatile uint32_t USART_GTPR;
}USART_REGDEF_t;

#endif /* INC_STM32F407XX_H_ */
