/*
 * stm32f407xx.h
 *
 *  Created on: Apr 13, 2023
 *  Author: Bacvu
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stdint.h>
#include <string.h>

#define __vo volatile
/********************START:Processor Specific Detail **************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0 				 ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 				 ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 				 ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 				 ((__vo uint32_t*)0xE000E10C)

//clear
#define NVIC_ICER0 				 ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1 				 ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2 				 ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3 				 ((__vo uint32_t*)0xE000E18C)

/*
 * ARM cotex Mx Processor Priority Register Address Calulation
 */
#define NVIC_PR_BASE_ADDR		  ((__vo uint32_t*)0xE000E400)


#define	NO_PR_BITS_IMPLEMENTED 	 4



/**
 * base addresses of flash and SRam meomories
 *
 */

#define FLASH_BASEADDR				 0x08000000U    //(main memory) adress always unsigned so need asigned it
#define SRAM1_BASEADDR				 0x20000000U	//112kb x 1024 = 114688 ->hex
#define SRAM2_BASEADDR				 0x2001C000U	//base+hex
#define ROM_BASEADDR				 0x1FFF0000U    //system memory
#define SRAM 						 SRAM1_BASEADDR



/*
 * AHBx and AHPx bus peripheral base addreses
 */



#define PERIPH_BASE					 0x40000000U
#define AHB1PERIPH_BASEADDR		     0x40020000U	//bus MAX168 MHZ clock
#define AHB2PERIPH_BASEADDR		     0x50000000U	//bus MAX168 MHz clock
#define APB1PERIPH_BASEADDR		     PERIPH_BASE	//bus MAX42  MHZ clock
#define APB2PERIPH_BASEADDR		     0x40010000U	//bus max84  Mhz clock

/*
 *Base Addresses of peripherals which are hanging on AHB1 bus
 *
 *
 */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 *l
 */
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)// can work 2 both sync and not

#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00) //no sp sync clock
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)


/*
 * Base address of peripherals which are hanging on APB2 bus
 *
 */
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

/*
 * configure register using structures
 *	SPI peripherals
 *
 *
 */
typedef struct {

	__vo uint32_t MODER;			//0x00 offset :GPIO port mode register
	__vo uint32_t OTYPER;			//0x04 offset :GPIO port output type register
	__vo uint32_t OSPEEDR;			//0x08 offset :GPIO port output speed register
	__vo uint32_t PUPDR;			//0x0C offset :GPIO port pull-up/pull-down register
	__vo uint32_t IDR;				//0x10 offset :GPIO port input data register
	__vo uint32_t ODR;				//0x14 offset :GPIO port output data register
	__vo uint32_t BSRR;				//0x18 offset :GPIO port bit set/reset register
	__vo uint32_t LCKR;				//0x1C offset :GPIO port configuration lock register
	__vo uint32_t AFR[2];        	//AFR[0] LOW and AFR[1] HIGH :0x20 and 0x24 :GPIO alternate function low register /GPIO alternate function HIGH register

}GPIO_regDef_t;
/*
 *
 * RCC member (reset clock control)
 */

typedef struct {

	__vo uint32_t CR;  				//RCC clock control register : 0x00
	__vo uint32_t PLLCFGR;			//RCC PLL configuration register:0x04
	__vo uint32_t CFGR;				//RCC clock configuration register:0x08
	__vo uint32_t CIR;				//RCC clock interrupt register :0x0C
	uint32_t 		 RESERVED0;
	__vo uint32_t AHB_RSTR[3];		//RCC AHBx peripheral reset register :AHB_RSTR[0] 0x10,AHB_RSTR[1] 0X14,0X18
	uint32_t 		 RESERVED1[2];
	__vo uint32_t APB_RSTR[2];  	//RCC APBx peripheral reset register :APB_RSTR[0] 0x20, APB_SRTR[1] 0X24
	__vo uint32_t AHB_ENR[3];   	//RCC AHBx peripheral clock enable register :0x30 -0x34 -0x38
	uint32_t		 RESERVED3[2];
	__vo uint32_t APB_ENR[2];   	//RCC APBx peripheral clock enable register :0x40 -0x44
	__vo uint32_t AHB_LPENR[3]; 	//RCC AHBx peripheral clock enable in low power mode register :0x50- 0x54- 0x58
	__vo uint32_t C_APB_LPENR[2];   //RCC APBx peripheral clock enable in low power mode register :0x60- 0x64- 0x70
	__vo uint32_t BDCR;				//RCC Backup domain control register -0x70
	__vo uint32_t CSR; 				//RCC clock control & status register -0x74
	__vo uint32_t SSCGR;			//RCC spread spectrum clock generation register :0x80
	__vo uint32_t PLLI_SCFGR;       //RCC PLLI2S configuration register :0x84

}RCC_RegDef_t;

/*
 * pheripheral register dfefinition structure for EXTI
 *
 *
 */
typedef struct {
	__vo uint32_t IMR;  //offset : 0x00
	__vo uint32_t EMR;  //offset : 0x04
	__vo uint32_t RTSR; //offset : 0x08
	__vo uint32_t FTSR; //offset : 0x0C
	__vo uint32_t SWIER;//offset : 0x10
	__vo uint32_t PR;   //offset : 0x14

}EXTI_RegDef_t;


/*
 * pheripheral register definition structure for SYSCFG
 */
typedef struct {
	__vo uint32_t MEMRMP;       //offset   : 0x00
	__vo uint32_t PMC;  	    //offset   : 0x04
	__vo uint32_t EXTICR[4];    //offset   : 0x08-0x14
	uint32_t 	  RESERVED1[2]; //reserved : 0x18-0x1C
	__vo uint32_t CMPCR;        //offset   : 0x20
	uint32_t 	  RESERVED2[2];	//Reserved : 0x24-0x28
	__vo uint32_t CFGR;         //offset   : 0x14
}SYSCFG_RegDef_t;
/*
 * create macro GPIO contains base to regDe(ep kieu con tro su dung GPIO to reg )
 */

#define GPIOA 					((GPIO_regDef_t*)GPIOA_BASEADDR) //typecasted to xxx_RegDef_t
#define GPIOB 					((GPIO_regDef_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_regDef_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_regDef_t*)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_regDef_t*)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_regDef_t*)GPIOF_BASEADDR)
#define GPIOG 					((GPIO_regDef_t*)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_regDef_t*)GPIOH_BASEADDR)
#define GPIOI 					((GPIO_regDef_t*)GPIOI_BASEADDR)


#define RCC 					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI 					((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Enable Maros for GPIOx peripheral
 */


#define GPIOA_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<0)) //store the address value AHB1 bus
#define GPIOB_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<1))
#define GPIOC_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<2))
#define GPIOD_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<3))
#define GPIOE_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<4))
#define GPIOF_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<5))
#define GPIOG_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<6))
#define GPIOH_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<7))
#define GPIOI_PCLOCK_EN()      (RCC->AHB_ENR[0] |= (1<<8))


/*
 * Enable clock for I2Cx peripherals
 *
 */
#define I2C1_PCLK_EN()			(RCC->APB_ENR[0] |= (1<<21)) //store the address value apb1 set bit 21th into 1
#define I2C2_PCLK_EN()			(RCC->APB_ENR[0] |= (1<<22))
#define I2C3_PCLK_EN()			(RCC->APB_ENR[0] |= (1<<23))
/*
 * Enable clock for SPIx peripherals
 */

#define SPI1_PCLK_EN()			(RCC->APB_ENR[1] |= (1<<12)) //store into arr 2 of bus APB set bit 22th into 1
#define SPI2_PCLK_EN()			(RCC->APB_ENR[0] |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->APB_ENR[0] |= (1<<15))

/*
 * clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		(RCC->APB_ENR[1] |= (1<<4))
#define USART6_PCLK_EN()		(RCC->APB_ENR[1] |= (1<<5))
#define USART2_PCLK_EN()		(RCC->APB_ENR[0] |= (1<<17))
#define USART3_PCLK_EN()		(RCC->APB_ENR[0] |= (1<<18))

/*
 *clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()		(RCC->APB_ENR[1] |= (1<<14))

/*
 * clock disable Macros for GPIOx peripherals
 *
 */
#define GPIOA_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<4))
#define GPIOF_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<5))
#define GPIOG_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<6))
#define GPIOH_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<7))
#define GPIOI_PCLK_DI()			(RCC->AHB_ENR[0] &= ~(1<<8))


/*
 * clock disable Macros for I2Cx peripherals
 *
 */

#define I2C1_PCLK_DI()			(RCC->APB_ENR[0] &= ~(1<<21))
#define I2C2_PCLK_DI()			(RCC->APB_ENR[0] &= ~(1<<22))
#define I2C3_PCLK_DI()			(RCC->APB_ENR[0] &= ~(1<<23))
/*
 * clock disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()			(RCC->APB_ENR[1] &= ~(1<<12))
#define SPI2_PCLK_DI()			(RCC->APB_ENR[0] &= ~(1<<14))
#define SPI3_PCLK_DI()			(RCC->APB_ENR[0] &= ~(1<<15))

/*
 * clock disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()		(RCC->APB_ENR[1] &= ~(1<<4))
#define USART6_PCLK_DI()		(RCC->APB_ENR[1] &= ~(1<<5))
#define USART2_PCLK_DI()		(RCC->APB_ENR[0] &= ~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB_ENR[0] &= ~(1<<18))

/*
 *
 * Macros to reset GPIOx peripherals
 *
 */
#define GPIOA_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<0));  (RCC->AHB_RSTR[0] &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<1));  (RCC->AHB_RSTR[0] &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<2));  (RCC->AHB_RSTR[0] &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<3));  (RCC->AHB_RSTR[0] &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<4));  (RCC->AHB_RSTR[0] &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<5));  (RCC->AHB_RSTR[0] &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<6));  (RCC->AHB_RSTR[0] &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<7));  (RCC->AHB_RSTR[0] &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB_RSTR[0] |= (1<<8));  (RCC->AHB_RSTR[0] &= ~(1<<8));}while(0)


/*
 * returns port code for given GPIOx base address
 *
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x==GPIOA)? 0:\
										(x==GPIOB)?  1:\
										(x==GPIOC)?  2:\
										(x==GPIOD)?  3:\
										(x==GPIOE)?  4:\
										(x==GPIOF)?  5:\
										(x==GPIOG)?  6:\
										(x==GPIOH)?  7:0)
//some generic marcos
#define ENABLE  		1
#define DISABLE 		0
#define SET     		ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

/*
 * IRQ(interrupt request)
 * is the number first column
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1	    7
#define IRQ_NO_EXTI2	    8
#define IRQ_NO_EXTI3	    9
#define IRQ_NO_EXTI4	    10
#define IRQ_NO_EXTI9_5	    23
#define IRQ_NO_EXTI15_10	40

//Proriy macros
#define NVIC_IRQ_PRI0       0
#define NVIC_IRQ_PRI1       1
#define NVIC_IRQ_PRI2       2
#define NVIC_IRQ_PRI3       3
#define NVIC_IRQ_PRI4       4
#define NVIC_IRQ_PRI5       5
#define NVIC_IRQ_PRI6       6
#define NVIC_IRQ_PRI7       7
#define NVIC_IRQ_PRI8       8
#define NVIC_IRQ_PRI9       9
#define NVIC_IRQ_PRI10      10
#define NVIC_IRQ_PRI11      11
#define NVIC_IRQ_PRI12      12
#define NVIC_IRQ_PRI13      13
#define NVIC_IRQ_PRI14      14
#define NVIC_IRQ_PRI15      15
#define NVIC_IRQ_PRI16      16
#define NVIC_IRQ_PRI17      17
#define NVIC_IRQ_PRI18      18



#include "stm32f407xx_gpio_driver.h"


#endif /* INC_STM32F407XX_H_ */

