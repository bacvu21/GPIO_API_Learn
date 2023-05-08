/*
 * interrupt_test.c
 *
 *  Created on: Apr 17, 2023
 *      Author: Bacvu
 */


#include "stm32f407xx.h"
#include <string.h>


#define HIGH 		 ENABLE
#define BTN_PRESSED  HIGH


void delay (){

		for (uint32_t i = 0 ; i < 500000/2;i++);

}

int main (void){

		GPIO_Handle_t GpioLed, Gpiobtn;
		memset(&GpioLed,0,sizeof(GpioLed));
		memset(&Gpiobtn,0,sizeof(Gpiobtn));


		GpioLed.pGPIOx = GPIOA;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP; //PP mode
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



		GPIO_PeriClockControl(GPIOA,ENABLE);
		GPIO_Init(&GpioLed);


		Gpiobtn.pGPIOx = GPIOD;
		Gpiobtn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;
		Gpiobtn.GPIO_PinConfig.GPIO_PinMode  = GPIO_MODE_IT_FT; // interrupt mode falling edge
		Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		Gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;



		GPIO_PeriClockControl(GPIOA,ENABLE);
		GPIO_Init(&Gpiobtn);

		//IRQ conffigurations
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
		GPIO_IRQITConfig(IRQ_NO_EXTI9_5,ENABLE);
		while(1);
	return 0;
}
void EXTI9_5_IRQHandler (void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}
