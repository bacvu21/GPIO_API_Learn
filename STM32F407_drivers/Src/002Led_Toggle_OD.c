/*
 * 002Led_Toggle_OD.c
 *
 *  Created on: Apr 15, 2023
 *      Author: Bacvu
 */

#include "stm32f407xx.h"



void delay (void)
{
	for (uint32_t i = 0 ; i < 50000;i++);
}
int main (void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD; //Open drain mode
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; //PUll up resgiter


	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);


	while (1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();

	}



	return 0;

}
