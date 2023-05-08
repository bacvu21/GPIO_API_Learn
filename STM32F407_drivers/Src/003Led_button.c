/*
 * 003Led_button.c
 *
 *  Created on: Apr 15, 2023
 *      Author: Bacvu
 */

#include "stm32f407xx.h"


#define HIGH 		 ENABLE
#define BTN_PRESSED  HIGH



void delay (){

		for (uint32_t i = 0 ; i < 500000/2;i++);

}
int click (GPIO_Handle_t Gpiobtn){

	Gpiobtn.pGPIOx = GPIOA;
	Gpiobtn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_0;
	Gpiobtn.GPIO_PinConfig.GPIO_PinMode  = GPIO_MODE_IN;
	Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&Gpiobtn);


	return (GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));


}

int main (void){



		GPIO_Handle_t GpioLed, GPIObtn;

		GpioLed.pGPIOx = GPIOA;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP; //PP mode
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



		GPIO_PeriClockControl(GPIOA,ENABLE);
		GPIO_Init(&GpioLed);


		click (GPIObtn);


		while (1){

			if (click(GPIObtn)== BTN_PRESSED){
				delay();
				GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
			}

		}



	return 0;
}
