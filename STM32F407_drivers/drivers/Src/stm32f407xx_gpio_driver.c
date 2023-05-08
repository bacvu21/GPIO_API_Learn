/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 13, 2023
 *      Author: Bacvu
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral clock setup
 *
 */
/*******************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 *@brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 *@param[in]		- base address of the gpio peripheral
 *@param[in]		- Enable or Disable macros
 *@param[in]		-
 *
 *@return			- none
 *
 *@Note				- none
 */

void GPIO_PeriClockControl(GPIO_regDef_t *pGPIOx, uint8_t EnorDi)
{

	if (EnorDi == ENABLE){
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLOCK_EN();
		}
		else if (pGPIOx ==GPIOB){
			GPIOB_PCLOCK_EN();
		}
		else if (pGPIOx ==GPIOC){
				GPIOC_PCLOCK_EN();
			}
		else if (pGPIOx ==GPIOD){
				GPIOD_PCLOCK_EN();
			}
		else if (pGPIOx ==GPIOE){
				GPIOE_PCLOCK_EN();
			}
		else if (pGPIOx ==GPIOF){
				GPIOF_PCLOCK_EN();
			}
		else if (pGPIOx ==GPIOG){
				GPIOG_PCLOCK_EN();
			}
		else if (pGPIOx ==GPIOH){
				GPIOH_PCLOCK_EN();
			}
		else if (pGPIOx ==GPIOI){
				GPIOI_PCLOCK_EN();
			}
	}
	else{
		//DISABLE
				if (pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
				else if (pGPIOx ==GPIOB){
					GPIOB_PCLK_DI();
				}
				else if (pGPIOx ==GPIOC){
					GPIOC_PCLK_DI();
					}
				else if (pGPIOx ==GPIOD){
					GPIOD_PCLK_DI();
					}
				else if (pGPIOx ==GPIOE){
					GPIOE_PCLK_DI();
					}
				else if (pGPIOx ==GPIOF){
					GPIOF_PCLK_DI();
					}
				else if (pGPIOx ==GPIOG){
					GPIOG_PCLK_DI();
					}
				else if (pGPIOx ==GPIOH){
					GPIOH_PCLK_DI();
					}
				else if (pGPIOx ==GPIOI){
					GPIOI_PCLK_DI();
					}
	}




}
/*
 * init and De-int
 */
/*******************************************************************
 * @fn				- GPIO_Init
 *
 *@brief			- Initial pin number
 *
 *@param[in]		- base address of the gpio peripheral
 *@param[in]		-
 *@param[in]		-
 *
 *@return			- none
 *
 *@Note				- use base address to inital pin
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	//1.configure the mode of gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//khong phai interrupt
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//moi thanh ghi nhan 2 truong bit
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear bit
		pGPIOHandle->pGPIOx->MODER |= temp; //khi de bang pin update se bat cac bit khac cung thanh ghi len
		//vi the dung toan tu or
	}
	else {
		//interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			//1 .configure the FTSR
			EXTI->FTSR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1 .configure the RTSR
			EXTI->RTSR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding TSR bit
			EXTI->FTSR &= ~(1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1.configure both FTSR and RTSR

			EXTI->FTSR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2.configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;//find the register which contain pin want to
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); //pass Port to register after check
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode <<(temp2 *4); //turn on bit location

		//3.enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp =0;

	//2.configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;   // thanh ghi vat li

	temp =0;
	//3 configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |=temp;

	temp = 0;
	//4.configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=temp;


	//5.configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint8_t temp1 = 0;
		uint8_t temp2 = 0;

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFR[temp1] &=~(0xF << (4 *temp2));

		pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 *temp2));




	}


}
/*******************************************************************
 * @fn				- GPIO_DeInit
 *
 *@brief			- Huy bo khoi tao cac chan pin reset
 *
 *@param[in]		- base address of the gpio peripheral
 *@param[in]		-
 *@param[in]		-
 *
 *@return			- none
 *
 *@Note				- using this , check resgister reset pin mode and set port
 */
void GPIO_DeInit(GPIO_regDef_t *pGPIOx){

			if (pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if (pGPIOx ==GPIOB){
				GPIOB_REG_RESET();
			}
			else if (pGPIOx ==GPIOC){
				GPIOC_REG_RESET();
				}
			else if (pGPIOx ==GPIOD){
				GPIOD_REG_RESET();
				}
			else if (pGPIOx ==GPIOE){
				GPIOE_REG_RESET();
				}
			else if (pGPIOx ==GPIOF){
				GPIOF_REG_RESET();
				}
			else if (pGPIOx ==GPIOG){
				GPIOG_REG_RESET();
				}
			else if (pGPIOx ==GPIOH){
				GPIOH_REG_RESET();
				}
			else if (pGPIOx ==GPIOI){
				GPIOI_REG_RESET();
				}

}

/*
 * data read and write
 */
/*******************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 *@brief			- This function use to read from 1  pin which input mode
 *
 *@param[in]		- base address of the gpio peripheral
 *@param[in]		- address is will contain pin number value
 *@param[in]		-
 *
 *@return			- 0 or 1
 *
 *@Note				- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber)
{

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >>PinNumber) & 0x00000001); //lấy vị trí của bit trong thanh ghi IDR , chạm tới vị trí bit ít nhất

	return value;


}

/*******************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 *@brief			- This function will read 16 bit from port
 *
 *@param[in]		- base address of the gpio peripheral
 *@param[in]		-
 *@param[in]		-
 *
 *@return			- full pin of port which (uint16_t) type
 *
 *@Note				- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_regDef_t *pGPIOx){

	uint16_t value;
		value = (uint16_t)pGPIOx->IDR;

		return value;


}	//return full port so that will take only port
/*******************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 *@brief			- This function will write to Output pin
 *
 *@param[in]		- base address of the gpio peripheral
 *@param[in]		- Pin number
 *@param[in]		- value return
 *
 *@return			- none
 *
 *@Note				- none
 */

void GPIO_WriteToOutputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber , uint8_t value){


	if (value == GPIO_PIN_SET){
	//write 1
		pGPIOx -> ODR |= (1 << PinNumber);

	}else
	{
		//write 0
		pGPIOx ->ODR &= ~(1<< PinNumber);
	}






}
/*******************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 *@brief			- This function will write to OutPut port
 *
 *@param[in]		- base address of the gpio peripheral
 *@param[in]		- Value will return
 *@param[in]		-
 *
 *@return			- none
 *
 *@Note				- none
 */
void GPIO_WriteToOutputPort(GPIO_regDef_t *pGPIOx,uint16_t value){

	pGPIOx ->ODR = value; //write into register

}	//using for write to the port
/*******************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 *@brief			- This function will change the pin output
 *
 *@param[in]		- base address of the gpio peripheral
 *@param[in]		-Pin Number wanna change
 *@param[in]		-
 *
 *@return			- none
 *
 *@Note				- none
 */
void GPIO_ToggleOutputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber)
{
	// chuyen doi gia tri dung XOR operator
	pGPIOx->ODR ^= (1<< PinNumber);

}


/*
 * interrupt config
 */
/*******************************************************************
 * @fn				- GPIO_IRQConfig
 *
 *@brief			- This function configure for Interrupt register(NVIC setting)
 *
 *@param[in]		- Interrupt number
 *@param[in]		- Priority interrupt
 *@param[in]		- Enable or Disable macros
 *
 *@return			- none
 *
 *@Note				- none
 */
void GPIO_IRQITConfig(uint8_t IRQNumber , uint8_t EnorDi){

	if (EnorDi == ENABLE){
		if (IRQNumber <=31){
			//modify ISER0 register
			*NVIC_ISER0 |= (1<< IRQNumber);
		}
		else if (IRQNumber >31 && IRQNumber <64){
			//modify ISER1 register
			*NVIC_ISER1 |= (1<< (IRQNumber%32));
		}
		else if (IRQNumber >=64 && IRQNumber < 96){

			//modify ISER2 register
			*NVIC_ISER2 |= (1<< (IRQNumber%64));

		}
	}
	else {
		if (IRQNumber <=31){
			//modify ICER0 register
			*NVIC_ICER0 |= (1<< IRQNumber);
		}
		else if (IRQNumber >31 && IRQNumber <64){
			//modify ICER1 register
			*NVIC_ICER1 |= (1<< (IRQNumber%32));

		}
		else if (IRQNumber >=64 && IRQNumber < 96){

			//modify ICER2 register
			*NVIC_ICER2 |= (1<< (IRQNumber%64));
		}

	}
}
/*******************************************************************
 * @fn				- GPIO_IRQConfig
 *
 *@brief			- This function configure for Interrupt register(NVIC setting priority)
 *
 *@param[in]		- IRQPriory which pass number
 *@param[in]		-
 *@param[in]		-
 *
 *@return			- none
 *
 *@Note				- none
 */
//59-IPRegister - 4section - 8bit/1 sec(0-239)
void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint8_t IRQPriorty){

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber %4;

	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED); // choses the bits you want to implemented
		// not sp Low register
	*(NVIC_PR_BASE_ADDR + (iprx*4))|= IRQPriorty << shift_amount;

}


 // num 1 : kind interrup , uu tien , cho phep hoac k
/*******************************************************************
 * @fn				- GPIO_IRQHandling
 *
 *@brief			- This function handle interrupt
 *
 *@param[in]		- number of pin
 *@param[in]		-
 *@param[in]		-
 *
 *@return			- none
 *
 *@Note				- none
 */
void GPIO_IRQHandling(uint8_t PinNumber){

	//clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber)){

		//clear
		EXTI->PR |= (1 << PinNumber);
	}



} // xu li ngat qua pin num
