/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Apr 13, 2023
 *      Author: Bacvu
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"

typedef struct {
	uint8_t GPIO_PinNumber ;		  //possible values form @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;             //possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			  //possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;      //possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;           //possible values from @GPIO_PIN_OPTYPE
	uint8_t GPIO_PinAltFunMode;       //possible values from @GPIO_PIN_ALTMODE
}GPIO_PinConfig_t;



typedef struct {

	//pointer to hold the base address of the GPIO peripheral
	GPIO_regDef_t *pGPIOx;  // Hold the base address of the GPIO port to which pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; //this hold GPIO pin configuration setting

}GPIO_Handle_t;



/*@GPIO_PIN_NUMBERS
 *GPIO_pins
 */

#define GPIO_PIN_NO_0     0
#define GPIO_PIN_NO_1     1
#define GPIO_PIN_NO_2     2
#define GPIO_PIN_NO_3     3
#define GPIO_PIN_NO_4     4
#define GPIO_PIN_NO_5     5
#define GPIO_PIN_NO_6     6
#define GPIO_PIN_NO_7     7
#define GPIO_PIN_NO_8     8
#define GPIO_PIN_NO_9     9
#define GPIO_PIN_NO_10    10
#define GPIO_PIN_NO_11    11
#define GPIO_PIN_NO_12    12
#define GPIO_PIN_NO_13    13
#define GPIO_PIN_NO_14    14
#define GPIO_PIN_NO_15    15






/*@GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN     0	   //281/1747 -- value <3 non interrupt
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3

//interrupt mode
#define GPIO_MODE_IT_FT  4     //input falling edge
#define GPIO_MODE_IT_RT  5	   //input rising edge
#define GPIO_MODE_IT_RFT 6     //kich hoat che do falling edge va rising egde


/*@GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP    0   // push/pull
#define GPIO_OP_TYPE_OD    1	//open drain


/*@GPIO_PIN_SPEED
 *GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3

/*@GPIO_PIN_PUPD
 *GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD          0
#define GPIO_PIN_PU			  1
#define GPIO_PIN_PD 		  2


/***************************************************************************
 * 					APIs supported by this driver
 * 				Xem thong tin ve API thi kiem tra ham
 ***************************************************************************/

/*
 * Peripheral clock setup
 *
 */
void GPIO_PeriClockControl(GPIO_regDef_t *pGPIOx, uint8_t EnorDi);
/*
 * init and De-int
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_regDef_t *pGPIOx); // using this check resgister reset RCC

/*
 * data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber); //return 1 pin
uint16_t GPIO_ReadFromInputPort(GPIO_regDef_t *pGPIOx);		//return full port so that will take only port
void GPIO_WriteToOutputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber , uint8_t value);
void GPIO_WriteToOutputPort(GPIO_regDef_t *pGPIOx,uint16_t value);	//using for write to the port
void GPIO_ToggleOutputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber); // using to change Pin num


/*
 * interrupt config
 */
void GPIO_IRQConfig(uint8_t IRQNumber , uint8_t EnorDi); // num 1 : kind interrup , uu tien , cho phep hoac k
void GPIO_IRQHandling(uint8_t PinNumber); // xu li ngat qua pin num
void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint8_t IRQPriorty);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
