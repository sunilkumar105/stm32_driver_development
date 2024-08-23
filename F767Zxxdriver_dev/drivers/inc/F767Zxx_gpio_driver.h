/*
 * F767Zxx_gpio_driver.h
 *
 *  Created on: Aug 16, 2024
 *      Author: WB07_24
 */

#ifndef INC_F767ZXX_GPIO_DRIVER_H_
#define INC_F767ZXX_GPIO_DRIVER_H_

#include "F767ZIxx.h"

//@ALL_GPIO_PIN_NUMBERS
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

//@ALL_GPIO_OUTPUT_MODES
// GPIO POSSIBLE OUTPUT MODES
#define GPIO_MODE_IN	 0
#define GPIO_MODE_OUT 	 1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4 //Interrupt falling edge trigger
#define GPIO_MODE_IT_RT  5
#define GPIO_MODE_IT_RFT 6

//GPIO POSSIBLE OUTPUT TYPES
#define GPIO_OP_TYPE_PP 0//GPIO OUTPUT TYPE PUSH PULL
#define GPIO_OP_TYPE_OD 1//GPIO OUTPUT TYPE OPN DRAIN

//@GPIO_PIN_POSSIBLE_OUTPUT_SPEEDS
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

//@GPIO_PINS_POSSIBLE_PULLUP_PULLDOWN_CONFIGURATION
#define GPIO_PIN_NO_PUPD 0 //GPIO PIN NO PULL UP PULL DOWN
#define GPIO_PIN_PU  1
#define GPIO_PIN_PD  2

//GPIO PIN CONFIG STRUCTURE
typedef struct {
	uint8_t GPIO_Pinumber;
	uint8_t GPIO_PinMode;  //@ALL_GPIO_OUTPUT_MODES
	uint8_t GPIO_PinSpeed; //@GPIO_PIN_POSSIBLE_OUTPUT_SPEEDS
	uint8_t GPIO_PinPuPdControl; //@GPIO_PINS_POSSIBLE_PULLUP_PULLDOWN_CONFIGURATION
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

//GPIO HANDLE STRUCTURE
typedef struct {
	GPIO_RegDef_t *pGPIOx; //This holds the address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/************************* API SUPPORTED BY THE DRIVER ****************************/
/*Peripheral Clock control*/
// will be used to enable or disable clock for the specific port of the given GPIO
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI); //base address if the register as well as en or di bit

/*GPIO INIT AND DEINIT*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle); //contain base address of the register, as well as pin configuration
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); //It just need the base address of the register

/*Data Read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*IRQ and ISR Handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQPrioirityConfig(uint8_t IRQNumber,uint8_t IRQpriority);
void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_F767ZXX_GPIO_DRIVER_H_ */
