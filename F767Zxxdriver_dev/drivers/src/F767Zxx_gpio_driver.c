/*
 * F767Zxx_gpio_driver.c
 *
 *  Created on: Aug 16, 2024
 *      Author: WB07_24
 */

/**********************************************
 * @fn       				-
 *
 * @brief      				-
 * @param[0]  				-
 * @param[1}				-
 * @param[2]      			-
 *
 * @return 					-
 *
 * @NOte					-
 *
 */

#include "F767Zxx_gpio_driver.h"

/************************* API SUPPORTED BY THE DRIVER ****************************/
/*Peripheral Clock control*/
// will be used to enable or disable clock for the specific port of the given GPIO
/**********************************************
 * @fn       				- GPIO_PeriClockControl
 *
 * @brief      				- Enable or disable clock for the given GPIO port
 * @param[0]  				- Base address of the GPIO peripherals
 * @param[1}				- ENABLE OR DISABLE MACRO
 *
 * @return 					- None
 *
 * @NOte					- None
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}
		if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}
		if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
		if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
		if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_EN();
		}
		if (pGPIOx == GPIOK) {
			GPIOK_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}
		if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}
		if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
		if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
		if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_DI();
		}
		if (pGPIOx == GPIOK) {
			GPIOK_PCLK_DI();
		}
	}
}
/**********************************************
 * @fn       				- GPIO_Init
 *
 * @brief      				- Initialize the specific GPIO pins
 * @param[0]  				-
 * @param[1}				-
 * @param[2]      			-
 *
 * @return 					-
 *
 * @NOte					-
 *
 */


void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); //It just need the base address of the register

/*Data Read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritwToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*IRQ and ISR Handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPRIORITY, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t pinNumber);
