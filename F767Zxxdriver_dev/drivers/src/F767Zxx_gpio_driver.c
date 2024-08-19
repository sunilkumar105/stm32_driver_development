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
	uint32_t temp = 0; // temporary register

	//1. CONFIGURE MODE OF GPIO PIN

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		//Means the mode is non-interrupt mode
		//Mode will be actual pin-mode positioned at the location with two bit
		//we can left shift the temporary register by the mode by the pos = pinNumber x 2
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
		//clearing reg
		pGPIOHandle->pGPIOx->MODER &= ~(0b11
				<< 2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
		//writing reg
		pGPIOHandle->pGPIOx->MODER |= temp;
	} else {
		//Means the mode is interrupt, will code it later
	}

	temp = 0;
	//2. CONFIGURE THE SPEED
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
	//clearing reg
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0b11
			<< 2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
	//setting reg
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. CONFIGURE THE PUPD REGISTERS
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
	//clearing reg
	pGPIOHandle->pGPIOx->PUPDR &= ~(0b11
			<< 2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
	//setting reg
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. CONFIGURE THE OUTPUT TYPE
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber;
	//clearing reg
	pGPIOHandle->pGPIOx->OTYPER &= ~(0b1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
	//setting reg
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	//5. CONFIGURE THE ALTERNATE FUNCTIONALITY
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN) { //If the mode is alternate function, then only configure the alternate function

		uint8_t AFR_REG, POS;
		AFR_REG = pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber / 8;
		POS = pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber % 8;
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * POS);

		//clearing reg
		pGPIOHandle->pGPIOx->AFR[AFR_REG] &= ~(0b1111 << 4 * POS);
		//setting reg
		pGPIOHandle->pGPIOx->AFR[AFR_REG] |= temp;

	}

}
/**********************************************
 * @fn       				- GPIO_DeInit
 *
 * @brief      				- Deinit the GPIO REGISTER, USE RCC RESET REGISTER
 * @param[0]  				-
 * @param[1}				-
 * @param[2]      			-
 *
 * @return 					-
 *
 * @NOte					-
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	}
	if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}
	if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}
	if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}
	if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}
	if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	}
	if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	}
	if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
	if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
	if (pGPIOx == GPIOJ) {
		GPIOJ_REG_RESET();
	}
	if (pGPIOx == GPIOK) {
		GPIOK_REG_RESET();
	}
}

/*******************************Data Read and Write************************************/
/**********************************************
 * @fn       				- GPIO_ReadFromInputPin
 *
 * @brief      				- Read GPIO in state
 * @param[0]  				-
 * @param[1}				-
 * @param[2]      			-
 *
 * @return 					-
 *
 * @NOte					-
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritwToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*IRQ and ISR Handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPRIORITY, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t pinNumber);
