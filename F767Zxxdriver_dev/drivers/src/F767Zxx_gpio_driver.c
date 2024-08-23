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
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		} else if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_EN();
		} else if (pGPIOx == GPIOK) {
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

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT) { //Mode is interrupt for falling edge
			// 1. Configure the FTSR (Falling edge trigger selection register)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
			//If we only want Falling edge trigger, then we will have to make sure that the RTSR for rising edge is disabled
			// it may be on because of previous configuration or for something else reason, so we will have to Clear it
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
		}
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT) { //Mode is interrupt for rising edge
			// 1. Configure the RTSR (Rising edge trigger selection register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
			//clear the corresponding FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
		}
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT) { //Mode is interrupt for falling edge
			// 1. Configure both FTSR (Falling edge trigger selection register) and RTSR (Rising edge trigger selection register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. Enable the exti interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinumber);
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
 * @return 					- 0 or 1
 *
 * @NOte					-
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	uint8_t value = 0;
	value = (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x00000001);
	return value;
}
/**********************************************
 * @fn       				- GPIO_ReadFromInputPort
 *
 * @brief      				- Read complete GPIO PORT state
 * @param[0]  				-
 * @param[1}				-
 * @param[2]      			-
 *
 * @return 					- uint16_t IDR STATUS OF TEH PORT
 *
 * @NOte					-
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value = 0;

	//It should be implemented like this, as the register is 32 bit, but MSB 16 bits are fixed.
	//value = (uint16_t) ((pGPIOx->IDR >> 16) & 0x0000FFFF);

	//according to the course, he is reading the value into 16 bit register from 32 bit and returning it as it is
	value = (uint16_t) (pGPIOx->IDR);

	return value;
}
/**********************************************
 * @fn       				- GPIO_WriteToOutputPin
 *
 * @brief      				- WRITE TO A SPECIFIC PIN OF THE GPIO
 * @param[0]  				- GPIO_RegDef_t*GPIO PORT BASE ADDRESS
 * @param[1}				- uint8_t PIN NUMBER
 * @param[2]      			- uint8_t VALUE
 *
 * @return 					-
 *
 * @NOte					-
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value) {
	if (value == GPIO_PIN_SET) {
		//WRITE 1 AT THE ODR REGISTER AT THE BIT FIELD CORRESPONDING TO THE PINN NUMBER
		pGPIOx->ODR |= (0b1 << pinNumber);
	} else {
		pGPIOx->ODR &= ~(0b1 << pinNumber);
	}
}
/**********************************************
 * @fn       				- GPIO_WriteToOutputPort
 *
 * @brief      				- WRITE TO A SPECIFIC GPIO PORT
 * @param[0]  				- GPIO_RegDef_t*GPIO PORT BASE ADDRESS
 * @param[1}				- uint8_t VALUE
 * @param[2]      			-
 *
 * @return 					-
 *
 * @NOte					-
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}
/**********************************************
 * @fn       				- GPIO_TogglePin
 *
 * @brief      				- TOGGLE THE PIN
 * @param[0]  				- GPIO_RegDef_t*GPIO PORT BASE ADDRESS
 * @param[1}				- uint8_t pinNumber
 * @param[2]      			-
 *
 * @return 					-
 *
 * @NOte					-
 *
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR ^= (1 << pinNumber);
}

/*************************IRQ and ISR Handling***********************************/
/**********************************************
 * @fn       				- GPIO_IRQConfig
 *
 * @brief      				- Enable or disable specific IRQ number & Its priority
 * @param[0]  				- uint8_t IRQNumber
 * @param[1}				- uint8_t IRQPRIORITY
 * @param[2]      			- uint8_t ENorDI
 *
 * @return 					-
 *
 * @NOte					-
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPRIORITY, uint8_t ENorDI) {

}

void GPIO_IRQHandling(uint8_t pinNumber);
