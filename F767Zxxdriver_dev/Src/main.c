/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "F767ZIxx.h"
#include "F767Zxx_gpio_driver.h"
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay() {
	for (uint64_t i = 0; i < 500000; i++)
		;
}

int main(void) {

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Handle_t gpioLed;
	gpioLed.pGPIOx = GPIOB;
	gpioLed.GPIO_PinConfig.GPIO_Pinumber = GPIO_PIN_NO_7;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioLed);

	while (1) {
		GPIO_TogglePin(GPIOB, GPIO_PIN_NO_7);
		delay();
	}
}
