/*
 * F767ZIxx.h
 *
 *  Created on: Aug 12, 2024
 *      Author: WB07_24
 */

#ifndef INC_F767ZIXX_H_
#define INC_F767ZIXX_H_

#define __vo volatile

//All Memory Address
#define FLASH_BASEADDR   				0x08000000U //Here U is used at the end to denote that the address is unsigned number as by default compiler assumes it as the signed number
#define SRAM1_BASEADDR					0x20020000U
#define SRAM                            SRAM1_BASEADDR
#define SRAM2_BASEADDR 					0x2007C000U
#define ROM_BASEADDR					0x1FF00000U
#define OTP_BASEADDR 					0x1FF0F000U

//All Bus Address of Bus Domains
//APB BUSes
#define PERI_BASEADDR 	 	 	 		0x40000000U
#define APB1_BASEADDR 	 	 	 		PERI_BASEADDR
#define APB2_BASEADDR 					0x40010000U

//AHB BUSes
#define AHB1_BASEADDR 					0x40020000U
#define AHB2_BASEADDR					0x50000000
#define AHB3_BASEADDR 					0xA000 0000

//Base address of the peripheral hanging over AHB1 bus
//ALL GPIO BASE ADDRESS
#define GPIOA_BASEADDR					(AHB1_BASEADDR)
#define GPIOB_BASEADDR					(AHB1_BASEADDR + 0X0400)
#define GPIOC_BASEADDR					(AHB1_BASEADDR + 0X0800)
#define GPIOD_BASEADDR					(AHB1_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR					(AHB1_BASEADDR + 0X1000)
#define GPIOF_BASEADDR					(AHB1_BASEADDR + 0X1400)
#define GPIOG_BASEADDR					(AHB1_BASEADDR + 0X1800)
#define GPIOH_BASEADDR					(AHB1_BASEADDR + 0X1C00)
#define GPIOI_BASEADDR					(AHB1_BASEADDR + 0X2000)
#define GPIOJ_BASEADDR					(AHB1_BASEADDR + 0X2400)
#define GPIOK_BASEADDR					(AHB1_BASEADDR + 0X2800)

//RCC
#define RCC_BASEADDR					(AHB1_BASEADDR + 0X3800)

/*
 * @TODO
 * List out other peripherals hanging on this bus later
 * */

//BASE ADDRESS OF THE PERIPHERALS HANGING ON THE APB1 BUS
#define USART2_BASEADDR 				(APB1_BASEADDR + 0X4400)
#define USART3_BASEADDR 				(APB1_BASEADDR + 0X4800)
#define UART4_BASEADDR 				    (APB1_BASEADDR + 0X4C00)
#define UART5_BASEADDR 					(APB1_BASEADDR + 0X7C00)
#define UART7_BASEADDR 					(APB1_BASEADDR + 0X5000)
#define UART8_BASEADDR 					(APB1_BASEADDR + 0X5000)

#define I2C1_BASEADDR 					(APB1_BASEADDR + 0X5400)
#define I2C2_BASEADDR 					(APB1_BASEADDR + 0X5800)
#define I2C3_BASEADDR 					(APB1_BASEADDR + 0X5C00)
#define I2C4_BASEADDR 					(APB1_BASEADDR + 0X6000)

/*
 * @TODO
 * List out other peripherals hanging on this bus later
 * */
//BASE ADDRESS OF THE PERIPHERALS HANGING ON THE APB2 BUS
#define USART1_BASEADDR  				(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR  				(APB2_BASEADDR + 0x1400)

#define SPI1_BASEADDR 					(APB2_BASEADDR + 0X3000)
#define SPI4_BASEADDR 					(APB2_BASEADDR + 0X3400)

#define SYSCFG_BASEADDR 				(APB2_BASEADDR + 0X3800)
#define EXTI_BASEADDR 					(APB2_BASEADDR + 0X3C00)

#define SPI5_BASEADDR 					(APB2_BASEADDR + 0X5000)
#define SPI6_BASEADDR 					(APB2_BASEADDR + 0X5400)

/******************************* PERIPHERAL REGISTER DEFINITION STRUCTURE **************************************/
//FOR GPIO REGISTERS
typedef struct {

	__vo uint32_t MODER; 				 	 //GPIO port mode register
	__vo uint32_t OTYPER; 					 //GPIO port output type register
	__vo uint32_t OSPEEDR;				     //GPIO port output speed register
	__vo uint32_t PUPDR;				//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;						 //GPIO port input data register
	__vo uint32_t ODR;						 //GPIO port output data register
	__vo uint32_t BSRR;						 //GPIO port bit set/reset register
	__vo uint32_t LCKR; 				// GPIO port configuration lock register
	__vo uint32_t AFR[2]; // AFR[0] : GPIO alternate function low register , AFR[1] : GPIO alternate function high register

} GPIO_RegDef_t;

//Peripheral definition (peripheral base address type-casted to xxxRegDef_t)
#define GPIOA (GPIO_RegDef_t*)GPIOA_BASEADDR
#define GPIOB (GPIO_RegDef_t*)GPIOB_BASEADDR
#define GPIOC (GPIO_RegDef_t*)GPIOC_BASEADDR
#define GPIOD (GPIO_RegDef_t*)GPIOD_BASEADDR
#define GPIOE (GPIO_RegDef_t*)GPIOE_BASEADDR
#define GPIOF (GPIO_RegDef_t*)GPIOF_BASEADDR
#define GPIOG (GPIO_RegDef_t*)GPIOG_BASEADDR
#define GPIOH (GPIO_RegDef_t*)GPIOH_BASEADDR
#define GPIOI (GPIO_RegDef_t*)GPIOI_BASEADDR
#define GPIOJ (GPIO_RegDef_t*)GPIOJ_BASEADDR
#define GPIOK (GPIO_RegDef_t*)GPIOK_BASEADDR

#define RCC   (RCC_RegDef_t*)RCC_BASEADDR

//RCC register structure to control each register of it
typedef struct {
	__vo uint32_t CR; 					//RCC clock control register
	__vo uint32_t PLLCFGR; 				//RCC PLL configuration register
	__vo uint32_t CFGR;					//RCC clock configuration register
	__vo uint32_t CIR; 					//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;			    //RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR; 			//RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;				//RCC AHB3 peripheral reset register
	__vo uint32_t APB1RSTR;  			//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR; 			//RCC APB2 peripheral reset register
	__vo uint32_t RESERVED1;
	__vo uint32_t RESERVED2;
	__vo uint32_t AHB1ENR; 	//RCC AHB1 peripheral clock register (RCC_AHB1ENR)
	__vo uint32_t AHB2ENR; 			//RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR; 			//RCC AHB3 peripheral clock enable register
	__vo uint32_t RESERVED3;
	__vo uint32_t APB1ENR; 			//RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR; //RCC APB2 peripheral clock enable register (RCC_APB2ENR)
	__vo uint32_t RESERVED4;
	__vo uint32_t RESERVED5;
	__vo uint32_t AHB1LPENR; //RCC AHB1 peripheral clock enable in low-power mode register
	__vo uint32_t AHB2LPENR; //RCC AHB2 peripheral clock enable in low-power mode register
	__vo uint32_t AHB3LPENR; //RCC AHB3 peripheral clock enable in low-power mode register
	__vo uint32_t RESERVED6;
	__vo uint32_t APB1LPENR; //RCC APB1 peripheral clock enable in low-power mode register
	__vo uint32_t APB2LPENR; //RCC APB2 peripheral clock enabled in low-power mode register
	__vo uint32_t RESERVED7;
	__vo uint32_t RESERVED8;
	__vo uint32_t BDCR; 				//RCC backup domain control register
	__vo uint32_t CSR;				    //RCC clock control & status register
	__vo uint32_t RESERVED9;
	__vo uint32_t RESERVED10;
	__vo uint32_t SSCGR;		//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR; 			//RCC PLLI2S configuration register
	__vo uint32_t PLLSAICFGR;		    //RCC PLLSAI configuration register
	__vo uint32_t DCKCFGR1; 	//RCC dedicated clocks configuration register 1
	__vo uint32_t DCKCFGR2; 	// RCC dedicated clocks configuration register 2

} RCC_RegDef_t;

/*Clock Enable MACRO*/

/**************************** CLOCK ENABLE MACRO FOR PERIPHERALS *********************************/
/**************************** CLOCK ENABLE MACRO FOR GPIO PERIPHERALS *********************************/
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))  //GPIOA PERIPHERAL CLOCK
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN() (RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN() (RCC->AHB1ENR |= (1<<10))

/**************************** I2C CLK ENABLE MACRO FOR GPIO PERIPHERALS *********************************/
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1<<23))
#define I2C4_PCLK_EN() (RCC->APB1ENR |= (1<<24))

/**************************** I2C CLK ENABLE MACRO FOR GPIO PERIPHERALS *********************************/
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() (RCC->APB2ENR |= (1<<15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1<<20))
#define SPI6_PCLK_EN() (RCC->APB2ENR |= (1<<21))

/**************************** USART CLK ENABLE MACRO FOR GPIO PERIPHERALS *********************************/
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1<<5))
#define UART7_PCLK_EN() (RCC->APB1ENR |= (1<<30))
#define UART8_PCLK_EN() (RCC->APB1ENR |= (1<<31))

/*************************** SYS-CFG peripheral CLK ENABLE ****************************/
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1<<14))

/**************************** CLOCK DISABLE MACRO FOR PERIPHERALS *********************************/
/**************************** CLOCK DISABLE MACRO FOR GPIO PERIPHERALS *********************************/
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1<<0))  //GPIOA PERIPHERAL CLOCK
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DI() (RCC->AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DI() (RCC->AHB1ENR &= ~(1<<10))

/**************************** I2C CLK DISABLE MACRO FOR GPIO PERIPHERALS *********************************/
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1<<23))
#define I2C4_PCLK_DI() (RCC->APB1ENR &= ~(1<<24))

/**************************** I2C CLK DISABLE MACRO FOR GPIO PERIPHERALS *********************************/
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI() (RCC->APB2ENR &= ~(1<<15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI() (RCC->APB2ENR &= ~(1<<20))
#define SPI6_PCLK_DI() (RCC->APB2ENR &= ~(1<<21))

/**************************** USART CLK DISABLE MACRO FOR GPIO PERIPHERALS *********************************/
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1<<5))
#define UART7_PCLK_DI() (RCC->APB1ENR &= ~(1<<30))
#define UART8_PCLK_DI() (RCC->APB1ENR &= ~(1<<31))

/*************************** SYS-CFG peripheral CLK ENABLE ****************************/
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1<<14))

#endif /* INC_F767ZIXX_H_ */
