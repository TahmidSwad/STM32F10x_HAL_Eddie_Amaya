#include "stm32f10x_hal.h"


/**
 * @brief  Enables the clock for a given GPIO port.
 * @param  port: Pointer to the GPIO port (GPIOA, GPIOB, GPIOC, etc.)
 * @retval None
 * @note   The function checks which port is passed and enables its clock 
 *         using the RCC (Reset and Clock Control) peripheral.
 */
void GPIO_CLK_INIT(GPIO_TypeDef *port)
{
	if(port == GPIOA)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	}
	else if(port == GPIOB)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	}
	else if(port == GPIOC)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	}
}


/**
 * @brief  Enables the clock for the AFIO (Alternate Function I/O) peripheral.
 * @param  None
 * @retval None
 * @note   The AFIO peripheral is needed for configuring external interrupts, 
 *         pin remapping, and event routing. This function enables its clock 
 *         via the RCC (Reset and Clock Control) peripheral.
*/
void AFIO_CLK_INIT()
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}

void USART_CLK_INIT(USART_TypeDef *port)
{
	if(port == USART1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	}
	/*else if(port == USART2)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	}
	else if(port == USART3)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	}
	else if(port == USART4)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	}
	else if(port == USART5)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	}*/
}

/**
 * @brief  Initializes a GPIO pin with the specified mode, speed, pull-up/pull-down, 
 *         and interrupt configuration.
 * @param  port: Pointer to the GPIO port (e.g., GPIOA, GPIOB, GPIOC).
 * @param  type: Pointer to a GPIO_TypeDef_Init structure containing the pin configuration:
 *               - pin: Pin number (0-15)
 *               - mode: Input, Output, Analog, or Alternate Function mode
 *               - speed: Output speed (if applicable)
 *               - pull: Pull-up, Pull-down, or No pull (for input modes)
 *               - interrupt: ENABLE/DISABLE interrupt configuration
 *               - edge: Rising, Falling, or Rising/Falling edge trigger
 * @retval None
 * @note   This function performs the following:
 *         1. Enables the GPIO port clock.
 *         2. Configures the pin mode and output speed.
 *         3. Configures pull-up or pull-down resistors if required.
 *         4. Configures the EXTI (external interrupt) if enabled, including
 *            AFIO remapping and NVIC interrupt enabling.
 *         5. Updates the appropriate GPIO CRL or CRH register for the pin.
 */
void GPIO_INIT(GPIO_TypeDef *port, GPIO_TypeDef_Init *type)
{
	GPIO_CLK_INIT(port);
	
	volatile uint32_t config = 0b0000;		// CNF+MODE configuration bits.
	switch(type->mode)
	{
		// ===== INPUT MODES =====
		// mode bits (last two) are defined 0, and will stay 0
		case GPIO_MODE_INPUT_ANALOG:
		{
			config = GPIO_MODE_INPUT_ANALOG;
			break;
		}
		case GPIO_MODE_INPUT_FLOATING:
		{
			config = GPIO_MODE_INPUT_FLOATING;
			break;
		}
		case GPIO_MODE_INPUT_PUPD:
		{
			config = GPIO_MODE_INPUT_PUPD;
			if (type->pull == GPIO_PULLUP) 
			{
				port->BSRR = (1 << type->pin);		// set high
			} 
			else if (type->pull == GPIO_PULLDOWN)
			{
				port->BSRR = (1 << (type->pin + 16));		// set low
			}
			break;
		}
		// ===== OUTPUT / AF MODES =====
		// mode bits (last two) are defined 1, but will change according to speed
		// that's why AND operation is used
		case GPIO_MODE_OUTPUT_PP:
		{
			config = GPIO_MODE_OUTPUT_PP;
			config &= type->speed;
			break;
		}
		case GPIO_MODE_OUTPUT_OD:
		{
			config = GPIO_MODE_OUTPUT_OD;
			config &= type->speed;
			break;
		}
		case AF_MODE_OUTPUT_PP:
		{
			config = AF_MODE_OUTPUT_PP;
			config &= type->speed;
			break;
		}
		case AF_MODE_OUTPUT_OD:
		{
			config = AF_MODE_OUTPUT_OD;
			config &= type->speed;
			break;
		}
	}
	// ===== CONFIGURE CRL/CRH REGISTER =====
	if(type->pin < 8)
	{
		port->CRL &= ~(0b1111 << (type->pin * 4));
		port->CRL |= (config << (type->pin * 4));
	}
	else if (type->pin < 16)
	{
		port->CRH &= ~(0b1111 << ((type->pin - 8) * 4));
		port->CRH |= (config << ((type->pin - 8) * 4));
	}
	
	// ===== CONFIGURE EXTI (if enabled) =====
	if(type->interrupt == SET)
	{
		AFIO_CLK_INIT();
		
		// Map pin to EXTI line via AFIO_EXTICR, (4 registers in an array)
		uint8_t extiIndex = type->pin / 4;
		uint8_t extiShift = (type->pin % 4) * 4;
		
		AFIO->EXTICR[extiIndex] &= ~(0b1111 << extiShift);
		if(port == GPIOA) 
		{
			AFIO->EXTICR[extiIndex] |= (0b0000 << extiShift);
		}
		else if(port == GPIOB) 
		{
			AFIO->EXTICR[extiIndex] |= (0b0001 << extiShift);
		}
		else if(port == GPIOC) 
		{
			AFIO->EXTICR[extiIndex] |= (0b0010 << extiShift);
		}
		
		// Enable interrupt line
		EXTI->IMR |= 1 << type->pin;
		
		// Configure edge trigger
		if (type->edge == EDGE_RISING) 
		{			
			EXTI->RTSR |= (1 << type->pin); 
			EXTI->FTSR &= ~(1 << type->pin);
    } 
		else if (type->edge == EDGE_FALLING)
		{
      EXTI->FTSR |= (1 << type->pin); 
      EXTI->RTSR &= ~(1 << type->pin);
    } 
		else if(type->edge == EDGE_RISING_FALLING)
		{
      EXTI->RTSR |= (1 << type->pin);
      EXTI->FTSR |= (1 << type->pin);
    }
		
		// Enable NVIC interrupt (EXTI lines are grouped)
		if (type->pin <= 4) 
		{
       NVIC_EnableIRQ(EXTI0_IRQn + type->pin);
    } 
		else if (type->pin <= 9) 
	  {
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    } 
		else if (type->pin <= 15) 
		{
        NVIC_EnableIRQ(EXTI15_10_IRQn);
    }

		// Reinforce pull-up/down state
		if (type->pull == GPIO_PULLUP) 
		{
			port->BSRR = (1 << type->pin);		// Set pin high
		} 
		else if (type->pull == GPIO_PULLDOWN)
		{
			port->BSRR = (1 << (type->pin + 16));  // Set pin low
		}
	}
}


/**
 * @brief  Clears the pending bit for a given EXTI (External Interrupt) line.
 * @param  pin: EXTI line number corresponding to the GPIO pin (0-15)
 * @retval None
 * @note   Writing a '1' to the pending register (PR) clears the pending interrupt
 *         for that EXTI line.
 */
void CLEAR_EXTI_PENDING(uint8_t pin)
{
	EXTI->PR = (1 << pin);
}


/**
 * @brief  Sets or resets the state of a specific GPIO pin.
 * @param  port: Pointer to the GPIO port (GPIOA, GPIOB, GPIOC, etc.)
 * @param  pin: Pin number to write (0-15)
 * @param  pinState: Desired state of the pin (SET or RESET)
 * @retval None
 * @note   This function uses the GPIO port's BSRR register to atomically
 *         set or reset the output pin.
 */
void GPIO_WRITE(GPIO_TypeDef *port, uint8_t pin, uint8_t pinState)
{
	if(pinState == SET)
	{
		port->BSRR = 1 << pin;
	}
	else
	{
		port->BSRR = 1 << (pin + 16);
	}
}


/**
 * @brief  Toggles the output state of a specific GPIO pin.
 * @param  port: Pointer to the GPIO port (e.g., GPIOA, GPIOB, GPIOC)
 * @param  pin: Pin number to toggle (0-15)
 * @retval None
 * @note   Uses the ODR (Output Data Register) to invert the current pin state.
 */
void GPIO_TOGGLE(GPIO_TypeDef *port, uint8_t pin)
{
	port->ODR ^= 1 << pin;
}


/**
 * @brief  Reads the input state of a specific GPIO pin.
 * @param  port: Pointer to the GPIO port (e.g., GPIOA, GPIOB, GPIOC)
 * @param  pin: Pin number to read (0-15)
 * @retval uint32_t: Returns 1 if the pin is HIGH, 0 if LOW
 * @note   Uses the IDR (Input Data Register) to get the current pin state.
 */
char GPIO_READ(GPIO_TypeDef *port, uint8_t pin)
{
	return (port->IDR & (1 << pin)) ? 1 : 0;
}


/**
 * @brief  Initializes USART1 for basic serial communication.
 * @note   Configures the necessary GPIO pins, clocks, and USART settings.
 *         This function assumes PA9 as TX and uses default settings for 9600 baud.
 * @retval None
 */
void SERIAL_BEGIN()
{
	GPIO_CLK_INIT(GPIOA);
	AFIO_CLK_INIT();
	USART_CLK_INIT(USART1);
	GPIOA->CRH &= ~(0b1111 << 4);
	GPIOA-> CRH |= (0b1011 << 4);
	
	USART1->BRR = 0x1D4C;
	USART1->CR1 |= USART_CR1_TE;
	USART1->CR1 |= USART_CR1_UE;
}

/**
 * @brief  Sends a formatted string over USART1.
 * @param  msg: Pointer to a null-terminated format string (like printf).
 * @note   Supports variable arguments via `...`. Uses a local buffer of 80 bytes.
 * @retval None
 */
void printMsg(char *msg, ...)
{
	char buff[80];
	
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);
	va_end(args);
	for(unsigned int i = 0; i < strlen(buff); i++)
	{
		USART1->DR = buff[i];
		while(!( USART1->SR & USART_SR_TXE));
	}
	while (!(USART1->SR & USART_SR_TC));
}


void delay_timer_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 7199;
	TIM2->ARR = 9;
	TIM2->CNT = 0;
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->SR &= ~TIM_SR_UIF;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void delayMs(uint32_t ms)
{
	for(uint32_t i = 0; i < ms; i++)
	{
		TIM2->CNT = 0;
		TIM2->SR &= ~TIM_SR_UIF;
		
		while(!(TIM2->SR & TIM_SR_UIF));
	}
}


void hal_enter_standby()
{
   // Enable PWR peripheral
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Clear any previous wakeup and standby flags
    PWR->CR |= PWR_CR_CWUF;   // Clear wakeup flag
    PWR->CR |= PWR_CR_CSBF;   // Clear standby flag

    // Configure standby mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    PWR->CR |= PWR_CR_PDDS;   // Enter standby mode when WFI executed
    PWR->CSR |= PWR_CSR_EWUP; // Enable WKUP pin

    __WFI();                   // Enter standby
}

void hal_enter_stop()
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  // Select Stop mode: set SLEEPDEEP
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  // Set PDDS = 0 (Stop mode), LPDS = 1 (low-power regulator)
  PWR->CR &= ~PWR_CR_PDDS;  // ensure PDDS=0 for Stop mode
  PWR->CR |= PWR_CR_LPDS;   // low power regulator

  // Clear Wakeup flag if needed
  PWR->CR |= PWR_CR_CWUF;

  __WFI();  // Enter Stop mode	
}

void hal_enter_sleep()
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  // Clear SLEEPDEEP bit ? select Sleep mode
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	
  __WFI(); // Wait For Interrupt
}

void hal_enter_sleeponexit()
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// Clear SLEEPDEEP bit ? select Sleep mode
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

  SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // auto sleep on ISR exit
  __WFI(); // Wait For Interrupt
}

void wakeup_handler()
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	if((PWR->CSR) & (PWR_CSR_SBF))
	{
		PWR->CR |= PWR_CR_CWUF;
		PWR->CR |= PWR_CR_CSBF;
		
		printMsg("Awaken from standby\n");
	}
	else{
		PWR->CR |= PWR_CR_CWUF;
		PWR->CR |= PWR_CR_CSBF;
		printMsg("Awaken from power cycle\n");
	}
}
