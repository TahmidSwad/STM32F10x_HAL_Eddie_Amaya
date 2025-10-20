#include "stm32f10x_hal.h"

uint8_t blink = 1;

int main()
{	
	delay_timer_init();
	GPIO_TypeDef_Init led = {0};
	
	led.mode  = GPIO_MODE_OUTPUT_PP;
	led.speed  = GPIO_SPEED_HIGH;
	
	led.pin = 13; GPIO_INIT(GPIOC, &led);
	led.pin = 13; GPIO_INIT(GPIOB, &led);
	led.pin = 12; GPIO_INIT(GPIOB, &led);
	
	GPIO_TypeDef_Init button = {0};
	button.mode = GPIO_MODE_INPUT_PUPD;
	button.pull = GPIO_PULLUP;
	button.interrupt = SET;
	button.edge = EDGE_FALLING;
	
	button.pin = 9; GPIO_INIT(GPIOB, &button);
	
	button.edge = EDGE_RISING_FALLING;
	button.pin = 8; GPIO_INIT(GPIOB, &button);
	
	SERIAL_BEGIN();
	
	
	while(1)
	{
		printMsg("SWAD\n");
		GPIO_WRITE(GPIOC, 13, SET);
		delayMs(100);

		GPIO_WRITE(GPIOC, 13, RESET);
		delayMs(100);
			
		if(blink == 0)
		{
			GPIO_TOGGLE(GPIOB, 12);
		}
		
	}
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI->PR & (1 << 9))
	{
		delayMs(5);
		if (GPIO_READ(GPIOB, 9) == 0) 
		{
			GPIO_TOGGLE(GPIOB, 13);
		}
			// Clear the interrupt pending bit
			CLEAR_EXTI_PENDING(9);
	}
	
	
	
	if (EXTI->PR & (1 << 8))
  {
		delayMs(5);
		uint8_t confirmed = GPIO_READ(GPIOB, 8);

		if (confirmed != blink)
		{
			blink = confirmed;

			if (blink)
			{
					GPIO_WRITE(GPIOB, 12, RESET);
			}
			else
			{
				GPIO_WRITE(GPIOB, 12, SET);
			}
		}
		CLEAR_EXTI_PENDING(8);
	}
		
}



