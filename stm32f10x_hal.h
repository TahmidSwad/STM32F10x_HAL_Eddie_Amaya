#ifndef stm32f10x_hal
#define stm32f10x_hal

#include "stm32f10x.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include <stdio.h>

typedef enum
{
	GPIO_MODE_INPUT_ANALOG    = 0b0000,
	GPIO_MODE_INPUT_FLOATING  = 0b0100,
	GPIO_MODE_INPUT_PUPD      = 0b1000,
	GPIO_MODE_OUTPUT_PP       = 0b0011,
	GPIO_MODE_OUTPUT_OD       = 0b0111,
	AF_MODE_OUTPUT_PP         = 0b1011,
	AF_MODE_OUTPUT_OD         = 0b1111
}mode_t;

typedef enum
{
  GPIO_PULLUP,
  GPIO_PULLDOWN
}pull_t;

typedef enum {
    GPIO_SPEED_LOW     = 0b1110,
    GPIO_SPEED_MEDIUM  = 0b1101,
    GPIO_SPEED_HIGH    = 0b1111
} speed_t;

typedef enum
{
  EDGE_RISING,
  EDGE_FALLING,
  EDGE_RISING_FALLING
} edge_t;

typedef struct
{
	uint8_t    pin;
	mode_t     mode;
	pull_t     pull;
	speed_t    speed;
	uint32_t   interrupt;
	edge_t     edge;
}GPIO_TypeDef_Init;


void GPIO_CLK_INIT(GPIO_TypeDef *port);
void AFIO_CLK_INIT();
void USART_CLK_INIT(USART_TypeDef *port);
void GPIO_INIT(GPIO_TypeDef *port, GPIO_TypeDef_Init *type);
void GPIO_WRITE(GPIO_TypeDef *port, uint8_t pin, uint8_t pinState);
void GPIO_TOGGLE(GPIO_TypeDef *port, uint8_t pin);
char GPIO_READ(GPIO_TypeDef *port, uint8_t pin);

void CLEAR_EXTI_PENDING(uint8_t pin);

void delay_timer_init();
void delayMs(uint32_t ms);

void SERIAL_BEGIN();
void printMsg(char *msg, ...);

void hal_enter_standby();
void hal_enter_stop();
void hal_enter_sleep();
void wakeup_handler();

#endif
