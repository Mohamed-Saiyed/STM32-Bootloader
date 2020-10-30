#include "gpio_driver.h"
static void set_Pxx(GPIO_TypeDef * GPIOx);
uint8_t Pxx = 0 ;

volatile uint32_t val = 0 ;
void HAL_GPIO_INIT(GPIO_TypeDef * GPIOx , gpio_pin_confg * config)
{
   HAL_SET_GPIO_CLK(GPIOx);
	GPIOx->MODER   |= ((config->mode) << (2*(config->pin)));

	GPIOx->OTYPER  |= ((config->output_type) << (config->pin));
   
	GPIOx->OSPEEDR |= ((config->speed) <<  (2*(config->pin)));
	
	HAL_GPIO_seet_alt_function(GPIOx , config->pin , config->alternate);
	
	GPIOx->PUPDR   |= ((config->pull) << (2*(config->pin)));

}
 
void 	HAL_GPIO_seet_alt_function(GPIO_TypeDef * GPIOx , uint32_t pin_no ,uint16_t alt_func_value)
{
    GPIOx->AFR[(uint32_t)pin_no >> 3UL] |= (alt_func_value << (((uint32_t)pin_no) & 0x07UL)*4);

}


uint8_t HAL_GPIO_read_pin_value(GPIO_TypeDef * GPIOx  , uint32_t pin_no )
{
   uint8_t Value;
	
   Value =  ((GPIOx->IDR  >> pin_no ) & 0x00000001);

	 return Value;
}
void HAL_GPIO_write_pin_value(GPIO_TypeDef * GPIOx  , uint32_t pin_no , uint8_t value)
{
 if(value)
  GPIOx->ODR |= (1 << pin_no);
 else
	GPIOx->ODR &=~ (1 << pin_no);

}

void HAL_GPIO_configure_interrupt(GPIO_TypeDef * GPIOx, uint16_t pin_no , INT_edge_select edge )
{
	set_Pxx(GPIOx);
	HAL_GPIO_INT_SYS_CONFIG(pin_no);
  if(edge == INT_RISING_EDGE)
	{
		EXTI->RTSR |= ( 1<< pin_no);
	}
  else if(edge == INT_FALLING_EDGE)
	{
		EXTI->FTSR |= ( 1<< pin_no);
	}
	else if(edge == INT_RISING_FALLING_EDGE)
	{
		EXTI->RTSR |= ( 1<< pin_no);
		EXTI->FTSR |= ( 1<< pin_no);
	}
	else
	{
		; //TODO
	}
    
}

void HAL_GPIO_enable_interrupt(uint16_t pin_no , IRQn_Type irq_no)
{
   EXTI->IMR |= (1<<pin_no);
	 NVIC_EnableIRQ(irq_no);
}

void HAL_GPIO_clear_interrupt_flag(uint16_t pin_no)
{
	if(EXTI->PR & (1<< pin_no))
	{
    EXTI->PR |= ( 1 << pin_no );
  }
}

void HAL_SET_GPIO_CLK(GPIO_TypeDef *GPIOx)
{
  if(GPIOx == GPIO_PORT_A)
	{
	  _HAL_GPIOA_CLK_ENABLE();
	}
	else if(GPIOx == GPIO_PORT_B)
	{
	 _HAL_GPIOB_CLK_ENABLE();
	}
	else if(GPIOx == GPIO_PORT_C)
	{
	 _HAL_GPIOC_CLK_ENABLE();
	}
	else if(GPIOx == GPIO_PORT_D)
	{
	 _HAL_GPIOD_CLK_ENABLE();
	}
	else if(GPIOx == GPIO_PORT_E)
	{
	 _HAL_GPIOE_CLK_ENABLE();
	}
	else if(GPIOx == GPIO_PORT_D)
	{
	 _HAL_GPIOD_CLK_ENABLE();
	}
	else if(GPIOx == GPIO_PORT_F)
	{
	 _HAL_GPIOF_CLK_ENABLE();
	}
	else if(GPIOx == GPIO_PORT_H)
	{
	 _HAL_GPIOH_CLK_ENABLE();
	}
	else
	{
	  ;
	}

}
void HAL_GPIO_INT_SYS_CONFIG(uint16_t pin_no)
{
	uint16_t pin;
	pin = (( pin_no % 4 ) + 3*( pin_no % 4 ));
	RCC_TypeDef *pRCC;
	pRCC = RCC;
	RCC->APB2ENR |=(1<<14);
	SYSCFG_TypeDef *pSYSCFG;
	pSYSCFG = SYSCFG;
 if (pin_no <=3)
 { 
   pSYSCFG->EXTICR[0] &= ~(0xF << pin); //Clearing 
	 pSYSCFG->EXTICR[0] |=  (Pxx << pin); //set
 }
 else if (pin_no >=4 && pin_no <=7)
 {
	 pin_no =( pin_no % 4);
  pSYSCFG->EXTICR[1] &= ~(0xF << pin); //Clearing 
	pSYSCFG->EXTICR[1] |=  (Pxx << pin); //set
 }
  else if (pin_no >=8 && pin_no <=11)
 {
  pSYSCFG->EXTICR[2] &= ~(0xF << pin); //Clearing 
	pSYSCFG->EXTICR[2] |=  (Pxx << pin); //set
 }
  else if (pin_no >=12 && pin_no <=15)
 {
  pSYSCFG->EXTICR[3] &= ~(0xF << pin); //Clearing 
	pSYSCFG->EXTICR[3] |=  (Pxx << pin); //set
 }
 else
 {
   ;
 }
}
static void set_Pxx(GPIO_TypeDef * GPIOx)
{
  if(GPIOx == GPIO_PORT_A)
	{
	  Pxx = PAx;
	}
	else if(GPIOx == GPIO_PORT_B)
	{
	 Pxx = PBx;
	}
	else if(GPIOx == GPIO_PORT_C)
	{
	 Pxx = PCx;
	}
	else if(GPIOx == GPIO_PORT_D)
	{
	 Pxx = PDx;
	}
	else if(GPIOx == GPIO_PORT_E)
	{
	 Pxx = PEx;
	}
	else if(GPIOx == GPIO_PORT_D)
	{
	 Pxx = PDx;
	}
	else if(GPIOx == GPIO_PORT_F)
	{
	 Pxx = PFx;
	}
	else if(GPIOx == GPIO_PORT_H)
	{
	 Pxx = PHx;
	}
	else
	{
	  
	}

}
void HAL_BORAD_BUTTON_IT(void)
{
  gpio_pin_confg board;
 	
  board.pin = 13;
	board.mode = GPIO_PIN_INPUT_MODE;
	board.output_type = GPIO_PIN_NO_OP_TYPE;
	board.pull = GPIO_PIN_PULL_UP;
	board.speed = GPIO_PIN_SPEED_LOW;
	board.alternate = GPIO_PIN_AF0_SYSTEM;
	HAL_GPIO_INIT(GPIO_PORT_C , &board);	
	
	
	HAL_GPIO_configure_interrupt(GPIO_PORT_C , 13 , INT_FALLING_EDGE);
	HAL_GPIO_enable_interrupt(13 , EXTI15_10_IRQn);

}

void HAL_BORAD_BUTTON(void)
{
  gpio_pin_confg board;
 	
  board.pin = 13;
	board.mode = GPIO_PIN_INPUT_MODE;
	board.output_type = GPIO_PIN_NO_OP_TYPE;
	board.pull = GPIO_PIN_PULL_UP;
	board.speed = GPIO_PIN_SPEED_LOW;
	board.alternate = GPIO_PIN_AF0_SYSTEM;
	HAL_GPIO_INIT(GPIO_PORT_C , &board);	
	
}
void HAL_GPIO_CONFIGURE_EVENT(GPIO_TypeDef * GPIOx, uint16_t pin_no , INT_edge_select edge )
{
	set_Pxx(GPIOx);
	HAL_GPIO_INT_SYS_CONFIG(pin_no);
  if(edge == INT_RISING_EDGE)
	{
		EXTI->RTSR |= ( 1<< pin_no);
	}
  else if(edge == INT_FALLING_EDGE)
	{
		EXTI->FTSR |= ( 1<< pin_no);
	}
	else if(edge == INT_RISING_FALLING_EDGE)
	{
		EXTI->RTSR |= ( 1<< pin_no);
		EXTI->FTSR |= ( 1<< pin_no);
	}
	else
	{
		; //TODO
	}
    
}
void HAL_GPIO_EVENT_ENABLE(uint16_t pin_no)
{
   EXTI->EMR |= (1<<pin_no);

}

void HAL_BORAD_BUTTON_EV(void)
{
	
  gpio_pin_confg board;
 	
  board.pin = 13;
	board.mode = GPIO_PIN_INPUT_MODE;
	board.output_type = GPIO_PIN_NO_OP_TYPE;
	board.pull = GPIO_PIN_PULL_UP;
	board.speed = GPIO_PIN_SPEED_LOW;
	board.alternate = GPIO_PIN_AF0_SYSTEM;
	HAL_GPIO_INIT(GPIO_PORT_C , &board);	
	
	
	HAL_GPIO_CONFIGURE_EVENT(GPIO_PORT_C , 13 , INT_FALLING_EDGE);
	HAL_GPIO_EVENT_ENABLE(13);

}
