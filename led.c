#include "led.h"



void led_init(GPIO_TypeDef *GPIOx , gpio_pin_confg * led)
{
  HAL_GPIO_INIT(GPIOx , led);
}


void board_led_init(void)
{
    gpio_pin_confg led ;
	
	_HAL_GPIOA_CLK_ENABLE();
	led.pin = 5 ;
	led.mode = GPIO_PIN_OUTPUT_MODE ;
	led.pull = GPIO_PIN_NO_PULLUP_NO_PULLDOWN;
	led.speed = GPIO_PIN_SPEED_MEDIUM;
	led.output_type = GPIO_PIN_OP_TYPE_PUSHPULL;
  HAL_GPIO_INIT(GPIO_PORT_A , &led);

}
void led_turn_on(GPIO_TypeDef *GPIOx , uint8_t led_num )
{
	HAL_GPIO_write_pin_value(GPIOx , led_num , 1) ;
}

void led_turn_off(GPIO_TypeDef *GPIOx , uint8_t led_num )
{
	HAL_GPIO_write_pin_value(GPIOx , led_num , 0) ;
}

void led_toggle(GPIO_TypeDef *GPIOx , uint8_t led_num )
{
  if(HAL_GPIO_read_pin_value(GPIOx , led_num))
	{
	  HAL_GPIO_write_pin_value(GPIOx , led_num , 0) ;
	}
	else
	{
		HAL_GPIO_write_pin_value(GPIO_PORT_A , led_num , 1) ;
	}

}