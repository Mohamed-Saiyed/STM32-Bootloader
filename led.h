
#include "gpio_driver.h"


void led_init(GPIO_TypeDef *GPIOx , gpio_pin_confg * led);

void led_turn_on(GPIO_TypeDef *GPIOx , uint8_t led_num );

void led_turn_off(GPIO_TypeDef *GPIOx , uint8_t led_num );


void led_toggle(GPIO_TypeDef *GPIOx , uint8_t led_num );

void board_led_init(void);