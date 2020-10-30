#ifndef GOIP_DRIVER_H_
#define GOIP_DRIVER_H_


#include "stm32f446xx.h"
#include "stdint.h"

#define PAx    0
#define PBx    1
#define PCx    2
#define PDx    3
#define PEx    4
#define PFx    5
#define PGx    6
#define PHx    7
/*GPIO Alternative fun value*/
#define GPIO_PIN_AF0_SYSTEM                                 0
#define GPIO_PIN_AF1_TIM1_2                                 1
#define GPIO_PIN_AF2_TIM3__5                                2
#define GPIO_PIN_AF3_TIM_8__11_CEC                          3
#define GPIO_PIN_AF4_I2C_1__4_CEC                           4
#define GPIO_PIN_AF5_SPI_1__4                               5
#define GPIO_PIN_AF6_SPI_2__4_SAI_1                         6
#define GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN      7
#define GPIO_PIN_AF8_SPI_2_3_USART_1__3_UART5_SPDIF_IN      8
#define GPIO_PIN_AF9_CAN_1_2_TIM_12__14_QSPI                9
#define GPIO_PIN_AF10_SAI2_QSPI_OTG_HS_OTG_FS               10
#define GPIO_PIN_AF11                                       11
#define GPIO_PIN_AF12_FMC_SDIO_OTG_HS                       12
#define GPIO_PIN_AF13_DCMI                                  13
#define GPIO_PIN_AF14                                       14
#define GPIO_PIN_AF15_EVENTOUT                              15

/*GPIO Mode value*/
#define GPIO_PIN_INPUT_MODE      (0)
#define GPIO_PIN_OUTPUT_MODE     (1)
#define GPIO_PIN_ALT_FUN_MODE    (2)
#define GPIO_PIN_ANALOG_MODE     (3)



/*GPIO port output type mode*/

#define GPIO_PIN_NO_OP_TYPE            (0)
#define GPIO_PIN_OP_TYPE_PUSHPULL      (0)
#define GPIO_PIN_OP_TYPE_OPEN_DRAIN    (1)


/*GPIO port output speed selection*/

#define GPIO_PIN_SPEED_LOW         (0)
#define GPIO_PIN_SPEED_MEDIUM      (1)
#define GPIO_PIN_SPEED_FAST        (2)
#define GPIO_PIN_SPEED_HIGH        (3)

/*GPIO port pull-up/pull-down mode*/


#define GPIO_PIN_NO_PULLUP_NO_PULLDOWN        (0)
#define GPIO_PIN_PULL_UP                      (1)
#define GPIO_PIN_PULL_DOWN                    (2)
/*((uint32_t) 0x11) is Reserved bits*/



/*GPIO ports adresses*/

#define GPIO_PORT_A       GPIOA
#define GPIO_PORT_B       GPIOB
#define GPIO_PORT_C       GPIOC
#define GPIO_PORT_D       GPIOD
#define GPIO_PORT_E       GPIOE
#define GPIO_PORT_F       GPIOF
#define GPIO_PORT_G       GPIOG
#define GPIO_PORT_H       GPIOH



/*GPIO clock enable*/

#define _HAL_GPIOA_CLK_ENABLE()        (RCC->AHB1ENR |= (1<<0))
#define _HAL_GPIOB_CLK_ENABLE()        (RCC->AHB1ENR |= (1<<1))
#define _HAL_GPIOC_CLK_ENABLE()        (RCC->AHB1ENR |= (1<<2))
#define _HAL_GPIOD_CLK_ENABLE()        (RCC->AHB1ENR |= (1<<3))
#define _HAL_GPIOE_CLK_ENABLE()        (RCC->AHB1ENR |= (1<<4))
#define _HAL_GPIOF_CLK_ENABLE()        (RCC->AHB1ENR |= (1<<5))
#define _HAL_GPIOG_CLK_ENABLE()        (RCC->AHB1ENR |= (1<<6))
#define _HAL_GPIOH_CLK_ENABLE()        (RCC->AHB1ENR |= (1<<7))

/*****************************************************************************/
/*             Data Structur to configure I/O PINS
/                             
*****************************************************************************/

typedef struct
{
	uint32_t pin ;     /*specifies GPIO Pins to be configured */

	uint32_t mode ;      /*specifies GPIO Pin operating mode*/
	
	uint32_t output_type ;  /*specifies GPIO Pin output mode weather poen drain or push pull*/

	uint32_t speed ;   /*specifies GPIO Pin Speed*/

	uint32_t pull;   /*specifies GPIO Pin Pulled up or Pulled down in output mode*/

	uint32_t alternate ;   /*specifies GPIO Pin alternate function value if PIN is in alternate func mode*/


}gpio_pin_confg;

typedef enum
{
	INT_RISING_EDGE,

	INT_FALLING_EDGE,

	INT_RISING_FALLING_EDGE

}INT_edge_select;



/*****************************************************************************/
/*             Functions proto types
/                             
*****************************************************************************/


void HAL_GPIO_INIT(GPIO_TypeDef * GPIOx , gpio_pin_confg * config);


void HAL_GPIO_seet_alt_function(GPIO_TypeDef * GPIOx , uint32_t pinx_no ,uint16_t alt_func_value);


uint8_t HAL_GPIO_read_pin_value(GPIO_TypeDef * GPIOx  , uint32_t pin_no );


void HAL_GPIO_write_pin_value(GPIO_TypeDef * GPIOx  , uint32_t pin_no , uint8_t value);


void HAL_GPIO_configure_interrupt(GPIO_TypeDef * GPIOx, uint16_t pin_no , INT_edge_select edge );


void HAL_GPIO_enable_interrupt(uint16_t pin_no , IRQn_Type irq_no);


void HAL_GPIO_clear_interrupt_flag(uint16_t pin_no);

void HAL_SET_GPIO_CLK(GPIO_TypeDef *GPIOx);

void HAL_GPIO_INT_SYS_CONFIG(uint16_t pin_no);
void HAL_BORAD_BUTTON_IT(void);
void HAL_BORAD_BUTTON(void);


void HAL_GPIO_CONFIGURE_EVENT(GPIO_TypeDef * GPIOx, uint16_t pin_no , INT_edge_select edge );
void HAL_GPIO_EVENT_ENABLE(uint16_t pin_no);
void HAL_BORAD_BUTTON_EV(void);
void HAL_BORAD_BUTTON_EV(void);

#endif /*GOIP_DRIVER_H_*/

