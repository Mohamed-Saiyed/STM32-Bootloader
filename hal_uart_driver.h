#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include "stm32f446xx.h"
#include "stdint.h"
#include "gpio_driver.h"
#include "hal_DMA_driver.h"




/**********USART GPIO PINS*************************/
	
/*GPIO_PORT_A*/
#define USART1_TX_PIN      9
#define USART1_RX_PIN      10

/*GPIO_PORT_A*/
#define USART2_TX_PIN      2
#define USART2_RX_PIN      3

/*GPIO_PORT_C*/
#define USART3_TX_PIN      10
#define USART3_RX_PIN      11

/*GPIO_PORT_A*/
#define UART4_TX_PIN       0
#define UART4_RX_PIN       1

/*GPIO_PORT_C_D*/
#define UART5_TX_PIN       12
#define UART5_RX_PIN       2

/*GPIO_PORT_C*/
#define USART6_TX_PIN      6
#define USART6_RX_PIN      7

/*********USARTx CLOCK Enable*******/
#define _HAL_USART1_CLK_ENABLE()  ( RCC->APB2ENR |= (1 << 4))    
#define _HAL_USART2_CLK_ENABLE()  ( RCC->APB1ENR |= (1 << 17))  
#define _HAL_USART3_CLK_ENABLE()  ( RCC->APB1ENR |= (1 << 18))    
#define _HAL_UART4_CLK_ENABLE()   ( RCC->APB1ENR |= (1 << 19))  
#define _HAL_UART5_CLK_ENABLE()   ( RCC->APB1ENR |= (1 << 20))    
#define _HAL_USART6_CLK_ENABLE()  ( RCC->APB2ENR |= (1 << 5))  

/******************************************************
*             
*          1.          USART 
*             rigster Bit Definetions
*
*******************************************************/

/*************USART CR1 BITS***************/

#define USART_REG_CR1_OVER            ((uint32_t) 1 << 15)
#define USART_OVERSAMPLE_16                   0
#define USART_OVERSAMPLE_8                    1

#define USART_REG_CR1_UE              ((uint32_t) (1 << 13))
#define USART_ENABLE                          0
#define USART_DISABLE                         1

#define USART_REG_CR1_M               ((uint32_t) (1 << 12))
#define USART_8_BIT_DATA                      0
#define USART_9_BIT_DATA                      1
        
#define USART_REG_CR1_PEIE            ((uint32_t) (1 << 8))				
#define USART_REG_CR1_TXEIE           ((uint32_t) (1 << 7))         
#define USART_REG_CR1_TCIE            ((uint32_t) (1 << 6)) 
#define USART_REG_CR1_RXNEIE          ((uint32_t) (1 << 5)) 

#define USART_REG_CR1_TE              ((uint32_t) (1 << 3)) 
#define USART_REG_CR1_RE              ((uint32_t) (1 << 2)) 


#define USART_TX_MODE                  USART_REG_CR1_TE
#define USART_RX_MODE                  USART_REG_CR1_RE
#define USART_TX_RX_MODE              ( USART_REG_CR1_TE | USART_REG_CR1_RE )

#define USART_REG_CR1_PCE             ((uint32_t) (1 << 10))
#define USART_PARITY_NO                       5
#define USART_PARITY_EVEN                     0
#define USART_PARITY_ODD                      1


/*************USART CR2 BITS***************/
#define USART_REG_CR2_STOP_BITS          (12) 

#define USART_STOP_1           (uint32_t) (0) 
#define USART_STOP_HALF        (uint32_t) (1) 
#define USART_STOP_2           (uint32_t) (2) 
#define USART_STOP_1_HALF      (uint32_t) (3) 

/*************USART SR BITS***************/

#define USART_REG_SR_TXE             ((uint32_t) (1 << 7))				
#define USART_REG_SR_TC              ((uint32_t) (1 << 6))         
#define USART_REG_SR_RXNE            ((uint32_t) (1 << 5)) 
#define USART_REG_SR_IDLE            ((uint32_t) (1 << 4)) 
#define USART_REG_SR_ORE             ((uint32_t) (1 << 3))				
#define USART_REG_SR_NF              ((uint32_t) (1 << 2))         
#define USART_REG_SR_FE              ((uint32_t) (1 << 1)) 
#define USART_REG_SR_PE              ((uint32_t) (1 << 0)) 

/*************************************************************/

#define USART_BAUD_9600               0x683
#define USART_BAUD_19200              0x341
#define USART_BAUD_115200             0x8B

/***********************USART base adresses********************************/

#define USART_1               USART1
#define USART_2               USART2
#define USART_3               USART3
#define UART_4                UART4
#define UART_5                UART5
#define USART_6               USART6

/************************************************************/

/******************************************************
*             
*          1.          USART 
*                 Data Structure
*
*******************************************************/

typedef enum
{
  Transmiter,
	Reciever,
	Transmiter_Reciever

}USART_MODE;




typedef struct
{
	
  uint32_t Baudrate;
	
  uint32_t word_lenght;
	
	USART_MODE mode ;
	
	uint32_t OverSampling;

	
	uint32_t Stopbits ;
	
	
	uint32_t parity ;
 


}USART_config;



void HAL_USART_set_RX_TX(USART_TypeDef *USARTx , USART_MODE mode );

void HAL_USART_SET_CALL_BACK(void(*g_ptr)(void));

void HAL_USART_RX_interrupt(USART_TypeDef *USARTx , IRQn_Type irq_no);

void HAL_USART_INIT(USART_TypeDef *USARTx , USART_config * config);

void HAL_USART_Recieve_String(USART_TypeDef *USARTx , uint8_t *string);

void HAL_USART_Send_String(USART_TypeDef *USARTx , uint8_t *string);

void HAL_USART_ENABLE(USART_TypeDef *USARTx);

void HAL_USART_Disable(USART_TypeDef *USARTx);

void HAL_USART_SET_PINS(USART_TypeDef *USARTx);

void HAL_SET_USART_CLK(USART_TypeDef *USARTx);

void HAL_USART_SEND(USART_TypeDef *USARTx , uint8_t data);

uint8_t HAL_USART_RECIEVE(USART_TypeDef *USARTx );


void HAL_USART_SEND_DMA(USART_TypeDef *USARTx , DMA_conifg *config , uint32_t *SrcAddress , uint32_t DataLength);

void HAL_USART_SEND_BUFFER(USART_TypeDef *USARTx , uint8_t *pBuffer , uint32_t Bufferlen);

#endif /*UART_DRIVER_H_*/