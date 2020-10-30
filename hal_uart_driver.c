	#include "hal_uart_driver.h"

static volatile void(*USART_CALL_BACK)(void);

void HAL_USART_INIT(USART_TypeDef *USARTx , USART_config * config)
{
	  HAL_USART_Disable(USARTx);
	  HAL_USART_SET_PINS(USARTx);
	  HAL_SET_USART_CLK(USARTx);
  	USARTx->BRR = config->Baudrate ;       
	  USARTx->CR1 |= ((config->OverSampling) << 15 );
	  USARTx->CR1 |= ((config->word_lenght) << 12 );
    HAL_USART_set_RX_TX(USARTx , config->mode ) ;
    USARTx->CR2 |= (config->Stopbits << (USART_REG_CR2_STOP_BITS) );	
	  if(config->parity == USART_PARITY_NO)
		{
		   USARTx->CR1 &=~ USART_REG_CR1_PCE;
		}
		else if(config->parity != USART_PARITY_NO)
		{
			USARTx->CR1 |=  USART_REG_CR1_PCE;
		  USARTx->CR1 |= ((config->parity) << 9);
		}
		else
		{
      ;		
		}
	  HAL_USART_ENABLE(USARTx);
}

void HAL_USART_ENABLE(USART_TypeDef *USARTx)
{
   USARTx->CR1 |= USART_REG_CR1_UE;
}
	

void HAL_USART_Disable(USART_TypeDef *USARTx)
{
   USARTx->CR1 &=~ USART_REG_CR1_UE;
}

void HAL_USART_set_RX_TX(USART_TypeDef *USARTx , USART_MODE mode )
{
   USARTx->CR1 &=~ USART_REG_CR1_TE;
	 USARTx->CR1 &=~ USART_REG_CR1_RE;
	  
   switch(mode)
	 {
		 case Transmiter :
		 USARTx->CR1 |= USART_REG_CR1_TE;
		 break;
		 case Reciever :
		 USARTx->CR1 |= USART_REG_CR1_RE;
		 break;
		 case Transmiter_Reciever :
		 USARTx->CR1 |= USART_REG_CR1_TE;
		 USARTx->CR1 |= USART_REG_CR1_RE;
		 break;
	   default :
		 break;
	 
	 }

		 

}

void HAL_USART_SET_PINS(USART_TypeDef *USARTx)
{
	
	 gpio_pin_confg UART_GPIO;
	  
	 UART_GPIO.mode = GPIO_PIN_ALT_FUN_MODE;
	 UART_GPIO.pull = GPIO_PIN_PULL_UP;
	 UART_GPIO.output_type = GPIO_PIN_NO_OP_TYPE;
	 UART_GPIO.speed = GPIO_PIN_SPEED_LOW;
	  
   if(USARTx == USART_1)
	 {
	   _HAL_GPIOA_CLK_ENABLE();
		 
		 UART_GPIO.pin = USART1_TX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_A , USART1_TX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_A , &UART_GPIO);

		 
		 UART_GPIO.pin = USART1_RX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_A ,USART1_RX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_A , &UART_GPIO);
	 
	 }
 	 else if(USARTx == USART_2)
	 {
	   _HAL_GPIOA_CLK_ENABLE();
		 
		 UART_GPIO.pin = USART2_TX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_A , USART2_TX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_A , &UART_GPIO);
		 
		 UART_GPIO.pin = USART2_RX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_A ,USART2_RX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_A , &UART_GPIO);
	 
	 }
	 	else  if(USARTx == USART_3)
	 {
	 
		 _HAL_GPIOC_CLK_ENABLE();
		 
		 UART_GPIO.pin = USART3_TX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_C , USART3_TX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_C , &UART_GPIO);
		 
		
		 
		 UART_GPIO.pin = USART3_RX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_C ,USART3_RX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_C , &UART_GPIO);
	 
	 }
	 	else  if(USARTx == UART_4)
	 {
	    _HAL_GPIOA_CLK_ENABLE();
		 
		 UART_GPIO.pin = UART4_TX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_A , UART4_TX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_A , &UART_GPIO);
		 
		 UART_GPIO.pin = UART4_RX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_A ,UART4_RX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_A , &UART_GPIO);
	 
	   
	 }
	 	else  if(USARTx == UART_5)
	 {
	     _HAL_GPIOC_CLK_ENABLE();
		   _HAL_GPIOD_CLK_ENABLE();
		 
		 UART_GPIO.pin = UART5_TX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_C, UART5_TX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_C , &UART_GPIO);
		 
		
		 
		 UART_GPIO.pin = UART5_RX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_D ,UART5_RX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_D , &UART_GPIO);
	 
	 }
	 	else  if(USARTx == USART_6)
	 {
	   _HAL_GPIOC_CLK_ENABLE();
		 
		 UART_GPIO.pin = USART6_TX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_C , USART6_TX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_C , &UART_GPIO);
		 
		 UART_GPIO.pin = USART6_RX_PIN ;
		 HAL_GPIO_seet_alt_function(GPIO_PORT_C ,USART6_RX_PIN ,GPIO_PIN_AF7_SPI_2_3_USART_1__3_UART5_SPDIF_IN);
		 HAL_GPIO_INIT(GPIO_PORT_C , &UART_GPIO);
	 
	 }
	 else
	 {
	   ;
	 }

}
void HAL_SET_USART_CLK(USART_TypeDef *USARTx)
{

  if(USARTx == USART_1)
	{
	  _HAL_USART1_CLK_ENABLE();
	}
	else if(USARTx == USART_2)
	{
	 _HAL_USART2_CLK_ENABLE();
	}
	else if(USARTx == USART_3)
	{
	 _HAL_USART3_CLK_ENABLE();
	}
	else if(USARTx == UART_4)
	{
	  _HAL_UART4_CLK_ENABLE();
	}
	else if(USARTx == UART_5)
	{
	 _HAL_UART5_CLK_ENABLE();
	}
	else if(USARTx == USART_6)
	{
	 _HAL_USART6_CLK_ENABLE();
	}
	else
	{
	  ;
	}

}
void HAL_USART_SEND(USART_TypeDef *USARTx , uint8_t data)
{
	USARTx->DR = data ;
	
  while(!(USARTx->SR & (1<<7)));

}
uint8_t HAL_USART_RECIEVE(USART_TypeDef *USARTx )
{
	
	while(!(USARTx->SR & (1<<5)));
  
	return ((uint8_t)(USARTx->DR & 0x00FF));
}

void HAL_USART_Send_String(USART_TypeDef *USARTx , uint8_t *string)
{

	while(*string != '\0')
	{
			HAL_USART_SEND(USARTx , *string);
		  string++;
	}
  HAL_USART_SEND(USARTx , '\0');
   
}

void HAL_USART_SEND_BUFFER(USART_TypeDef *USARTx , uint8_t *pBuffer , uint32_t Bufferlen)
{
  int counter;
	
	for(counter = 0 ; counter < Bufferlen ; counter++)
	{
	  HAL_USART_SEND(USARTx , pBuffer[counter]);
	}
	
   
}

void HAL_USART_Recieve_String(USART_TypeDef *USARTx , uint8_t *string)
{
   /*temporary variable as a buffer*/
	uint8_t data;
	while(1) 
	{
	    data=HAL_USART_RECIEVE(USARTx);
		/*assign data to the string*/
		*string=data;
		/*move to next address*/
		if (data=='\0' || data=='\r' || data=='\n')
		{break;}
		string++;
		
	} 
}

void HAL_USART_RX_interrupt(USART_TypeDef *USARTx , IRQn_Type irq_no)
{
   USARTx->CR1 |= (1 << 5);
	 NVIC_EnableIRQ(irq_no);
}

void HAL_USART_SET_CALL_BACK(void(*g_ptr)(void))
{
     USART_CALL_BACK = g_ptr ; 

}
void USART2_IRQHandler(void)
{
   (*USART_CALL_BACK)() ;
}

void HAL_USART_SEND_DMA(USART_TypeDef *USARTx , DMA_conifg *config , uint32_t *SrcAddress , uint32_t DataLength)
{
	
	 HAL_DMA_Start(config , SrcAddress, (uint32_t)&(USARTx->DR) ,DataLength );
	 HAL_USART_ENABLE_DMAT(USARTx );

}


void HAL_USART_ENABLE_DMAT(USART_TypeDef *USARTx )
{
  USARTx->CR3 |=(1 << 7);
}

void HAL_USART_DISABLE_DMAT(USART_TypeDef *USARTx )
{
  USARTx->CR3 &=~(1 << 7);
}

void HAL_USART_ENABLE_DMAR(USART_TypeDef *USARTx )
{
  USARTx->CR3 |=(1 << 6);
}

void HAL_USART_DISABLE_DMAR(USART_TypeDef *USARTx )
{
  USARTx->CR3 &=~(1 << 6);
}









