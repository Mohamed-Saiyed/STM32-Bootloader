#ifndef DMA_H_
#define DMA_H_

#include "stdint.h"
#include "stm32f446xx.h"
#include "gpio_driver.h"

/******************************************************
*             
*          1.         DMA
*             rigster Bit Definetions
*
*******************************************************/

/*************DMA Straem CR BITS***************/


#define DMA_REG_CR_CHSEL    25
#define DMA_CHANNEL_0       0
#define DMA_CHANNEL_1       1
#define DMA_CHANNEL_2       2
#define DMA_CHANNEL_3       3
#define DMA_CHANNEL_4       4
#define DMA_CHANNEL_5       5
#define DMA_CHANNEL_6       6
#define DMA_CHANNEL_7       7


#define DMA_REG_CR_PL      	16	
#define DMA_PRIORITY_LOW     0
#define DMA_PRIORITY_MEDIUM  1
#define DMA_PRIORITY_HIGH    2
#define DMA_CHANNEL_VHIGH    3


#define DMA_REG_CR_MSIZE      	13	
#define DMA_MSIZE_BYTE           0
#define DMA_MSIZE_HALF_WORD      1
#define DMA_MSIZE_WORD           2


#define DMA_REG_CR_PSIZE      	11	
#define DMA_PSIZE_BYTE           0
#define DMA_PSIZE_HALF_WORD      1
#define DMA_PSIZE_WORD           2

#define DMA_REG_CR_CIRC       	 8	
#define DMA_CIRCULAR_DISABLE     0
#define DMA_CIRCULAR_ENABLE 	   1


#define DMA_REG_CR_MINC       	10	
#define DMA_MINC_DISABLE         0
#define DMA_MINC_ENABLE 	       1

#define DMA_REG_CR_PINC       	 9	
#define DMA_PINC_DISABLE         0
#define DMA_PINC_ENABLE 	       1
 
#define DMA_REG_CR_DIR        	 6	
#define DMA_DIR_P2M              0
#define DMA_DIR_M2P              1
#define DMA_DIR_M2M              2 


#define DMA_REG_CR_TCIE        ((uint32_t)1 << 4 )

#define DMA_REG_CR_HTIE        ((uint32_t)1 << 3 )
 
#define DMA_REG_CR_TEIE        ((uint32_t)1 << 2 )
 
#define DMA_REG_CR_DMEIE       ((uint32_t)1 << 1 )
 
#define DMA_REG_CR_DMA_ST_EN   ((uint32_t)1 << 0 )

/*************DMA Stream FCR BITS***************/

#define DMA_REG_FCR_FEIE        ((uint32_t)1 << 7 )

#define DMA_REG_FCR_DIRECT_MODE       2
#define DMA_DIRECT_MODE               0
#define DMA_FIFO_MODE                 1


#define DMA_REG_FCR_FTH        	      0
#define DMA_FTH_DISABLE               0
#define DMA_FTH_1_4_FIFO              0
#define DMA_FTH_1_2_FIFO              1
#define DMA_FTH_3_4_FIFO              2 
#define DMA_FTH_FULL_FIFO             3 

/*************DMA LISR BITS***************/

#define DMA_REG_LISR_FEIF0     ((uint32_t)1 << 0 )
#define DMA_REG_LISR_DMEIF0    ((uint32_t)1 << 2 )
#define DMA_REG_LISR_TEIF0     ((uint32_t)1 << 3 )
#define DMA_REG_LISR_HTIF0     ((uint32_t)1 << 4 )
#define DMA_REG_LISR_TCIF0     ((uint32_t)1 << 5 )

#define DMA_REG_LISR_FEIF1     ((uint32_t)1 << 6 )
#define DMA_REG_LISR_DMEIF1    ((uint32_t)1 << 8 )
#define DMA_REG_LISR_TEIF1     ((uint32_t)1 << 9 )
#define DMA_REG_LISR_HTIF1     ((uint32_t)1 << 10 )
#define DMA_REG_LISR_TCIF1     ((uint32_t)1 << 11 )

#define DMA_REG_LISR_FEIF2     ((uint32_t)1 << 16 )
#define DMA_REG_LISR_DMEIF2    ((uint32_t)1 << 18 )
#define DMA_REG_LISR_TEIF2     ((uint32_t)1 << 19 )
#define DMA_REG_LISR_HTIF2     ((uint32_t)1 << 20 )
#define DMA_REG_LISR_TCIF2     ((uint32_t)1 << 21 )

#define DMA_REG_LISR_FEIF3     ((uint32_t)1 << 22 )
#define DMA_REG_LISR_DMEIF3    ((uint32_t)1 << 24 )
#define DMA_REG_LISR_TEIF3     ((uint32_t)1 << 25 )
#define DMA_REG_LISR_HTIF3     ((uint32_t)1 << 26 )
#define DMA_REG_LISR_TCIF3     ((uint32_t)1 << 27 )

/*************DMA HISR BITS***************/

#define DMA_REG_HISR_FEIF4     ((uint32_t)1 << 0 )
#define DMA_REG_HISR_DMEIF4    ((uint32_t)1 << 2 )
#define DMA_REG_HISR_TEIF4     ((uint32_t)1 << 3 )
#define DMA_REG_HISR_HTIF4     ((uint32_t)1 << 4 )
#define DMA_REG_HISR_TCIF4     ((uint32_t)1 << 5 )

#define DMA_REG_HISR_FEIF5     ((uint32_t)1 << 6 )
#define DMA_REG_HISR_DMEIF5    ((uint32_t)1 << 8 )
#define DMA_REG_HISR_TEIF5     ((uint32_t)1 << 9 )
#define DMA_REG_HISR_HTIF5     ((uint32_t)1 << 10 )
#define DMA_REG_HISR_TCIF5     ((uint32_t)1 << 11 )

#define DMA_REG_HISR_FEIF6     ((uint32_t)1 << 16 )
#define DMA_REG_HISR_DMEIF6    ((uint32_t)1 << 18 )
#define DMA_REG_HISR_TEIF6     ((uint32_t)1 << 19 )
#define DMA_REG_HISR_HTIF6     ((uint32_t)1 << 20 )
#define DMA_REG_HISR_TCIF6     ((uint32_t)1 << 21 )

#define DMA_REG_HISR_FEIF7     ((uint32_t)1 << 22 )
#define DMA_REG_HISR_DMEIF7    ((uint32_t)1 << 24 )
#define DMA_REG_HISR_TEIF7     ((uint32_t)1 << 25 )
#define DMA_REG_HISR_HTIF7     ((uint32_t)1 << 26 )
#define DMA_REG_HISR_TCIF7     ((uint32_t)1 << 27 )

/*************DMA2 LIFCR BITS***************/

#define DMA2_CLEAR_FEIF0_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 0 )
#define DMA2_CLEAR_DMEIF0_FLAG()       DMA2->LIFCR|=((uint32_t)1 << 2 )
#define DMA2_CLEAR_TEIF0_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 3 )
#define DMA2_CLEAR_HTIF0_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 4 )
#define DMA2_CLEAR_TCIF0_FLAG()        DMA2->LIFCR|= ((uint32_t)1 << 5 )

#define DMA2_CLEAR_FEIF1_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 6 )
#define DMA2_CLEAR_DMEIF1_FLAG()       DMA2->LIFCR|=((uint32_t)1 << 8 )
#define DMA2_CLEAR_TEIF1_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 9 )
#define DMA2_CLEAR_HTIF1_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 10 )
#define DMA2_CLEAR_TCIF1_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 11 )
 
#define DMA2_CLEAR_FEIF2_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 16 )
#define DMA2_CLEAR_DMEIF2_FLAG()       DMA2->LIFCR|=((uint32_t)1 << 18 )
#define DMA2_CLEAR_TEIF2_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 19 )
#define DMA2_CLEAR_HTIF2_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 20 )
#define DMA2_CLEAR_TCIF2_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 21 )

#define DMA2_CLEAR_FEIF3_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 22 )
#define DMA2_CLEAR_DMEIF3_FLAG()       DMA2->LIFCR|=((uint32_t)1 << 24 )
#define DMA2_CLEAR_TEIF3_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 25 )
#define DMA2_CLEAR_HTIF3_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 26 )
#define DMA2_CLEAR_TCIF3_FLAG()        DMA2->LIFCR|=((uint32_t)1 << 27 )

/*************DMA2 HIFCR BITS***************/

#define DMA2_CLEAR_FEIF4_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 0 )
#define DMA2_CLEAR_DMEIF4_FLAG()       DMA2->HIFCR|=((uint32_t)1 << 2 )
#define DMA2_CLEAR_TEIF4_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 3 )
#define DMA2_CLEAR_HTIF4_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 4 )
#define DMA2_CLEAR_TCIF4_FLAG()        DMA2->HIFCR|= ((uint32_t)1 << 5 )

#define DMA2_CLEAR_FEIF5_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 6 )
#define DMA2_CLEAR_DMEIF5_FLAG()       DMA2->HIFCR|=((uint32_t)1 << 8 )
#define DMA2_CLEAR_TEIF5_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 9 )
#define DMA2_CLEAR_HTIF5_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 10 )
#define DMA2_CLEAR_TCIF5_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 11 )
 
#define DMA2_CLEAR_FEIF6_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 16 )
#define DMA2_CLEAR_DMEIF6_FLAG()       DMA2->HIFCR|=((uint32_t)1 << 18 )
#define DMA2_CLEAR_TEIF6_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 19 )
#define DMA2_CLEAR_HTIF6_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 20 )
#define DMA2_CLEAR_TCIF6_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 21 )

#define DMA2_CLEAR_FEIF7_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 22 )
#define DMA2_CLEAR_DMEIF7_FLAG()       DMA2->HIFCR|=((uint32_t)1 << 24 )
#define DMA2_CLEAR_TEIF7_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 25 )
#define DMA2_CLEAR_HTIF7_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 26 )
#define DMA2_CLEAR_TCIF7_FLAG()        DMA2->HIFCR|=((uint32_t)1 << 27 )


/*************DMA1 LIFCR BITS***************/

#define DMA1_CLEAR_FEIF0_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 0 )
#define DMA1_CLEAR_DMEIF0_FLAG()       DMA1->LIFCR|=((uint32_t)1 << 2 )
#define DMA1_CLEAR_TEIF0_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 3 )
#define DMA1_CLEAR_HTIF0_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 4 )
#define DMA1_CLEAR_TCIF0_FLAG()        DMA1->LIFCR|= ((uint32_t)1 << 5 )

#define DMA1_CLEAR_FEIF1_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 6 )
#define DMA1_CLEAR_DMEIF1_FLAG()       DMA1->LIFCR|=((uint32_t)1 << 8 )
#define DMA1_CLEAR_TEIF1_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 9 )
#define DMA1_CLEAR_HTIF1_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 10 )
#define DMA1_CLEAR_TCIF1_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 11 )
 
#define DMA1_CLEAR_FEIF2_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 16 )
#define DMA1_CLEAR_DMEIF2_FLAG()       DMA1->LIFCR|=((uint32_t)1 << 18 )
#define DMA1_CLEAR_TEIF2_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 19 )
#define DMA1_CLEAR_HTIF2_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 20 )
#define DMA1_CLEAR_TCIF2_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 21 )

#define DMA1_CLEAR_FEIF3_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 22 )
#define DMA1_CLEAR_DMEIF3_FLAG()       DMA1->LIFCR|=((uint32_t)1 << 24 )
#define DMA1_CLEAR_TEIF3_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 25 )
#define DMA1_CLEAR_HTIF3_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 26 )
#define DMA1_CLEAR_TCIF3_FLAG()        DMA1->LIFCR|=((uint32_t)1 << 27 )

/*************DMA1 HIFCR BITS***************/

#define DMA1_CLEAR_FEIF4_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 0 )
#define DMA1_CLEAR_DMEIF4_FLAG()       DMA1->HIFCR|=((uint32_t)1 << 2 )
#define DMA1_CLEAR_TEIF4_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 3 )
#define DMA1_CLEAR_HTIF4_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 4 )
#define DMA1_CLEAR_TCIF4_FLAG()        DMA1->HIFCR|= ((uint32_t)1 << 5 )

#define DMA1_CLEAR_FEIF5_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 6 )
#define DMA1_CLEAR_DMEIF5_FLAG()       DMA1->HIFCR|=((uint32_t)1 << 8 )
#define DMA1_CLEAR_TEIF5_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 9 )
#define DMA1_CLEAR_HTIF5_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 10 )
#define DMA1_CLEAR_TCIF5_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 11 )
 
#define DMA1_CLEAR_FEIF6_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 16 )
#define DMA1_CLEAR_DMEIF6_FLAG()       DMA1->HIFCR|=((uint32_t)1 << 18 )
#define DMA1_CLEAR_TEIF6_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 19 )
#define DMA1_CLEAR_HTIF6_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 20 )
#define DMA1_CLEAR_TCIF6_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 21 )

#define DMA1_CLEAR_FEIF7_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 22 )
#define DMA1_CLEAR_DMEIF7_FLAG()       DMA1->HIFCR|=((uint32_t)1 << 24 )
#define DMA1_CLEAR_TEIF7_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 25 )
#define DMA1_CLEAR_HTIF7_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 26 )
#define DMA1_CLEAR_TCIF7_FLAG()        DMA1->HIFCR|=((uint32_t)1 << 27 )

/***********************DMAx base adresses********************************/

#define DMA_1               DMA1
#define DMA_2               DMA2
/************************************************************/

/******************************************************
*             
*          1.          USART 
*                 Data Structure
*
*******************************************************/


typedef struct
{
  DMA_Stream_TypeDef  *DMA_Streamx;
	
	DMA_TypeDef  *DMAx;
	
	uint32_t Direction;
	
	uint32_t Circular;
	 		
	uint32_t PIncreamentAddress;
	
	uint32_t MIncreamentAddress;
	
	uint32_t PSize;
	
	uint32_t MSize;
	
	uint32_t StreamPriority;
	
	uint32_t StreamChannel;
	
	uint32_t StreamMode;
	
	uint32_t FIFOThreshold;
	
	
}DMA_conifg;	




void HAL_DMA_CLK_ENABLE(DMA_TypeDef *DMAx);
void HAL_DMA_STREAMx_ENABLE(DMA_Stream_TypeDef *DMA_Streamx );
void HAL_DMA_STREAMx_DISABLE(DMA_Stream_TypeDef *DMA_Streamx );
void HAL_DMA_INIT(DMA_conifg *config);
void HAL_DMA_INT_Configure(DMA_conifg *config ,IRQn_Type IRQn);
void HAL_DMA_Start(DMA_conifg *config , uint32_t *PAddress , uint32_t *MAddress , uint32_t DataLength );
void HAL_USART_DISABLE_DMAT(USART_TypeDef *USARTx );
void HAL_USART_ENABLE_DMAT(USART_TypeDef *USARTx );
void HAL_USART_DISABLE_DMAR(USART_TypeDef *USARTx );
void HAL_USART_ENABLE_DMAR(USART_TypeDef *USARTx );







#endif  /*DMA_H_*/
