#include "hal_DMA_driver.h"



void HAL_DMA_INIT(DMA_conifg *config)
{
	
    HAL_DMA_CLK_ENABLE(config->DMAx);
	
	  config->DMA_Streamx->CR |=((config->Direction) << DMA_REG_CR_DIR);
	  config->DMA_Streamx->CR|=((config->MIncreamentAddress) << DMA_REG_CR_MINC);
	  config->DMA_Streamx->CR |=((config->PIncreamentAddress) << DMA_REG_CR_PINC);
	  config->DMA_Streamx->CR |=((config->MSize) << DMA_REG_CR_MSIZE);
	  config->DMA_Streamx->CR|=((config->PSize) << DMA_REG_CR_PSIZE);
	  config->DMA_Streamx->CR|=((config->StreamPriority) << DMA_REG_CR_PL);
	  config->DMA_Streamx->CR|=((config->Circular) << DMA_REG_CR_CIRC);
	  config->DMA_Streamx->CR |=((config->StreamChannel) << DMA_REG_CR_CHSEL);
	  config->DMA_Streamx->FCR|=((config->StreamMode) << DMA_REG_FCR_DIRECT_MODE );
	  config->DMA_Streamx->FCR|=((config->FIFOThreshold) << DMA_REG_FCR_FTH);
	

		
}


void HAL_DMA_CLK_ENABLE(DMA_TypeDef *DMAx)
{
  if(DMAx == DMA_1)
	{
	  RCC->AHB1ENR |= ( 1<< 21 );
	}
	else if(DMAx == DMA_2)
	{
	  RCC->AHB1ENR |= ( 1<< 22 );
	}
	else
	{
	  
	}
}


void HAL_DMA_STREAMx_ENABLE(DMA_Stream_TypeDef *DMA_Streamx )
{
   DMA_Streamx->CR |= DMA_REG_CR_DMA_ST_EN;
}


void HAL_DMA_STREAMx_DISABLE(DMA_Stream_TypeDef *DMA_Streamx )
{
   DMA_Streamx->CR &=~ DMA_REG_CR_DMA_ST_EN;
}



void HAL_DMA_Start(DMA_conifg *config , uint32_t *PAddress , uint32_t *MAddress , uint32_t DataLength )
{
  
   config->DMA_Streamx->NDTR = DataLength;
	
	 config->DMA_Streamx->PAR =(PAddress);
	
	
	 config->DMA_Streamx->M0AR =(MAddress);
	
	 HAL_DMA_STREAMx_ENABLE(config->DMA_Streamx);
	  
   
}

void HAL_DMA_INT_Configure(DMA_conifg *config ,IRQn_Type IRQn)
{
  config->DMA_Streamx->CR |= DMA_REG_CR_TCIE;
	NVIC_EnableIRQ(IRQn);
}

