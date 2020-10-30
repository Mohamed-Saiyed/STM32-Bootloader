#include "hal_crc_driver.h"

CRC_TypeDef *CRC1 = CRC;

void HAL_CRC_INIT(void)
{
  HAL_CRC_SET_CLK();

}

void HAL_CRC_SET_CLK(void)
{
  _HAL_CRC_CLK_ENABLE();

}

uint32_t HAL_CRC_ACCUMULAT(uint32_t pBuffer[] , uint32_t BufferLenght)
{
	
  uint32_t index ;
	uint32_t tempcrc = 0;
	
	for(index = 0 ; index < BufferLenght ; index++)
	{
	   CRC1->DR = pBuffer[index];
	}
	
	tempcrc = CRC1->DR;
	
  return tempcrc;
	
}

uint32_t HAL_CRC_CALCULATE(uint32_t pBuffer[] , uint32_t BufferLenght)
{
	
  uint32_t index;
	uint32_t tempcrc = 0;
	
	CRC_DR_RESET();
	
	for(index = 0 ; index < BufferLenght ; index++)
	{
	   CRC1->DR = pBuffer[index];
	}
	
	tempcrc = CRC1->DR;
	
  return tempcrc;
	
}

void CRC_DR_RESET(void)
{
  CRC1->CR |= CRC_REG_CR_RESET;
}
