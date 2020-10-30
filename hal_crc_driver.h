#ifndef _CRC_DRIVER
#define _CRC_DRIVER

#include "stm32f446xx.h"
#include "stdint.h"


/*********CRC CLOCK Enable*******/
#define _HAL_CRC_CLK_ENABLE()   ((RCC->AHB1ENR) |= (1 << 12))

/******************************************************
*             
*          1.          CRC 
*             rigster Bit Definetions
*
*******************************************************/
#define CRC_REG_CR_RESET      ((uint32_t)1 << 0);

/*******************************************************/

void HAL_CRC_INIT(void);
void HAL_CRC_SET_CLK(void);
uint32_t HAL_CRC_ACCUMULAT(uint32_t pBuffer[] , uint32_t BufferLenght);
uint32_t HAL_CRC_CALCULATE(uint32_t pBuffer[] , uint32_t BufferLenght);
void CRC_DR_RESET(void);

#endif /*_CRC_DRIVER*/

