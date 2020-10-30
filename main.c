#include "main.h"


volatile int x ;
USART_config uart;
FLASH_TypeDef *FLASH1 = FLASH;
uint32_t flah_status = 0;

uint8_t supported_command[] ={BL_GET_VER , BL_GET_CID , BL_GET_RDP_STATUS , BL_FLASH_ERASE,
	                            BL_GO_TO_ADDR , BL_READ_SECTOR_STATUS , BL_MEM_READ , BL_MEM_WRITE,
	                            BL_ENDIS_RW_PROTECT , BL_OTP_READ};
	

uint8_t BL_RX_BUFFER[BL_RX_LEN];

									

int main(void)
{
	UART_INIT();
	HAL_BORAD_BUTTON();
  board_led_init();
	HAL_CRC_INIT();
	
	if(HAL_GPIO_read_pin_value(GPIOC , 13) == RESET)
	{
		HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:Button is pressed...going to BL mode\r\n");
		bootloader_uart_read_data();
	}	  
	else
	{
		HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:Button is pressed...executing user app\r\n");
	
		bootloader_jump_to_user_app();
	}
		
	return 0;
}

void UART_INIT(void)
{
  memset(&uart , 0 , sizeof(uart));
	uart.Baudrate = USART_BAUD_115200 ;
	uart.mode = Transmiter_Reciever;
	uart.Stopbits = USART_STOP_1;
	uart.word_lenght = USART_8_BIT_DATA;
	uart.OverSampling = USART_OVERSAMPLE_16;
	uart.parity = USART_PARITY_NO;
	HAL_USART_INIT(BL_UART , &uart);
	HAL_USART_INIT(USART_3 , &uart);
	
}
void bootloader_jump_to_user_app(void)
{
   void(*app_rest_handler)(void);
	
	 uint32_t MSP_value = *(volatile uint32_t *)FLASH_SECTOR_2_BASE;
	
	 HAL_USART_Send_String(BL_UART , "BL_DEBUG_MSG:bootloader_jump_to_user_app\r\n");
	 
	 __set_MSP(MSP_value);
	
	 uint32_t reset_handler = *(volatile uint32_t *)FLASH_RESET_HANDLER_BASE;
	
	 app_rest_handler = (void*) reset_handler;
	
	 app_rest_handler();

}

void bootloader_uart_read_data(void)
{
   uint32_t rcv_len = 0;
	 
	 while(1)
	 {
			memset(BL_RX_BUFFER , 0 , BL_RX_LEN);

			BL_RX_BUFFER[0]= HAL_USART_RECIEVE(BL_UART);
			rcv_len = BL_RX_BUFFER[0];
			for(int i = 1 ; i <= rcv_len ; i++)
			{
				BL_RX_BUFFER[i]= HAL_USART_RECIEVE(BL_UART);
			}
		  switch(BL_RX_BUFFER[1])
			{
			  case BL_GET_VER:
				 bootloader_handle_getver_cmd(BL_RX_BUFFER);	
				break;
				case BL_GET_HELP:
				 bootloader_handle_gethelp_cmd(BL_RX_BUFFER);
			  break;
				case BL_GET_CID:
				 bootloader_handle_getcid_cmd(BL_RX_BUFFER); 	
				break;
				case BL_GET_RDP_STATUS:
				 bootloader_handle_getrdp_cmd(BL_RX_BUFFER);
				break;
				case BL_GO_TO_ADDR:
				 bootloader_handle_go_cmd(BL_RX_BUFFER);	
				break;
				case BL_FLASH_ERASE:
				 bootloader_handle_flash_erase_cmd(BL_RX_BUFFER);
				break;
				case BL_MEM_WRITE:
				 bootloader_handle_mem_write_cmd(BL_RX_BUFFER);	
				break;
				case BL_ENDIS_RW_PROTECT:
				 bootloader_handle_rw_protect_cmd(BL_RX_BUFFER);	
				break;
				case BL_DIS_RW_PROTECT:
				 bootloader_handle_dis_rw_protect_cmd(BL_RX_BUFFER);
				break;
				case BL_MEM_READ:
				 bootloader_handle_mem_read_cmd(BL_RX_BUFFER);	
				break;
				case BL_READ_SECTOR_STATUS:
				 bootloader_handle_read_sector_cmd(BL_RX_BUFFER);	
				break;
				case BL_OTP_READ:
				 bootloader_handle_otp_cmd(BL_RX_BUFFER);
				break;
				default:
				HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:INVALID Command code recived from HOST\r\n");
				break;
				
			}
	 
	 }
	
	
}


void bootloader_handle_getver_cmd(uint8_t *pBuffer)
{
	
	uint8_t bl_version;
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_getver_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
		HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
		bootloader_send_ack(pBuffer[0] ,1);
		bl_version = bootloader_get_version();
		HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:Sending bootloader\r\n");
		HAL_USART_SEND(BL_UART , bl_version);
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}
	CRC_DR_RESET();

}
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
		HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
		bootloader_send_ack(pBuffer[0] ,sizeof(supported_command));
		bootloader_uart_write_data(supported_command , sizeof(supported_command));
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}
	CRC_DR_RESET();

}
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t mcu_cid = 0 ;
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
		HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
		bootloader_send_ack(pBuffer[0] ,2);
		mcu_cid =bootloader_get_mcu_id();
		bootloader_uart_write_data((uint8_t*)&mcu_cid , 1);
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}
	
	CRC_DR_RESET();
}
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
	uint8_t rdb_status = 0 ;
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
		bootloader_send_ack(pBuffer[0] ,1);
		rdb_status =bootloader_get_flash_rdp_level();
		bootloader_uart_write_data((uint8_t*)&rdb_status , 1);
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}
	
	CRC_DR_RESET();
}
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
	uint32_t Address = 0 ;
	uint8_t VALID_ADDRESS = VALID_ADDR;
	uint8_t INVALID_ADDRESS = INVALID_ADDR;
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
			HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
			bootloader_send_ack(pBuffer[0] ,1);
			

			Address = *((uint32_t*)(&pBuffer[2]));
			
			if(check_addr_validation(Address) == VALID_ADDR)
			{
				bootloader_uart_write_data(&VALID_ADDRESS,1);
				
				Address +=1; //T-bit 
				
				void(*go_to_addr)(void) = (void*)Address;
				 	
				go_to_addr();
			
			}
			else
			{
			  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:addr_invalid\r\n");
				bootloader_uart_write_data(&INVALID_ADDRESS,1);
			
			}
		
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}
	CRC_DR_RESET();
}
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
	uint8_t Status = 0;
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
			HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
		   	bootloader_send_ack(pBuffer[0] ,1); 	
		
    		Status =	FLASH_EARSE(pBuffer[2] , pBuffer[3]);
		
				bootloader_uart_write_data(&Status,1);
		
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}	
	CRC_DR_RESET();
}

uint8_t bin_buffer[2712];

void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t Status = 0;
	uint32_t Address = 0 ;
	uint8_t VALID_ADDRESS = VALID_ADDR;
	uint8_t INVALID_ADDRESS = INVALID_ADDR;
	uint8_t pay_load = pBuffer[6];
	uint32_t mem_address = ((uint32_t*)0x08008000);
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	
	    HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
			bootloader_send_ack(pBuffer[0] ,1);
			

			Address = ((uint32_t*)0x08008000);
	    for(int i = 0 ; i < 2712 ; i++)
		  {
		    bin_buffer[i] = HAL_USART_RECIEVE(USART_2);
		  }
			
			if(check_addr_validation(Address) == VALID_ADDR)
			{
				bootloader_uart_write_data(&VALID_ADDRESS,1);
				
			  Status =	mem_write(&bin_buffer , mem_address , 2712);
		
				bootloader_uart_write_data(&Status,1);
						
			}
			else
			{
			  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:addr_invalid\r\n");
				bootloader_uart_write_data(&INVALID_ADDRESS,1);
			
			}
		
	
	
	CRC_DR_RESET();
}
void bootloader_handle_mem_read_cmd(uint8_t *pBuffer)
{
	
}

void bootloader_handle_rw_protect_cmd(uint8_t *pBuffer)
{
	uint8_t Status = 0;
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
		  	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
		   	bootloader_send_ack(pBuffer[0] ,1); 	
		
    		Status =	FLASH_RW_PROTECTION(pBuffer[2] , pBuffer[3] , 0);
		
				bootloader_uart_write_data(&Status,1);
		
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}	
	CRC_DR_RESET();
}

void bootloader_handle_dis_rw_protect_cmd(uint8_t *pBuffer)
{
	uint8_t Status = 0;
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
		  	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
		   	bootloader_send_ack(pBuffer[0] ,1); 	
		
    		Status =	FLASH_RW_PROTECTION(0 ,0 , 1);
		
				bootloader_uart_write_data(&Status,1);
		
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}	
	CRC_DR_RESET();
}

void bootloader_handle_read_sector_cmd(uint8_t *pBuffer)
{
	
	uint16_t Status = 0;
	
	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\r\n");
	
	uint32_t command_packet_len = pBuffer[0]+1;
	
	uint32_t host_crc = *((uint32_t*)(pBuffer + command_packet_len - 4));
	 
	
	if(!bootloader_verify_crc(&pBuffer[0] , command_packet_len - 4 ,host_crc)) 
	{
		  	HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Success\r\n");
		   	bootloader_send_ack(pBuffer[0] ,2); 	
		
    		Status =	FLASH_PROTECTION_Status();
		
				bootloader_uart_write_data(&Status,2);
		
	}
	else
	{
	  HAL_USART_Send_String(DEBUG_UART , "BL_DEBUG_MSG:checksum Fail!\r\n");
		bootloader_send_nack();	
	}	
	CRC_DR_RESET();
}
void bootloader_handle_otp_cmd(uint8_t *pBuffer)
{
	
}

uint8_t bootloader_verify_crc(uint8_t *pData ,uint32_t len, uint32_t host_crc)
{
  uint32_t uwCRCValue = 0xFF;
	
	for(uint32_t i = 0 ; i < len ; i++)
	{
	  uint32_t data = pData[i];  
		
		uwCRCValue = HAL_CRC_ACCUMULAT(&data , 1);	
	}
	
	if(uwCRCValue == host_crc)
	{
	 
		return VERIFY_CRC_SUCCESS;
	
	}
	
	return VERIFY_CRC_FAIL;

}


void bootloader_send_ack(uint8_t command_code , uint8_t follow_len)
{
  uint8_t ack_buffer[2];
	ack_buffer[0] = BL_ACK;
	ack_buffer[1] = follow_len;
	HAL_USART_SEND(BL_UART , BL_ACK );
	HAL_USART_SEND(BL_UART ,follow_len );

}

void bootloader_send_nack(void)
{
	uint8_t NACK = BL_NACK;
	HAL_USART_SEND(BL_UART , NACK);
	
}

void bootloader_uart_write_data(uint8_t *pBuffer , uint32_t len)
{
 HAL_USART_SEND_BUFFER(BL_UART , pBuffer , len);
}

uint8_t bootloader_get_version(void)
{
 return (uint8_t)BL_VERSION;
}

uint16_t bootloader_get_mcu_id(void)
{
 uint16_t cid ;
	
 cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;	
	
 return cid;	

}

uint8_t bootloader_get_flash_rdp_level(void)
{
  uint8_t rdp_status = 0;
	
	volatile uint32_t *OB_rdp = (uint32_t*)0x1FFFC000;
	rdp_status = (uint8_t)(*OB_rdp >> 8) ;
	
	return rdp_status;

}

uint8_t check_addr_validation(uint32_t addr)
{
 if(addr >= SRAM1_BASE && addr <= SRAM1_END)
 {
	 return VALID_ADDR;
 }
 else if(addr >= SRAM2_BASE && addr <= SRAM2_END)
 {
	 return VALID_ADDR;
 }
 else if(addr >= FLASH_BASE && addr <= FLASH_END)
 {
	 return VALID_ADDR;
 }
 else if(addr >= BKPSRAM_BASE && addr <= BKPSRAM_END)
 {
	 return VALID_ADDR;
 }
 else
 {
	 return INVALID_ADDR;
 }

}

void FLASH_UNLOCK(void)
{
  
	if((FLASH1->CR & FLASH_CR_LOCK) != RESET)
	{	  
	  FLASH1->KEYR = FLASH_KEY_1 ;
	  FLASH1->KEYR = FLASH_KEY_2 ;
	}
	else
	{
	  
	}
 
}

void FLASH_LOCK(void)
{
  FLASH1->CR |= FLASH_CR_LOCK;
}

HAL_FlashStatusTypeDef FLASH_PROGRAM_BYTE(uint32_t Address , uint8_t data)
{
	while(((FLASH1->SR) & FLASH_SR_BSY) != 0);
	if(((FLASH1->SR) & FLASH_SR_BSY) == 0)
	{
	FLASH1->CR &=~(0x03 << 8);	
	FLASH1->CR |= FLASH_CR_PG;	
	
	*(volatile uint8_t*)Address = data;
	
	 return Flash_HAL_OK ;
 	}
	else
		return Flash_HAL_ERROR;
}	

HAL_FlashStatusTypeDef mem_write(uint8_t *pBuffer , uint32_t mem_addr , uint32_t len)
{
  uint8_t status = Flash_HAL_OK;
	FLASH_UNLOCK();
	
	for(uint32_t i = 0 ; i< len ; i++)
	{
	  status = FLASH_PROGRAM_BYTE(mem_addr +i ,pBuffer[i]);
	}
	
	FLASH_LOCK();
	
	return status ;

}

HAL_FlashStatusTypeDef Flash_sector_earse(uint32_t sector_num)
{

	while(((FLASH1->SR) & FLASH_SR_BSY) != 0);
	
	if(((FLASH1->SR) & FLASH_SR_BSY) == 0)
	{
		if(sector_num <= 7)
		{
			FLASH1->CR |= (2 << FLASH_CR_PSIZE_Pos);
			FLASH1->CR |= FLASH_CR_SER;
			FLASH1->CR |=(sector_num << 3);
			FLASH->CR |=FLASH_CR_STRT;
			return Flash_HAL_OK ;
	 	}
		else
		{
			return Flash_HAL_ERROR;
		}
 	}
	else
	{
	 return Flash_HAL_ERROR;
	}	
	
}

HAL_FlashStatusTypeDef Flash_Mass_earse(void)
{
	while(((FLASH1->SR) & FLASH_SR_BSY) != 0);
	
	if(((FLASH1->SR) & FLASH_SR_BSY) == 0)
	{
			FLASH1->CR |= (2 << FLASH_CR_PSIZE_Pos);
			FLASH1->CR |= FLASH_CR_MER;
			FLASH->CR |=FLASH_CR_STRT;
		  return Flash_HAL_OK ;
	}	
	else
	{
	 return Flash_HAL_ERROR;
	}	

}
 
HAL_FlashStatusTypeDef FLASH_EARSE(uint32_t sector_num , uint32_t numOfsectors)
{
  uint8_t status = Flash_HAL_OK;
	FLASH_UNLOCK();
	if(sector_num == 0xFF)
	{
	   status = Flash_Mass_earse();
		 while(((FLASH1->SR) & FLASH_SR_BSY) != 0);
	}
	else
	{
		for(uint32_t i = 0 ; i< numOfsectors ; i++)
		{
			status = Flash_sector_earse(sector_num++);
			while(((FLASH1->SR) & FLASH_SR_BSY) != 0);
		}
		
	}
	
	FLASH_LOCK();
	
	return status ;

}
HAL_FlashStatusTypeDef FLASH_RW_PROTECTION(uint8_t sector_num , uint8_t protection_mode , uint32_t disable)
{
	//just disable RW protection level 1
	if(disable == 1)
	{
	    FLASH_OPT_UNLOCK();
			 
			while(((FLASH1->SR) & FLASH_SR_BSY) != 0);
			
			FLASH1->OPTCR &=~FLASH_OPTCR_SPRMOD;
			 
			FLASH1->OPTCR |=(0xFF << 16 );
			
			FLASH->OPTCR |=FLASH_OPTCR_OPTSTRT;
			
			while(((FLASH1->SR) & FLASH_SR_BSY) != 0);	

      FLASH_OPT_LOCK();		
			
			return Flash_HAL_OK ;
	
	}		
	
	if(protection_mode == 1)
	{
			FLASH_OPT_UNLOCK();
			
			while(((FLASH1->SR) & FLASH_SR_BSY) != 0);
			
			FLASH1->OPTCR &=~FLASH_OPTCR_SPRMOD;
			 
			FLASH1->OPTCR &=~(sector_num << 16 );
			
			FLASH->OPTCR |=FLASH_OPTCR_OPTSTRT;
			
			while(((FLASH1->SR) & FLASH_SR_BSY) != 0);	
		
			FLASH_OPT_LOCK();	
		
			return Flash_HAL_OK ;
		
	}
	else if(protection_mode == 2)
	{
			FLASH_OPT_UNLOCK();
			
			while(((FLASH1->SR) & FLASH_SR_BSY) != 0);
			
			FLASH1->OPTCR |=FLASH_OPTCR_SPRMOD;
			 
		  FLASH1->OPTCR &=~(0xFF << 16 );
			FLASH1->OPTCR |=(sector_num << 16 );
			
			FLASH->OPTCR |=FLASH_OPTCR_OPTSTRT;
			
			while(((FLASH1->SR) & FLASH_SR_BSY) != 0);		
		
			FLASH_OPT_LOCK();	 
		
			return Flash_HAL_OK ;
		
	}
  return 0;
}

uint16_t FLASH_PROTECTION_Status(void)
{
  uint16_t WPRstatus = 0;
	
  FLASH_OPT_UNLOCK();
 
	
	WPRstatus = Get_WPR();
		
	FLASH_OPT_LOCK();
	
	return WPRstatus;

}

uint16_t Get_WPR(void)
{
	flah_status =(FLASH->OPTCR ) >> 16;
 return (uint16_t)(flah_status);
}

void FLASH_OPT_UNLOCK(void)
{
  if((FLASH1->OPTCR & FLASH_OPTCR_OPTLOCK) !=RESET)
	{
	   FLASH1->OPTKEYR = FLASH_OPT_KEY_1;
		 FLASH1->OPTKEYR = FLASH_OPT_KEY_2;
	}
	else
	{
	
	}
}
void FLASH_OPT_LOCK(void)
{
  FLASH1->OPTCR |= FLASH_OPTCR_OPTLOCK;
}

