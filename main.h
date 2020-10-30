#include "stm32f446xx.h"
#include "stdlib.h"
#include "stdio.h"
#include "hal_uart_driver.h"
#include "gpio_driver.h"
#include "led.h"
#include "hal_crc_driver.h"

#define BL_UART      USART_2
#define DEBUG_UART   USART_3

#define SRAM1_SIZE  (112*1024)
#define SRAM1_END   (SRAM1_BASE + SRAM1_SIZE)

#define SRAM2_SIZE  (16*1024)
#define SRAM2_END   (SRAM2_BASE + SRAM2_SIZE)

#define BKPSRAM_SIZE  (4*1024)
#define BKPSRAM_END  (BKPSRAM_BASE + BKPSRAM_SIZE)

#define VALID_ADDR     0
#define INVALID_ADDR   1


#define BL_RX_LEN   200

#define RESET  0
#define FLASH_SECTOR_2_BASE       0x08008000U
#define FLASH_RESET_HANDLER_BASE  0x08008004U

#define BL_GET_VER                0x51
#define BL_GET_HELP               0x52
#define BL_GET_CID                0x53
#define BL_GET_RDP_STATUS         0x54
#define BL_GO_TO_ADDR             0x55
#define BL_FLASH_ERASE            0x56
#define BL_MEM_WRITE              0x57
#define BL_ENDIS_RW_PROTECT       0x58
#define BL_MEM_READ               0x59
#define BL_READ_SECTOR_STATUS     0x5A
#define BL_OTP_READ               0x5B
#define BL_DIS_RW_PROTECT				  0x5C

#define BL_ACK     0xA5
#define BL_NACK    0x7F

#define BL_VERSION 0x02

#define VERIFY_CRC_SUCCESS    0
#define VERIFY_CRC_FAIL       1

#define FLASH_KEY_1       0x45670123
#define FLASH_KEY_2       0xCDEF89AB

#define FLASH_OPT_KEY_1       0x08192A3B
#define FLASH_OPT_KEY_2       0x4C5D6E7F

#define OPTCR_BYTE2_ADDRESS   0x40023C16U

typedef enum
{
  Flash_HAL_OK       = 0x00U,
  Flash_HAL_ERROR    = 0x01U,
  Flash_HAL_BUSY     = 0x02U,
  Flash_HAL_TIMEOUT  = 0x03U,
  Flash_HAL_INV_ADDR = 0x04U
} HAL_FlashStatusTypeDef;						

void UART_INIT(void);
void bootloader_jump_to_user_app(void);
void bootloader_uart_read_data(void);
void bootloader_handle_getver_cmd(uint8_t *pBuffer);
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);
void bootloader_handle_go_cmd(uint8_t *pBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_read_cmd(uint8_t *pBuffer);
void bootloader_handle_rw_protect_cmd(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect_cmd(uint8_t *pBuffer);
void bootloader_handle_read_sector_cmd(uint8_t *pBuffer);
void bootloader_handle_otp_cmd(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect_cmd(uint8_t *pBuffer);

void bootloader_send_ack(uint8_t command_code , uint8_t follow_len);
void bootloader_send_nack(void);

uint8_t bootloader_verify_crc(uint8_t *pData ,uint32_t len, uint32_t host_crc);

void bootloader_uart_write_data(uint8_t *pBuffer , uint32_t len);
uint8_t bootloader_get_version(void);

uint16_t bootloader_get_mcu_id(void);

uint8_t bootloader_get_flash_rdp_level(void);

uint8_t check_addr_validation(uint32_t addr);

void FLASH_LOCK(void);
void FLASH_UNLOCK(void);

HAL_FlashStatusTypeDef FLASH_PROGRAM_BYTE(uint32_t Address , uint8_t data);

HAL_FlashStatusTypeDef mem_write(uint8_t *pBuffer , uint32_t mem_addr , uint32_t len);

HAL_FlashStatusTypeDef Flash_sector_earse(uint32_t sector_num); 
HAL_FlashStatusTypeDef FLASH_EARSE(uint32_t sector_num , uint32_t numOfsectors);
HAL_FlashStatusTypeDef Flash_Mass_earse(void);


void FLASH_OPT_UNLOCK(void);
void FLASH_OPT_LOCK(void);

HAL_FlashStatusTypeDef FLASH_RW_PROTECTION(uint8_t sector_num , uint8_t protection_mode , uint32_t disable);

uint16_t FLASH_PROTECTION_Status(void);

uint16_t Get_WPR(void);
