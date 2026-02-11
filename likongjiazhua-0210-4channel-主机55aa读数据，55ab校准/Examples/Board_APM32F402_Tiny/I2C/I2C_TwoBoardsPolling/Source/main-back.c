/**
 * @file        main.c
 *
 * @brief       Main program body
 *
 * @version     V1.0.0
 *
 * @date        2024-12-01
 *
 * @attention
 *
 *  Copyright (C) 2024-2025 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Includes ***************************************************************/
#include "main.h"

/* Private includes *******************************************************/
#include "apm32f4xx_device_cfg.h"
#include "log.h"
#include <string.h>
#include <stdbool.h>

/* Private macro **********************************************************/

/* Define firmware to switch between Master board or Slave board */
//#define IS_MASTER_BOARD

/* Private typedef ********************************************************/

/* Private variables ******************************************************/
static char uart_dma_tx_buffer[76] = {0};
#define pressor_base  43
#define temp_base     13
uint32_t seq = 0;

/* External variables *****************************************************/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static volatile uint32_t uartEventFlag = 0U;

int16_t temperature[8] = {0};
int32_t pressor[8] = {0};

#define send_485_enable() \
    do { \
        DAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); \
        DAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); \
    } while (0)

#define recv_485_enable() \
    do { \
        DAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); \
        DAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); \
    } while (0)

int32_t process_24bit_signed(const uint8_t data[3]) {
    // 先将24位数据存储在32位整数的低24位
    int32_t value = 0;
		
		value = (data[1] << 8)|data[2];
		
    value |= ((int32_t)data[0] << 16);
    // 如果符号位为1（负数），需要扩展符号位到32位
    if (value & 0x00800000) {
        value |= 0xFF000000; // 设置高8位为1
    } else {
        value &= 0x00FFFFFF; // 确保高8位为0
    }
    
    return value;
}
/* External functions *****************************************************/

//I2C读函数
uint8_t i2c_ReadReg(uint8_t dev_addr, uint8_t *buf, uint8_t len, uint8_t reg)
{ 
    DAL_I2C_Master_Transmit(&hi2c1, (uint16_t)dev_addr<<1, (uint8_t*)&reg, 1, 1000U);
    DAL_I2C_Master_Receive(&hi2c1, (uint16_t)dev_addr<<1, (uint8_t *)buf, len, 1000U);
}
//I2C写函数
uint8_t i2c_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t buf)
{
    uint8_t data[2];
    data[0] = reg; // 寄存器地址
    data[1] = buf; // 要写入的数据
    return DAL_I2C_Master_Transmit(&hi2c1, (uint16_t)dev_addr<<1, data, sizeof(data), 1000U);
}

uint8_t NSA2302_READ(void)
{
	uint8_t addr,i;
    uint8_t conversion_complete_flag = 0x0a;
    uint8_t buf[5] = {0};
	#if 1
    for (addr = 0; addr < 3; addr++)
    {
		i2c_WriteReg(addr+1, 0x30, conversion_complete_flag);
	}
    send_485_enable();
    if (DAL_UART_Transmit(&huart1, (uint8_t *)uart_dma_tx_buffer, sizeof(uart_dma_tx_buffer), 100U) != DAL_OK)
    {
        Error_Handler();
    }
    
    recv_485_enable();
    #endif



    for (addr = 0; addr < 3; addr++)
    {
        #if 1
        do
        {
            //i2c_ReadReg(addr+1, &conversion_complete_flag, 0x30);
            i2c_ReadReg(addr+1, 0x30, &conversion_complete_flag, 0x1);
        }while(conversion_complete_flag == 0x0a);
        conversion_complete_flag = 0x0a;
        #endif
        //conversion_complete_flag的bit3应该变为0，则转换完成了
        //i2c_ReadReg_Multi(addr+1, &buf[0], 5, 0x6);
        //i2c_ReadReg_Multi(addr+1, &buf[0], 5, 0x6);
        i2c_ReadReg(addr+1, &buf[0], 5, 0x6);
			            
        pressor[addr] = (int)((float)process_24bit_signed(buf) * 0.0119209f);//除以2的23次方，再乘以1000，取三位小数
        uart_dma_tx_buffer[0 + (addr*6)] = (uint8_t)( (pressor[addr] >> 0  )& 0xff); //低位
        uart_dma_tx_buffer[1 + (addr*6)] = (uint8_t)( (pressor[addr] >> 8  )& 0xff); //高位
        uart_dma_tx_buffer[2 + (addr*6)] = (uint8_t)( (pressor[addr] >> 16 )& 0xff); //高位
        uart_dma_tx_buffer[3 + (addr*6)] = (uint8_t)( (pressor[addr] >> 24 )& 0xff); //高位
        temperature[addr] = ( (int32_t)((buf[3] << 8) | buf[4]) * 10) >> 8; //除以2的8次方，再乘以10，取一位小数
        uart_dma_tx_buffer[4 + (addr*6)] = (uint8_t)(temperature[addr] & 0x00ff); //低位
        uart_dma_tx_buffer[5 + (addr*6)] = (uint8_t)(temperature[addr] >> 8); //高位
    }
    	//*(uint32_t*)(uart_dma_tx_buffer+9) = seq++;
	//*(uint32_t*)(uart_dma_tx_buffer+39) = seq++;
	uart_dma_tx_buffer[9]  = (seq >> 0  )& 0xff;
	uart_dma_tx_buffer[10] = (seq >> 8  )& 0xff;
	uart_dma_tx_buffer[11] = (seq >> 16 )& 0xff;
	uart_dma_tx_buffer[12] = (seq >> 24 )& 0xff;
	seq++;
	uart_dma_tx_buffer[39] = (seq >> 0  )& 0xff;
	uart_dma_tx_buffer[40] = (seq >> 8  )& 0xff;
	uart_dma_tx_buffer[41] = (seq >> 16 )& 0xff;
	uart_dma_tx_buffer[42] = (seq >> 24 )& 0xff;
	seq++;
	uart_dma_tx_buffer[29] = 0;
	for(i = 2; i < 29; i++)
	{
		uart_dma_tx_buffer[29] += uart_dma_tx_buffer[i];
	}
	uart_dma_tx_buffer[75] = 0;
	for(i = 32; i < 75; i++)
	{
		uart_dma_tx_buffer[75] += uart_dma_tx_buffer[i];
	}
	
    return 0;
}

/**
 * @brief   Main program
 *
 * @param   None
 *
 * @retval  None
 */
int main(void)
{
    uint8_t addr;
    uint8_t conversion_complete_flag = 0x0a;

    /* Device configuration */
    DAL_DeviceConfig();

	uart_dma_tx_buffer[0] = 0x42;
	uart_dma_tx_buffer[1] = 0x54;
	uart_dma_tx_buffer[30] = 0x42;
	uart_dma_tx_buffer[31] = 0x54;
	
	uart_dma_tx_buffer[2] = 30;
	uart_dma_tx_buffer[3] = 0;
	uart_dma_tx_buffer[32] = 46;
	uart_dma_tx_buffer[33] = 0;
	
	uart_dma_tx_buffer[4] = 0xf4;
	uart_dma_tx_buffer[34] = 0xf5;
	
	uart_dma_tx_buffer[5] = 0;
	uart_dma_tx_buffer[6] = 0;
	uart_dma_tx_buffer[7] = 0;
	uart_dma_tx_buffer[8] = 0;
	
	uart_dma_tx_buffer[35] = 0;
	uart_dma_tx_buffer[36] = 0;
	uart_dma_tx_buffer[37] = 0;
	uart_dma_tx_buffer[38] = 0;
#if 0
    for (addr = 0; addr < 3; addr++)
    {
        i2c_WriteReg(addr+1, 0xa5, 0x10);
        i2c_WriteReg(addr+1, 0xa6, 0x4b);
        i2c_WriteReg(addr+1, 0xa7, 0x83);
        
		i2c_WriteReg(addr+1, 0x6a, 0x40);
		i2c_WriteReg(addr+1, 0x6c, 0x6a);
    }

    DAL_Delay(1000); // 延时1秒，等待EEPROM写入
    #endif
    for (addr = 0; addr < 3; addr++)
    {
		i2c_ReadReg(addr+1, &conversion_complete_flag, 1, 0xA5);
		i2c_ReadReg(addr+1, &conversion_complete_flag, 1, 0xA6);
		i2c_ReadReg(addr+1, &conversion_complete_flag, 1, 0xA7);
	}
    #if 0
    if (DAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), 1000U) != DAL_OK)
    {
        Error_Handler();
    }
    
	if (DAL_UART_Transmit_DMA(&huart1, (uint8_t *)txBuffer, strlen(txBuffer)) != DAL_OK)
    {
        Error_Handler();
    }
    if (DAL_UART_Transmit_DMA(&huart2, (uint8_t *)txBuffer, strlen(txBuffer)) != DAL_OK)
    {
        Error_Handler();
    }
    if (DAL_UART_Transmit_DMA(&huart1, (uint8_t *)txBuffer, strlen(txBuffer)) != DAL_OK)
    {
        Error_Handler();
    }
    #endif
    /* Infinite loop */
    while (1)
    {
        NSA2302_READ();
    }
}
#if 1
/**
 * @brief  Tx Transfer completed callbacks
 *
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module
 *
 * @retval None
 */
void DAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    //uartEventFlag = 1U;
}

/**
 * @brief  Rx Transfer completed callbacks
 *
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module
 *
 * @retval None
 */
void DAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //uartEventFlag = 1U;
}
/**
 * @brief  UART error callbacks
 *
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module
 *
 * @retval None
 */
void DAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    //BOARD_LED_On(LED3);
}
#endif
