/**
 * @file        apm32f4xx_int.c
 *
 * @brief       Main interrupt service routines
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
#include "apm32f4xx_int.h"

/* Private includes *******************************************************/

/* Private macro **********************************************************/

/* Private typedef ********************************************************/

/* Private variables ******************************************************/

/* Private function prototypes ********************************************/

/* External variables *****************************************************/
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
/* External functions *****************************************************/

/**
 * @brief     This function handles NMI exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void NMI_Handler(void)
{
}

/**
 * @brief     This function handles Hard Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief     This function handles Memory Manage exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @brief     This function handles Bus Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief     This function handles Usage Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief     This function handles SVCall exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void SVC_Handler(void)
{
}

/**
 * @brief     This function handles Debug Monitor exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief     This function handles PendSV_Handler exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void PendSV_Handler(void)
{
}

/**
 * @brief     This function handles SysTick request
 *
 * @param     None
 *
 * @retval    None
 *
 */
void SysTick_Handler(void)
{
    DAL_IncTick();
}
/**
 * @brief     This function handles DMA1 channel 5 request
 *
 * @param     None
 *
 * @retval    None
 *
 */
void DMA1_Channel6_IRQHandler(void)
{
    DAL_DMA_IRQHandler(&hdma_i2c1_tx);
    __DAL_DMA_DISABLE(&hdma_i2c1_tx);
}

/**
 * @brief     This function handles DMA1 channel 7 request
 *
 * @param     None
 *
 * @retval    None
 *
 */
void DMA1_Channel7_IRQHandler(void)
{
    DAL_DMA_IRQHandler(&hdma_i2c1_rx);
}
/**
 * @brief     This function handles USART1 request
 *
 * @param     None
 *
 * @retval    None
 *
 */
void USART2_IRQHandler(void)
{
    DAL_UART_IRQHandler(&huart2);
}
void USART1_IRQHandler(void)
{
    DAL_UART_IRQHandler(&huart1);
}
