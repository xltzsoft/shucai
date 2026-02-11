/**
 * @file        apm32f4xx_i2c_cfg.c
 *
 * @brief       This file provides configuration support for I2C
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
#include "apm32f4xx_i2c_cfg.h"

/* Private includes *******************************************************/

/* Private macro **********************************************************/

/* Private typedef ********************************************************/

/* Private variables ******************************************************/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;
/* Private function prototypes ********************************************/

/* External variables *****************************************************/

/* External functions *****************************************************/

/**
 * @brief  Initialize the I2C MSP
 *
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *         the configuration information for the specified I2C
 *
 * @retval None
 */
void DAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0U};

    if(hi2c->Instance == I2C1)
    {
        /* Enable peripheral clock */
        __DAL_RCM_I2C1_CLK_ENABLE();

        /* Enable peripheral GPIO clock */
        __DAL_RCM_GPIOB_CLK_ENABLE();

        /* Enable SYSCFG clock for pin remap */
        //__DAL_RCM_AFIO_CLK_ENABLE();

        /* Remap I2C1 to PB8/PB9 */
        __DAL_AFIO_REMAP_I2C1_ENABLE();

        /* Enable DMA1 clock */
        __DAL_RCM_DMA1_CLK_ENABLE();

        /* I2C1 pin configuration */
        GPIO_InitStruct.Pin         = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull        = GPIO_PULLUP;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FAST;
        DAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

                /* Configure USART2 TX DMA */
        hdma_i2c1_tx.Instance                  = DMA1_Channel6;
        hdma_i2c1_tx.Init.Direction            = DMA_MEMORY_TO_PERIPH;
        hdma_i2c1_tx.Init.PeriphInc            = DMA_PINC_DISABLE;
        hdma_i2c1_tx.Init.MemInc               = DMA_MINC_ENABLE;
        hdma_i2c1_tx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
        hdma_i2c1_tx.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
        hdma_i2c1_tx.Init.Mode                 = DMA_NORMAL;
        hdma_i2c1_tx.Init.Priority             = DMA_PRIORITY_HIGH;
        if (DAL_DMA_Init(&hdma_i2c1_tx) != DAL_OK)
        {
            Error_Handler();
        }



        /* Configure USART2 RX DMA */
        hdma_i2c1_rx.Instance                  = DMA1_Channel7;
        hdma_i2c1_rx.Init.Direction            = DMA_PERIPH_TO_MEMORY;
        hdma_i2c1_rx.Init.PeriphInc            = DMA_PINC_DISABLE;
        hdma_i2c1_rx.Init.MemInc               = DMA_MINC_ENABLE;
        hdma_i2c1_rx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
        hdma_i2c1_rx.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
        hdma_i2c1_rx.Init.Mode                 = DMA_NORMAL;
        hdma_i2c1_rx.Init.Priority             = DMA_PRIORITY_HIGH;
        if (DAL_DMA_Init(&hdma_i2c1_rx) != DAL_OK)
        {
            Error_Handler();
        }

        /* Link DMA handle */
        __DAL_LINKDMA(hi2c, hdmatx, hdma_i2c1_tx);
        /* Link DMA handle */
        __DAL_LINKDMA(hi2c, hdmarx, hdma_i2c1_rx);
        /* USART2 TX DMA interrupt */
        DAL_NVIC_SetPriority(I2C1_EV_IRQn, 1U, 0U);
        DAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        /* USART2 TX DMA interrupt */
        DAL_NVIC_SetPriority(I2C1_ER_IRQn, 1U, 1U);
        DAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
        /* USART2 TX DMA interrupt */
        DAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2U, 0U);
        DAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
        /* USART2 RX DMA interrupt */
        DAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 2U, 0U);
        DAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    }
}

/**
 * @brief  DeInitialize the I2C MSP
 *
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *         the configuration information for the specified I2C
 *
 * @retval None
 */
void DAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C1)
    {
        /* Disable peripheral clock */
        __DAL_RCM_I2C1_CLK_DISABLE();

        /* I2C GPIO configuration */
        DAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
    }
}

/**
 * @brief   I2C1 configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_I2C1_Config(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.ClockSpeed       = 1000000U;
    hi2c1.Init.DutyCycle        = I2C_DUTYCYCLE_16_9;
    hi2c1.Init.OwnAddress1      = I2C_SALVE_ADDRESS;
    hi2c1.Init.OwnAddress2      = 0xFEU;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (DAL_I2C_Init(&hi2c1) != DAL_OK)
    {
        Error_Handler();
    }
}
