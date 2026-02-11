/**
 * @file        apm32f4xx_spi_cfg.c
 *
 * @brief       This file provides configuration support for SPI
 *
 * @version     V1.0.0
 *
 * @date        2024-08-01
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
#include "apm32f4xx_spi_cfg.h"
#include "apm32f4xx_dal_gpio_ex.h"  // 包含GPIO复用功能定义

/* Private includes *******************************************************/

/* Private macro **********************************************************/

/* Private typedef ********************************************************/

/* Private variables ******************************************************/

/* Private function prototypes ********************************************/

/* External variables *****************************************************/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* External functions *****************************************************/

/**
 * @brief   检查SPI时钟是否已打开
 *
 * @param   hspi: SPI句柄指针
 *
 * @retval  1: 时钟已打开, 0: 时钟未打开
 */
uint8_t SPI_CheckClockEnabled(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL)
    {
        return 0;
    }
    
    if (hspi->Instance == SPI1)
    {
        // SPI1在APB2总线上
        return (__DAL_RCM_SPI1_IS_CLK_ENABLED() != RESET) ? 1 : 0;
    }
    else if (hspi->Instance == SPI2)
    {
        // SPI2在APB1总线上
        return (__DAL_RCM_SPI2_IS_CLK_ENABLED() != RESET) ? 1 : 0;
    }
    
    return 0;
}

/**
 * @brief   SPI1 configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_SPI1_Config(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;      // CPOL=0
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;          // CPHA=0
    
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 0;
    
    
    if (DAL_SPI_Init(&hspi1) != DAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief   SPI2 configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_SPI2_Config(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 0;

    if (DAL_SPI_Init(&hspi2) != DAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief   SPI MSP Init
 *
 * @param  hspi   Pointer to a SPI_HandleTypeDef structure that contains
 *                the configuration information for the specified SPI module
 *
 * @retval  None
 */
void DAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hspi->Instance == SPI1)
    {
        /* Enable SPI1 clock */
        __DAL_RCM_SPI1_CLK_ENABLE();

        /* Enable SPI1 GPIO clock */
        __DAL_RCM_GPIOA_CLK_ENABLE();

        /* Configure SPI1 pins: PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI) */
        /* Configure SPI1 NSS SCK MISO and MOSI pin */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			
        DAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        /* 确保SPI1使用默认引脚映射（PA4-PA7） */
        __DAL_AFIO_REMAP_SPI1_DISABLE();
    }

    if (hspi->Instance == SPI2)
    {
        /* Enable SPI2 clock */
        __DAL_RCM_SPI2_CLK_ENABLE();

        /* Enable SPI2 GPIO clock */
        __DAL_RCM_GPIOB_CLK_ENABLE();

        /* Configure SPI2 pins: PB12(NSS), PB13(SCK), PB14(MISO), PB15(MOSI) */
        /* Configure SPI2 NSS SCK MISO and MOSI pin */
        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        
        DAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
 * @brief   SPI MSP DeInit
 *
 * @param  hspi   Pointer to a SPI_HandleTypeDef structure that contains
 *                the configuration information for the specified SPI module
 * @retval  None
 */
void DAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        /* Reset SPI1 */
        __DAL_RCM_SPI1_FORCE_RESET();
        __DAL_RCM_SPI1_RELEASE_RESET();

        /* DeInit SPI1 GPIO */
        DAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    }
    else if (hspi->Instance == SPI2)
    {
        /* Reset SPI2 */
        __DAL_RCM_SPI2_FORCE_RESET();
        __DAL_RCM_SPI2_RELEASE_RESET();

        /* DeInit SPI2 GPIO */
        DAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    }
}

/**
 * @brief   Initialize all SPI Chip Select pins
 *          CS1-CS6 for SPI1, CS7-CS12 for SPI2
 *          All CS pins are on GPIOC: PC0-PC11
 *
 * @param   None
 *
 * @retval  None
 */
void SPI_CS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
        /* Enable GPIOA clock */
    __DAL_RCM_GPIOA_CLK_ENABLE();
    
    /* Configure CS1-CS12 pins (PC0-PC11) as output push-pull */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    DAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Set all CS pins to HIGH (inactive state) */
    DAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        /* Enable GPIOB clock */
    __DAL_RCM_GPIOB_CLK_ENABLE();
    
    /* Configure CS1-CS12 pins (PC0-PC11) as output push-pull */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    DAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Set all CS pins to HIGH (inactive state) */
    DAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    /* Enable GPIOC clock */
    __DAL_RCM_GPIOC_CLK_ENABLE();
    
    /* Configure CS1-CS12 pins (PC0-PC11) as output push-pull */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    DAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /* Set all CS pins to HIGH (inactive state) */
    DAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_SET);
}

/**
 * @brief   Select (activate) a specific CS pin
 *          CS1-CS6 use SPI1, CS7-CS12 use SPI2
 *
 * @param   cs_num: CS number (1-12)
 *
 * @retval  None
 */
void SPI_CS_Select(uint8_t cs_num)
{
    if (cs_num < 1 || cs_num > 12)
    {
        return;
    }
    
    /* CS pin mapping: CS1->PC0, CS2->PC1, ..., CS12->PC11 */
    uint16_t pin = (uint16_t)(1U << (cs_num - 1));
    
    /* Set CS pin to LOW (active) */
    DAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_RESET);
}

/**
 * @brief   Deselect (deactivate) a specific CS pin
 *          CS1-CS6 use SPI1, CS7-CS12 use SPI2
 *
 * @param   cs_num: CS number (1-12)
 *
 * @retval  None
 */
void SPI_CS_Deselect(uint8_t cs_num)
{
    if (cs_num < 1 || cs_num > 12)
    {
        return;
    }
    
    /* CS pin mapping: CS1->PC0, CS2->PC1, ..., CS12->PC11 */
    uint16_t pin = (uint16_t)(1U << (cs_num - 1));
    
    /* Set CS pin to HIGH (inactive) */
    DAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET);
}

/**
 * @brief   Deselect all CS pins (set all to inactive state)
 *
 * @param   None
 *
 * @retval  None
 */
void SPI_CS_DeselectAll(void)
{
    /* Set all CS pins (PC0-PC11) to HIGH (inactive) */
    DAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                             GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                             GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11, 
                      GPIO_PIN_SET);
}

/**
 * @brief   重新配置SPI1的四种SPI模式
 *          Mode 0: CPOL=0, CPHA=0 (时钟空闲低，第一个边沿采样) - 最常用
 *          Mode 1: CPOL=0, CPHA=1 (时钟空闲低，第二个边沿采样)
 *          Mode 2: CPOL=1, CPHA=0 (时钟空闲高，第一个边沿采样)
 *          Mode 3: CPOL=1, CPHA=1 (时钟空闲高，第二个边沿采样)
 *
 * @param   mode: SPI模式 (0, 1, 2, 3)
 *
 * @retval  None
 */
void DAL_SPI1_Reconfig_Mode(uint8_t mode)
{
    /* 先关闭SPI1 */
    DAL_SPI_DeInit(&hspi1);
    
    /* 根据mode参数设置CPOL和CPHA */
    switch(mode)
    {
        case 0:  // Mode 0: CPOL=0, CPHA=0 (时钟空闲低，上升沿采样，下降沿输出)
            hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
        case 1:  // Mode 1: CPOL=0, CPHA=1 (时钟空闲低，下降沿采样，上升沿输出)
            hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
            break;
        case 2:  // Mode 2: CPOL=1, CPHA=0 (时钟空闲高，下降沿采样，上升沿输出)
            hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
        case 3:  // Mode 3: CPOL=1, CPHA=1 (时钟空闲高，上升沿采样，下降沿输出)
            hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
            break;
        default:
            // 默认使用Mode 0
            hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
    }
    
    /* 保持其他配置不变 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 0;
    
    /* 重新初始化SPI1 */
    if (DAL_SPI_Init(&hspi1) != DAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief   重新配置SPI2的四种SPI模式
 *          Mode 0: CPOL=0, CPHA=0 (时钟空闲低，第一个边沿采样)
 *          Mode 1: CPOL=0, CPHA=1 (时钟空闲低，第二个边沿采样)
 *          Mode 2: CPOL=1, CPHA=0 (时钟空闲高，第一个边沿采样)
 *          Mode 3: CPOL=1, CPHA=1 (时钟空闲高，第二个边沿采样)
 *
 * @param   mode: SPI模式 (0, 1, 2, 3)
 *
 * @retval  None
 */
void DAL_SPI2_Reconfig_Mode(uint8_t mode)
{
    /* 先关闭SPI2 */
    DAL_SPI_DeInit(&hspi2);
    
    /* 根据mode参数设置CPOL和CPHA */
    switch(mode)
    {
        case 0:  // Mode 0: CPOL=0, CPHA=0
            hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
        case 1:  // Mode 1: CPOL=0, CPHA=1
            hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
            break;
        case 2:  // Mode 2: CPOL=1, CPHA=0
            hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
        case 3:  // Mode 3: CPOL=1, CPHA=1
            hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
            break;
        default:
            // 默认使用Mode 0
            hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
    }
    
    /* 保持其他配置不变 */
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 0;
    /* 重新初始化SPI2 */
    if (DAL_SPI_Init(&hspi2) != DAL_OK)
    {
        Error_Handler();
    }
}

