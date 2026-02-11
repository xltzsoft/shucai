/**
 * @file        apm32f4xx_gpio_cfg.c
 *
 * @brief       This file provides configuration support for GPIO
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
#include "apm32f4xx_gpio_cfg.h"

/* Private includes *******************************************************/

/* Private macro **********************************************************/

/* Private typedef ********************************************************/

/* Private variables ******************************************************/

/* Private function prototypes ********************************************/

/* External variables *****************************************************/

/* External functions *****************************************************/

/**
 * @brief   GPIO configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0U};
    /* Enable USART1 GPIO clock */
    __DAL_RCM_GPIOA_CLK_ENABLE();
    /* Configure the 485DE pin */
    GPIO_InitStruct.Pin     = GPIO_PIN_0;
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FAST;

    DAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    DAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    DAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    /* Configure the 485RE pin */
    GPIO_InitStruct.Pin     = GPIO_PIN_1;
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FAST;

    DAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    DAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    DAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    
    __DAL_RCM_GPIOD_CLK_ENABLE();
    /* Configure the LED pin */
    GPIO_InitStruct.Pin     = GPIO_PIN_2;
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FAST;

    DAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //ADDR1
    DAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    DAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    //ADDR2
    DAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    DAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    //ADDR3
    DAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    DAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

    /* 初始化SPI片选引脚 CS1-CS12 (PC0-PC11) */
    __DAL_RCM_GPIOC_CLK_ENABLE();
    
    /* 配置PC0-PC11为输出推挽模式 */
    GPIO_InitStruct.Pin     = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                              GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                              GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_HIGH;
    
    DAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /* 将所有CS引脚设置为高电平(未选中状态) */
    DAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                             GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                             GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11, 
                      GPIO_PIN_SET);
}
