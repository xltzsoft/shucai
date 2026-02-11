/**
 * @file        apm32f4xx_spi_cfg.h
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

/* Define to prevent recursive inclusion */
#include "main.h"


#ifndef APM32F4XX_SPI_CFG_H
#define APM32F4XX_SPI_CFG_H


/* SPI Chip Select functions */
void SPI_CS_Init(void);
void SPI_CS_Select(uint8_t cs_num);
void SPI_CS_Deselect(uint8_t cs_num);
void SPI_CS_DeselectAll(void);

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ***************************************************************/
#include "apm32f4xx_dal.h"

/* Exported macro *********************************************************/

/* Exported typedef *******************************************************/

/* Exported function prototypes *******************************************/
void DAL_SPI1_Config(void);
void DAL_SPI2_Config(void);
uint8_t SPI_CheckClockEnabled(SPI_HandleTypeDef *hspi);
void DAL_SPI1_Reconfig_Mode(uint8_t mode);  // 重新配置SPI1模式 (0,1,2,3)
void DAL_SPI2_Reconfig_Mode(uint8_t mode);  // 重新配置SPI2模式 (0,1,2,3)

#ifdef __cplusplus
}
#endif

#endif /* APM32F4XX_SPI_CFG_H */
