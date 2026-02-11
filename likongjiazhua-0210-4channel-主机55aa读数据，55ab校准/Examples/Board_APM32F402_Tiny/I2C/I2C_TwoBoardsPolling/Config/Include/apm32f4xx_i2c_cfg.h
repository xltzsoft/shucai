/**
 * @file        apm32f4xx_i2c_cfg.h
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

/* Define to prevent recursive inclusion */
#ifndef APM32F4XX_I2C_CFG_H
#define APM32F4XX_I2C_CFG_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ***************************************************************/
#include "apm32f4xx_dal.h"

/* Exported macro *********************************************************/
#define I2C_SALVE_ADDRESS           0xA0U

/* Exported typedef *******************************************************/

/* Exported function prototypes *******************************************/
void DAL_I2C1_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* APM32F4XX_I2C_CFG_H */
