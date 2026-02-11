/**
 * @file        readme.txt
 *
 * @brief       This file is routine instruction
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

&par Example Description

This example shows how to use I2C1 to communicate between Master board and Slave board by polling.
Master board uses I2C1 to send a string and then enters the data waiting for Slave board to return data, 
while Slave board uses I2C1 to receive the string of Master board, and then sends back the data. 
If the data received and sent are inconsistent, the LED3 of Master board and Slave board will turn off, 
otherwise LED3 will turn on.

note:The two boards share a firmware project, and you can use "IS_MASTER_BOARD" to switch different 
firmware to fit the two boards.

&par Hardware Description

    1st board            2nd board
  I2C1_SCL(PB6) ----> I2C1_SCL(PB6)
  I2C1_SDA(PB7) ----> I2C1_SDA(PB7)
  1st board GND ----> 2nd board GND

  - USART1 configured as follow:
  - BaudRate = 115200
  - Word Length = USART_WordLength_8b
  - Stop Bit = USART_StopBits_1
  - Parity = USART_Parity_No
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled

&par Directory contents

  - I2C/I2C_TwoBoardsPolling/Source/main.c                  Main program
  - I2C/I2C_TwoBoardsPolling/Source/apm32f4xx_int.c         Interrupt handlers

&par IDE environment

  - MDK-ARM V5.36
  - EWARM V8.50.5.26295
  - ECLIPSE-EMB V4.24.0

&par Hardware and Software environment

  - This example runs on APM32F402 TINY Devices.
