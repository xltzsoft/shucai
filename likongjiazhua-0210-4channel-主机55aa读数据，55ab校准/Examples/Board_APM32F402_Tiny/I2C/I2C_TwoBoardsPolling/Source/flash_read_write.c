/*!
 * @file        flash_read_write.c
 *
 * @brief       This file provides a flash read/write interface
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
#include "flash_read_write.h"

/* Private includes *******************************************************/

/* Private macro **********************************************************/

/* Private typedef ********************************************************/

/* Private variables ******************************************************/

/* Specifies the start address of the page. The purpose is to occupy a space at the specified address of MCU flash. */
#if defined (__CC_ARM)
    //const uint8_t __attribute__((section(".ARM.__at_0x0801F000"))) Flash_Para_Area[FLASH_READ_WRITE_TOTAL_SIZE];
#elif defined (__ICCARM__)
    #pragma location = 0x08004000
    __root const uint8_t Flash_Para_Area[FLASH_READ_WRITE_TOTAL_SIZE];
#elif defined (__GNUC__)

#else
    #warning Not supported compiler type
#endif

/* The buffer that write or erase page data */
static uint8_t Flash_Buffer[APM32_FLASH_PAGE_SIZE];

/* Private function prototypes ********************************************/
static uint8_t Flash_ReadByte(uint32_t readAddr);
static uint32_t Flash_GetPageNum(uint32_t Addr);
static int Flash_WriteOnePage(uint32_t writeAddr, const uint8_t *pData, uint32_t len);

/* External variables *****************************************************/

/* External functions *****************************************************/

/*!
 * @brief     Write the specified length of data from the specified address.
 *            Can be written across page.
 *
 * @param     writeAddr: write address.
 *
 * @param     pData: save the write data.
 *
 * @param     len: write data length.
 *
 * @retval    Return Success or error status. It can be one of value:
 *            @arg -1         : Write data error.
 *            @arg flashStatus: FMC program status. The value refer to FMC_STATE_T.
 *
 * @note      Address and length must be 4-bytes aligned.
 *            The example must be performed in the page 1~3.
 *
 */
int Flash_Write(uint32_t writeAddr, uint8_t *pData, uint32_t len)
{
    uint32_t numOfPage = 0, numOfByte = 0, offsetAddr = 0;
    uint32_t count = 0, temp = 0;
    int writeStatus = 0;

    /* address and len is not 4-bytes aligned */
    if ((writeAddr % 4 != 0) || (len % 4 != 0))
        return -1;

    /* offerset address in the page */
    offsetAddr = writeAddr % APM32_FLASH_PAGE_SIZE;

    /* The size of the remaining space inthe page from writeAddr */
    count = APM32_FLASH_PAGE_SIZE - offsetAddr;

    /* Calculate how many pages to write */
    numOfPage = len / APM32_FLASH_PAGE_SIZE;

    /* Calculate how many bytes are left less than one page */
    numOfByte = len % APM32_FLASH_PAGE_SIZE;

    /* offsetAddr = 0, writeAddr is page aligned */
    if (offsetAddr == 0)
    {
        /* len < APM32_FLASH_PAGE_SIZE */
        if (numOfPage == 0)
        {
            if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, len)) != DAL_OK)
            {
                return writeStatus;
            }
        }
        /* len > APM32_FLASH_PAGE_SIZE */
        else
        {
            /* write numOfPage page */
            while (numOfPage--)
            {
                if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, APM32_FLASH_PAGE_SIZE)) != DAL_OK)
                {
                    return writeStatus;
                }
                writeAddr +=  APM32_FLASH_PAGE_SIZE;
                pData += APM32_FLASH_PAGE_SIZE;
            }

            /* write remaining data */
            if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, numOfByte)) != DAL_OK)
            {
                return writeStatus;
            }
        }
    }
    /* offsetAddr != 0, writeAddr is not page aligned */
    else
    {
        /* len < APM32_FLASH_PAGE_SIZE, the data length is less than one page */
        if (numOfPage == 0)
        {
            /* numOfByte > count,  need to write across the page */
            if (numOfByte > count)
            {
                temp = numOfByte - count;
                /* fill the current page */
                if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, count)) != DAL_OK)
                {
                    return writeStatus;
                }

                writeAddr +=  count;
                pData += count;
                /* write remaining data */
                if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, temp)) != DAL_OK)
                {
                    return writeStatus;
                }
            }
            else
            {
                if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, len)) != DAL_OK)
                {
                    return writeStatus;
                }
            }
        }
        /* len > APM32_FLASH_PAGE_SIZE */
        else
        {
            len -= count;
            numOfPage = len / APM32_FLASH_PAGE_SIZE;
            numOfByte = len % APM32_FLASH_PAGE_SIZE;

            /* write count data */
            if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, count)) != DAL_OK)
            {
                return writeStatus;
            }

            writeAddr +=  count;
            pData += count;

            /* write numOfPage page */
            while (numOfPage--)
            {
                if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, APM32_FLASH_PAGE_SIZE)) != DAL_OK)
                {
                    return writeStatus;
                }
                writeAddr +=  APM32_FLASH_PAGE_SIZE;
                pData += APM32_FLASH_PAGE_SIZE;
            }

            if (numOfByte != 0)
            {
                if ((writeStatus = Flash_WriteOnePage(writeAddr, pData, numOfByte)) != DAL_OK)
                {
                    return writeStatus;
                }
            }
        }
    }

    return DAL_OK;
}

/*!
 * @brief     Read the specified length of data from the specified address.
 *
 * @param     readAddr: read address.
 *
 * @param     pData: save the read data.
 *
 * @param     len: read data length.
 *
 * @retval    Return Success or error status. It can be one of value:
 *            @arg -1 : Read data error.
 *            @arg 0  : Read data successful.
 *
 * @note      Address and length must be 4-bytes aligned.
 *            The example must be performed in the sectors 1~3.
 *
 */
int Flash_Read(uint32_t readAddr, uint8_t *pData, uint32_t len)
{
    /* illegal address direct return */
    if ((readAddr < FLASH_READ_WRITE_START_ADDR) || ((readAddr + len) > FLASH_READ_WRITE_END_ADDR))
        return -1;

    /* illegal pointer direct return */
    if (pData == NULL)
        return -1;

    /* read data */
    for (int i = 0; i < len; i++)
    {
        pData[i] = Flash_ReadByte(readAddr);
        readAddr += 1;
    }

    return 0;
}

/*!
 * @brief     flash read byte.
 *
 * @param     readAddr:  flash address.
 *
 * @retval    the data of assign address.
 */
static uint8_t Flash_ReadByte(uint32_t readAddr)
{
    return (*(__IO uint8_t *)readAddr);
}

/*!
 * @brief     Get flash page number.
 *
 * @param     Addr:  Flash address.
 *                   The value of address must be between 1~3 page.
 *
 * @retval    The page number.
 */
static uint32_t Flash_GetPageNum(uint32_t Addr)
{
    return Addr&0xFFFFFC00;
    /*
    if(Addr < ADDR_FLASH_PAGE_2)
    {
        return ADDR_FLASH_PAGE_1;
    }
    else if(Addr < ADDR_FLASH_PAGE_3)
    {
        return ADDR_FLASH_PAGE_2;
    }
    else if(Addr < ADDR_FLASH_PAGE_4)
    {
        return ADDR_FLASH_PAGE_3;
    }
    else 
    {
        return ADDR_FLASH_PAGE_4;
    }
    */
}

/*!
 * @brief     In a page, write the specified length of data from the specified address.
 *
 * @param     writeAddr: write address.
 *
 * @param     pData: save the write data.
 *
 * @param     len: write data length.
 *
 * @retval    Return Success or error status. It can be one of value:
 *            @arg -1         : Write data error.
 *            @arg flashStatus: FMC program status. The value refer to FMC_STATE_T.
 *
 * @note      Address and length must be 4-bytes aligned.
 *            The example must be performed in the sectors 1~3.
 *
 */
static int Flash_WriteOnePage(uint32_t writeAddr, const uint8_t *pData, uint32_t len)
{
    uint32_t isErase = 0;
    uint32_t startAddr;
    uint32_t offsetAddr;
    uint32_t i = 0;
    uint32_t sectorError = 0U;
    uint8_t *pTemp = Flash_Buffer;
    DAL_StatusTypeDef flashStatus = DAL_OK;
    FLASH_EraseInitTypeDef Erase_InitStruct = {0};

    startAddr = writeAddr / APM32_FLASH_PAGE_SIZE * APM32_FLASH_PAGE_SIZE;
    offsetAddr = writeAddr % APM32_FLASH_PAGE_SIZE;

    /* illegal address direct return */
    if ((writeAddr < FLASH_READ_WRITE_START_ADDR) || ((writeAddr + len) > FLASH_READ_WRITE_END_ADDR))
        return -1;

    /* illegal pointer direct return */
    if (pData == NULL)
        return -1;

    /* unlock flash for erase or write*/
    DAL_FLASH_Unlock();

    /* check whether the page need to be erased */
    for (i = 0; i < len; i++)
    {
        if (Flash_ReadByte(writeAddr + i) != 0xFF)
        {
            isErase = 1;
            break;
        }
    }

    /* the page needs to be erase */
    if (isErase == 1)
    {
        /* read the entire page data to the buffer before write or erase */
        Flash_Read(startAddr, Flash_Buffer, APM32_FLASH_PAGE_SIZE);

        /* copy the data to the buffer */
        for (i = 0; i < len; i++)
        {
            Flash_Buffer[offsetAddr + i] = pData[i];
        }

        /* FLASH Erase structure init */
        Erase_InitStruct.PageAddress    = Flash_GetPageNum(writeAddr);
        Erase_InitStruct.NbPages        = 1U;
        Erase_InitStruct.TypeErase      = FLASH_TYPEERASE_PAGES;

        /* erase the page where the address is located */
        if ((flashStatus = DAL_FLASHEx_Erase(&Erase_InitStruct, &sectorError)) != DAL_OK)
        {
            return flashStatus;
        }

        /* write the entire page data */
        for (i = 0; i < APM32_FLASH_PAGE_SIZE / 4; i++)
        {
            if ((flashStatus = DAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddr, *(uint32_t *)pTemp)) != DAL_OK)
            {
                return flashStatus;
            }
            startAddr += 4;
            pTemp += 4;
        }
    }
    /* the page don't need to be erase */
    else
    {
        /* write n bytes of data to the page */
        for (i = 0; i < len; i += 4)
        {
            if ((flashStatus = DAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, writeAddr, *(uint32_t *)pData)) != DAL_OK)
            {
                return flashStatus;
            }
            writeAddr += 4;
            pData += 4;
        }
    }

    /* lock flash */
    DAL_FLASH_Lock();

    return DAL_OK;
}
