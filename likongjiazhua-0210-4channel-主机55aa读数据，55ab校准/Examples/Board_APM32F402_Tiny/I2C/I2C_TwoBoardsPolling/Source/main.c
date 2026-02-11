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
#include "apm32f4xx_spi_cfg.h"
#include "apm32f4xx_i2c_cfg.h"
#include "log.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "flash_read_write.h"
#include "math.h"

/* Private macro **********************************************************/

/* Define firmware to switch between Master board or Slave board */
//#define IS_MASTER_BOARD

/* Private typedef ********************************************************/

/* Private variables ******************************************************/
static char uart_dma_tx_buffer[200] = {0};
static char temp_buffer[100] = {0};
static char pres_buffer[100] = {0};
static char uart_dma_tx_buffer_send[100] = {0};

static uint8_t uart_dma_rx_buffer[100] = {0};
uint32_t seq = 0;

/* External variables *****************************************************/
extern UART_HandleTypeDef huart2;

static volatile uint32_t uartEventFlag = 0U;

#define NSA_NUMBER 4

int16_t temperature[NSA_NUMBER] = {0};
float pressor[NSA_NUMBER] = {0};

float pressor_cali[NSA_NUMBER] = {0};
float pressor_real[NSA_NUMBER] = {0};

/* External variables *****************************************************/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;

/* ================= I2C1 Slave (Addr 0x2C) command/response =================
 * Protocol:
 *   Master writes 2 bytes: 0x55 0xAA
 *   After receiving 0x55 0xAA, Slave arms a TX buffer.
 *   Master then performs a READ to fetch the prepared data.
 */
#define I2C_SLAVE_ADDR_7BIT      (0x2CU)
static uint8_t  i2c_rx_cmd[2]   = {0};
static uint8_t  i2c_tx_buf[200] = {0};
static uint16_t i2c_tx_len      = 0;
static volatile uint8_t i2c_ready_to_tx = 0;
/* ========================================================================== */

uint8_t rs485_addr_flag = 0;
uint8_t rs485_temp_cali_reg_flag = 0;
uint8_t rs485_temp_read_cali_reg_flag = 0;
uint8_t rs485_temp_collector_flag = 0;
uint8_t uart_dma_flag = 0;
/**
 * @brief   Main program
 *
 * @param   None
 *
 * @retval  None
 */
 //MCU��ַ
 #define MCU_ADDR (*((uint8_t*)0x8004C00))
 //У׼��־λ
 #define FLAG_CALI (*((uint8_t*)0x8004C01))
#define FLASH_USER_ADDR     (FLASH_READ_WRITE_START_ADDR + APM32_FLASH_PAGE_SIZE)
static uint8_t Write_Buffer[4] = {0};


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
    
    // ����SPI���ţ�ʹ��PA5-SCK, PA6-MISO, PA7-MOSI��
// NSS��CS��ʹ���û��Զ��壬���� PB0
#define NSA2302_CS1_PORT     GPIOA
#define NSA2302_CS1_PIN      GPIO_PIN_4
#define NSA2302_CS2_PORT     GPIOC
#define NSA2302_CS2_PIN      GPIO_PIN_4
#define NSA2302_CS3_PORT     GPIOB
#define NSA2302_CS3_PIN      GPIO_PIN_12
#define NSA2302_CS4_PORT     GPIOC
#define NSA2302_CS4_PIN      GPIO_PIN_6
#define NSA2302_CS5_PORT     GPIOC
#define NSA2302_CS5_PIN      GPIO_PIN_4
#define NSA2302_CS6_PORT     GPIOC
#define NSA2302_CS6_PIN      GPIO_PIN_5
#define NSA2302_CS7_PORT     GPIOC
#define NSA2302_CS7_PIN      GPIO_PIN_6
#define NSA2302_CS8_PORT     GPIOC
#define NSA2302_CS8_PIN      GPIO_PIN_7
#define NSA2302_CS9_PORT     GPIOC
#define NSA2302_CS9_PIN      GPIO_PIN_8
#define NSA2302_CS10_PORT     GPIOC
#define NSA2302_CS10_PIN      GPIO_PIN_9
#define NSA2302_CS11_PORT     GPIOC
#define NSA2302_CS11_PIN      GPIO_PIN_10
#define NSA2302_CS12_PORT     GPIOC
#define NSA2302_CS12_PIN      GPIO_PIN_11

GPIO_TypeDef *NSA2302_CS_PORT[NSA_NUMBER] = {
    NSA2302_CS1_PORT, NSA2302_CS2_PORT, NSA2302_CS3_PORT, NSA2302_CS4_PORT
};
uint16_t NSA2302_CS_PIN[NSA_NUMBER] = {
    NSA2302_CS1_PIN, NSA2302_CS2_PIN, NSA2302_CS3_PIN, NSA2302_CS4_PIN
};

//余弦相似度算法



// 定义常量
#define ESP 1e-9f
#define NEAREST_COUNT 4
#define MAX_SAMPLES 70  // 根据实际情况调整

// 结构体定义
typedef struct {
    float x;
    float y;
    float e1;
    float e2;
    float e3;
    float e4;
} StandardSample;

typedef struct {
    float e1;
    float e2;
    float e3;
    float e4;
} InputSample;

typedef struct {
    float x;
    float y;
    float F;
} PredictionResult;

// 全局变量声明
StandardSample ss[70] = {
{1,	1,	108166,   205535,  107997,  16836},
{1,	2,	180651,   272721,  190027,  21723},
{1,	3,	305358,   292913,  342533,  25478},
{1,	4,	429054,   291415,  524259,  32800},
{1,	5,	504261,   239403,  704443,  29842},
{1,	6,	556881,   229641,  881023,  37836},
{1,	7,	603911,   219397,  1071304,  53620},
{1,	8,	631507,   208076,  1130396,  72503},
{1,	9,	625522,   187994,  1022245,  88009},
{1,	10,	541254,   140690,  768140,  84156},
{1,	11,	467164,   115133,  575142,  98630},
{1,	12,	356006,   87138,  393364,  111178},
{1,	13,	231682,   61943,  236063,  160963},
{1,	14,	136940,   44351,  130489,  205465},
{2,	1,	109454,   632244,  107582,  45794},
{2,	2,	212481,   553523,  221683,  44901},
{2,	3,	368332,   600324,  401107,  59709},
{2,	4,	478666,   521600,  545605,  65641},
{2,	5,	550597,   374020,  699864,  63886},
{2,	6,	614809,   322535,  846690,  79143},
{2,	7,	674712,   272501,  1006810,  101291},
{2,	8,	700244,   242809,  1061785,  120855},
{2,	9,	713243,   223638,  996707,  148874},
{2,	10,	694250,   201746,  824094,  210599},
{2,	11,	573409,   157177,  616482,  260343},
{2,	12,	424234,   121171,  433414,  287402},
{2,	13,	247289,   85181,  237321,  353842},
{2,	14,	139996,   64385,  127512,  377710},
{3,	1,	132372,   909773,  134146,  56113},
{3,	2,	253569,   873169,  259467,  67400},
{3,	3,	412392,   795640,  425514,  78169},
{3,	4,	488928,   643585,  509750,  80792},
{3,	5,	597083,   470041,  646062,  87475},
{3,	6,	767520,   333152,  823653,  128341},
{3,	7,	816788,   276782,  885211,  164257},
{3,	8,	841088,   254738,  839646,  212718},
{3,	9,	755936,   215831,  720079,  288793},
{3,	10,	641594,   182272,  597771,  351503},
{3,	11,	495837,   151483,  460528,  418638},
{3,	12,	307356,   116647,  281001,  505848},
{3,	13,	172950,   90420,  155985,  540273},
{3,	14,	71912,   73980,  61790,  571050},
{4,	1,	85228,   535060,  91887,  37240},
{4,	2,	197425,   519536,  192354,  45054},
{4,	3,	397426,   553100,  363744,  64830},
{4,	4,	690370,   373969,  520393,  80757},
{4,	5,	891430,   317205,  573859,  95127},
{4,	6,	1054541,   291005,  624758,  118100},
{4,	7,	1134827,   266319,  653392,  143451},
{4,	8,	1062933,   243537,  652691,  168428},
{4,	9,	831384,   195343,  572430,  195033},
{4,	10,	645311,   162420,  488208,  217660},
{4,	11,	446805,   125221,  364831,  241762},
{4,	12,	867772,   201677,  585853,  190356},
{4,	13,	261029,   87400,  226411,  246574},
{4,	14,	54373,   64006,  57206,  362046},
{5,	1,	82241,   295558,  85968,  23885},
{5,	2,	206505,   240417,  190964,  30519},
{5,	3,	329874,   232451,  283009,  40773},
{5,	4,	433565,   156554,  321289,  38911},
{5,	5,	581327,   168049,  374077,  46903},
{5,	6,	733838,   165908,  392317,  52353},
{5,	7,	875214,   153943,  389980,  55909},
{5,	8,	1011798,   156801,  424880,  68065},
{5,	9,	896605,   142717,  420017,  70801},
{5,	10,	683838,   126185,  394011,  70386},
{5,	11,	499333,   104468,  339004,  60722},
{5,	12,	372232,   84101,  280621,  47251},
{5,	13,	209298,   54713,  173700,  40118},
{5,	14,	102357,   37498,  90230,  51055},

};  // 标准样本数组
int ss_count = 70;  // 标准样本数量
InputSample s[1];  // 输入样本数组
int s_count = 1;  // 输入样本数量
PredictionResult results[1];  // 预测结果

// 函数声明
float vector_norm2(const float* vec, int size);
float dot_product(const float* a, const float* b, int size);
int find_nearest_samples(const InputSample* input, int* nearest_idx);
void predict_position_and_F(const InputSample* input, const int* nearest_idx, 
                           float* x, float* y, float* F);


void cos_calculate()
{
            int nearest_idx[NEAREST_COUNT];
            float predicted_x, predicted_y, predicted_F;
            
            // 1. 找到最近的4个样本
            int valid_count = find_nearest_samples(&s[0], nearest_idx);
            
            if (valid_count >= NEAREST_COUNT) {
                // 2. 预测位置和F值
                predict_position_and_F(&s[0], nearest_idx, 
                                      &predicted_x, &predicted_y, &predicted_F);
                
                // 存储结果
                results[0].x = predicted_x;
                results[0].y = predicted_y;
							if(predicted_F > results[0].F)
							{
								if((predicted_F - results[0].F) < 50)
									results[0].F = predicted_F;
							}
							if(results[0].F > predicted_F)
							{
								if((results[0].F - predicted_F) < 50)
									results[0].F = predicted_F;
							}
                // 输出结果（通过串口或其他方式）
                // printf("Sample %d: x=%.4f, y=%.4f, F=%.4f\n", 
                //        i, predicted_x, predicted_y, predicted_F);
            }
}


// 计算向量的2-范数
float vector_norm2(const float* vec, int size) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += vec[i] * vec[i];
    }
    return sqrtf(sum);
}

// 计算点积
float dot_product(const float* a, const float* b, int size) {
    float result = 0.0f;
    for (int i = 0; i < size; i++) {
        result += a[i] * b[i];
    }
    return result;
}

// 找到最近的4个样本
int find_nearest_samples(const InputSample* input, int* nearest_idx) {
    float d[MAX_SAMPLES];
    float input_vec[4] = {input->e1, input->e2, input->e3, input->e4};
    
    // 计算与所有标准样本的共线程度
    for (int l = 0; l < ss_count; l++) {
        float standard_vec[4] = {ss[l].e1, ss[l].e2, ss[l].e3, ss[l].e4};
        float dot = dot_product(input_vec, standard_vec, 4);
        float norm_a = vector_norm2(input_vec, 4);
        float norm_c = vector_norm2(standard_vec, 4);
        
        d[l] = dot / (norm_a * norm_c + ESP);
    }
    
    // 简单排序找到最大的4个值
    for (int i = 0; i < NEAREST_COUNT; i++) {
        int max_idx = i;
        for (int j = i + 1; j < ss_count; j++) {
            if (d[j] > d[max_idx]) {
                max_idx = j;
            }
        }
        
        nearest_idx[i] = max_idx;
        
        // 将找到的最大值设为负无穷，避免重复选择
        d[max_idx] = -INFINITY;
    }
    
    return NEAREST_COUNT;
}

// 预测位置和F值
void predict_position_and_F(const InputSample* input, const int* nearest_idx,
                           float* x, float* y, float* F) {
    float w[NEAREST_COUNT];
    float d_vals[NEAREST_COUNT];
    float input_vec[4] = {input->e1, input->e2, input->e3, input->e4};
    float norm_a = vector_norm2(input_vec, 4);
    float w_sum = 0.0f;
    float F_sum = 0.0f;
    
    // 1. 计算权重
    for (int i = 0; i < NEAREST_COUNT; i++) {
        int idx = nearest_idx[i];
        float standard_vec[4] = {ss[idx].e1, ss[idx].e2, ss[idx].e3, ss[idx].e4};
        float dot = dot_product(input_vec, standard_vec, 4);
        float norm_c = vector_norm2(standard_vec, 4);
        
        d_vals[i] = dot / (norm_a * norm_c + ESP);
        w[i] = 1.0f / (ESP + 1.0f - d_vals[i]);
        w_sum += w[i];
    }
    
    // 2. 归一化权重并计算预测值
    *x = 0.0f;
    *y = 0.0f;
    
    for (int i = 0; i < NEAREST_COUNT; i++) {
        w[i] /= w_sum;
        
        int idx = nearest_idx[i];
        *x += w[i] * ss[idx].x;
        *y += w[i] * ss[idx].y;
        
        // 计算F值需要的分量
        float standard_vec[4] = {ss[idx].e1, ss[idx].e2, ss[idx].e3, ss[idx].e4};
        float norm_c = vector_norm2(standard_vec, 4);
        F_sum += w[i] / norm_c;
    }
    
    // 3. 计算最终的F值
    *F = 20.0f * norm_a * F_sum;
}
















/**
 * @brief  Send RS485 data by DMA
 *
 * @retval None
 */
void rs485_com_send_by_dma(uint8_t size)
{
    DAL_StatusTypeDef status;
    
    /* Enable RS485 sending mode */
    send_485_enable();
    DAL_Delay(1); /* Allow time for RS485 hardware to settle */
    
    /* Send data using DMA */
    status = DAL_UART_Transmit(&huart2, (uint8_t *)uart_dma_tx_buffer_send, size,10);
    
    if (status != DAL_OK)
		{
        /* Handle DMA initialization error */
        Error_Handler();
    }
    
    DAL_Delay(1); /* Allow time for last byte to be sent */
    /* Disable RS485 sending mode */
    recv_485_enable();
}



int32_t process_24bit_signed(const uint8_t data[3]) 
{
    // �Ƚ�24λ���ݴ洢��32λ�����ĵ�24λ
    int32_t value = 0;
		
		value = (data[1] << 8)|data[2];
		
    value |= ((int32_t)data[0] << 16);
    // �������λΪ1������������Ҫ��չ����λ��32λ
    if (value & 0x00800000) {
        value |= 0xFF000000; // ���ø�8λΪ1
    } else {
        value &= 0x00FFFFFF; // ȷ����8λΪ0
    }
    
    return value;
}
/* External functions *****************************************************/

void Spi_WriteReg(uint8_t ch, uint8_t reg, uint8_t data)
{
    uint8_t val;
    SPI_HandleTypeDef *hspi_temp = &hspi1;
    if(ch >= NSA_NUMBER) return;
    //if(ch >= NSA_NUMBER/2) hspi_temp = &hspi2;
    // CS ����
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_RESET);
    //DAL_Delay(1);
    val = 0x00;
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1000);

    val = reg;
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1000);
    val = data;
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1000);

    // CS ����
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_SET);

}

void Spi_ReadReg(uint8_t ch, uint8_t *data, uint8_t reg)
{
    uint8_t val;
    SPI_HandleTypeDef *hspi_temp = &hspi1;
    if(ch >= NSA_NUMBER) return;
    //if(ch >= NSA_NUMBER/2) hspi_temp = &hspi2;
    // CS ����
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_RESET);
    //DAL_Delay(1);
    val = 0x80;
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1000);
    val = reg;
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1000);
    DAL_SPI_Receive(hspi_temp, data, 1, 1000); // ����Dummy���ݶ�ȡ

    // CS ����
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_SET);


}
void Spi_ReadMultiReg(uint8_t ch, uint8_t *data, uint8_t reg, uint8_t len)
{
    uint8_t val;
    SPI_HandleTypeDef *hspi_temp = &hspi1;
    if(ch >= NSA_NUMBER) return;
    //if(ch >= NSA_NUMBER/2) hspi_temp = &hspi2;
    // CS ����
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_RESET);
    
    //DAL_Delay(1);
    val = 0xE0;
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 100);
    val = reg;
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 100);
    
    DAL_SPI_Receive(hspi_temp, data, 1, 100); // ����Dummy���ݶ�ȡ
    DAL_SPI_Receive(hspi_temp, data+1, 1, 100); // ����Dummy���ݶ�ȡ
    DAL_SPI_Receive(hspi_temp, data+2, 1, 100); // ����Dummy���ݶ�ȡ
    
    
    // CS ����
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_SET);


}

uint8_t NSA2302_READ_CALIBRATE(void)
{
	uint8_t addr,i;
    uint8_t conversion_flag = 0;
    uint8_t buf[5] = {0};
    uint8_t temp_index = 0;    /* Calculate and add checksums */
    uint8_t pres_index = 0;    /* Calculate and add checksums */
    uint8_t checksum = 0;
    uint32_t error=0;
    
    temp_buffer[temp_index++] = 0x42;
    temp_buffer[temp_index++] = 0x54;
    
    temp_buffer[temp_index++] = NSA_NUMBER*2+14;//2�ֽ�ͷ��2�ֽڳ��ȣ�1�ֽ��������ͣ�F4�¶ȣ�F5ѹ����4�ֽ�ռλ����4�ֽ����кţ�1�ֽ�β�ۼӺ�
    temp_buffer[temp_index++] = MCU_ADDR;

    temp_buffer[temp_index++] = 0xf4;
    temp_buffer[temp_index++] = 0;
    temp_buffer[temp_index++] = 0;
    temp_buffer[temp_index++] = 0;
    temp_buffer[temp_index++] = 0;

    temp_buffer[temp_index++] = (seq >> 0  )& 0xff;
    temp_buffer[temp_index++] = (seq >> 8  )& 0xff;
    temp_buffer[temp_index++] = (seq >> 16 )& 0xff;
    temp_buffer[temp_index++] = (seq >> 24 )& 0xff;
    //seq++;
    
    pres_buffer[pres_index++] = 0x42;
    pres_buffer[pres_index++] = 0x54;
    
    pres_buffer[pres_index++] = NSA_NUMBER*4+14;//2�ֽ�ͷ��2�ֽڳ��ȣ�1�ֽ��������ͣ�F4�¶ȣ�F5ѹ����4�ֽ�ռλ����4�ֽ����кţ�1�ֽ�β�ۼӺ�
    pres_buffer[pres_index++] = MCU_ADDR;

    pres_buffer[pres_index++] = 0xf5;
    pres_buffer[pres_index++] = 0;
    pres_buffer[pres_index++] = 0;
    pres_buffer[pres_index++] = 0;
    pres_buffer[pres_index++] = 0;

    pres_buffer[pres_index++] = (seq >> 0  )& 0xff;
    pres_buffer[pres_index++] = (seq >> 8  )& 0xff;
    pres_buffer[pres_index++] = (seq >> 16 )& 0xff;
    pres_buffer[pres_index++] = (seq >> 24 )& 0xff;
    //seq++;

    conversion_flag = 0x0A;
    /* Read temperature for all devices */
    //for (addr = 0; addr < NSA_NUMBER; addr++)
    //{
        /* Request temperature conversion */
        //Spi_WriteReg(addr, 0x30, conversion_flag);
    //}

    /* Read temperature values */
    for (addr = 0; addr < NSA_NUMBER; addr++)
    {
        /* Wait for conversion completion */
        //do
        //{
            //Spi_ReadReg(addr, &conversion_flag, 0x30);
        //} while( (conversion_flag == 0x0a) && (error++ <=500));
        //error = 0;
        /* Read temperature data from 0x31 register */
        //Spi_ReadReg(addr, &buf[0], 0x9);
        //Spi_ReadReg(addr, &buf[1], 0xA);
        /* Read pressure data from 0x32 register */
        Spi_ReadReg(addr, &buf[2], 0x6);
        Spi_ReadReg(addr, &buf[3], 0x7);
        Spi_ReadReg(addr, &buf[4], 0x8);        

        //temperature[addr] = (int16_t)((buf[0] << 8) | buf[1]);
        //temperature[addr] = ( (int32_t)((buf[0] << 8) | buf[1]) * 10) >> 8; //除以2的8次方，再乘以10，取一位小数
		//pressor[addr] = (int32_t)process_24bit_signed(buf);
			if(abs((float)process_24bit_signed(&buf[2]), pressor[addr]) < 100000)
			{
        pressor[addr] = (float)process_24bit_signed(&buf[2]);//除以2的23次方，再乘以100，取两位小数
			}
				Spi_ReadReg(addr, &buf[2], 0x6);
        Spi_ReadReg(addr, &buf[3], 0x7);
        Spi_ReadReg(addr, &buf[4], 0x8);        

        if(abs((float)process_24bit_signed(&buf[2]), pressor[addr]) < 100000)
			{
        pressor[addr] = (float)process_24bit_signed(&buf[2]);//除以2的23次方，再乘以100，取两位小数
			}
        //if( (addr == 1) || (addr == 8)) continue;
        /* Store temperature in transmit buffer */
        //temp_buffer[temp_index++] = (uint8_t)(temperature[addr] & 0x00ff); //��λ
        //temp_buffer[temp_index++] = (uint8_t)(temperature[addr] >> 8); //��λ
        pressor_real[addr] = pressor[addr] - pressor_cali[addr];

        /* Store pressure in transmit buffer */
        pres_buffer[pres_index++] = (uint8_t)( ((int32_t)pressor_real[addr] >> 0  )& 0xff); //��λ
        pres_buffer[pres_index++] = (uint8_t)( ((int32_t)pressor_real[addr] >> 8  )& 0xff); //�ε�λ
        pres_buffer[pres_index++] = (uint8_t)( ((int32_t)pressor_real[addr] >> 16 )& 0xff); //�θ�λ
        pres_buffer[pres_index++] = (uint8_t)( ((int32_t)pressor_real[addr] >> 24 )& 0xff); //��λ
    }
		memcpy(s, pressor_real, sizeof(s));
		cos_calculate();
		memcpy(pres_buffer+pres_index, (uint8_t*)&results[0], sizeof(results[0]));
		
		pres_index+= sizeof(results[0]);
			
    /* Temperature frame checksum */
    checksum = 0;
    for(i = 2; i < temp_index; i++) /* Start from length (2) to checksum position (41) */
    {
        checksum += temp_buffer[i];
    }
    temp_buffer[temp_index++] = checksum;

    /* Pressure frame checksum */
    checksum = 0;
    for(i = 2; i < pres_index; i++) /* Start from length (2) to checksum position (41) */
    {
        checksum += pres_buffer[i];
    }
    pres_buffer[pres_index++] = checksum;
    
    /* Send the data frames */
    //send_485_enable();
    //DAL_Delay(1);
    
    /* Send temperature frame */
    //if (DAL_UART_Transmit(&huart2, (uint8_t *)temp_buffer, temp_index, 1000U) != DAL_OK)
    //{
    //    Error_Handler();
    //}
    /* Send pressure frame */
    //if (DAL_UART_Transmit(&huart2, (uint8_t *)pres_buffer, pres_index, 1000U) != DAL_OK)
    //{
    //    Error_Handler();
    //}
    //DAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0x2c<<1, (uint8_t *)temp_buffer, temp_index, 1000U);
    // DAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)0x2c<<1, (uint8_t *)pres_buffer, pres_index);
     // (disabled: now I2C1 works as SLAVE)
//DAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0x2c<<1, (uint8_t *)pres_buffer, pres_index, 1000U);
    //DAL_Delay(1);
    //recv_485_enable();
    
    return 0;
}

uint8_t NSA2302_READ(void)
{
	uint8_t addr,i;
    uint8_t conversion_flag = 0;
    uint8_t buf[3] = {0};
    uint8_t temp_index = 0;    /* Calculate and add checksums */
    uint8_t pres_index = 0;    /* Calculate and add checksums */
    uint8_t checksum = 0;
		uint32_t error = 0;
    
    temp_buffer[temp_index++] = 0x42;
    temp_buffer[temp_index++] = 0x54;
    
    temp_buffer[temp_index++] = NSA_NUMBER*2+14;//2�ֽ�ͷ��2�ֽڳ��ȣ�1�ֽ��������ͣ�F4�¶ȣ�F5ѹ����4�ֽ�ռλ����4�ֽ����кţ�1�ֽ�β�ۼӺ�
    temp_buffer[temp_index++] = MCU_ADDR;

    temp_buffer[temp_index++] = 0xf4;
    temp_buffer[temp_index++] = 0;
    temp_buffer[temp_index++] = 0;
    temp_buffer[temp_index++] = 0;
    temp_buffer[temp_index++] = 0;

    temp_buffer[temp_index++] = (seq >> 0   )& 0xff;
    temp_buffer[temp_index++] = (seq >> 8   )& 0xff;
    temp_buffer[temp_index++] = (seq >> 16  )& 0xff;
    temp_buffer[temp_index++] = (seq >> 24  )& 0xff;
    seq++;
    
    conversion_flag = 0x08;
    /* Read temperature for all devices */
    for (addr = 0; addr < NSA_NUMBER; addr++)
    {
        
        /* Request temperature conversion */
        Spi_WriteReg(addr, 0x30, conversion_flag);
    }

    /* Read temperature values */
    for (addr = 0; addr < NSA_NUMBER; addr++)
    {
        /* Wait for conversion completion */
        error = 0;
        do
        {
            Spi_ReadReg(addr, &conversion_flag, 0x30);
        } while( (conversion_flag == 0x08) && (error++ <=500));
		//if(error >= 500) continue;
        /* Read temperature data from 0x31 register */
        
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0;
        //Spi_ReadMultiReg(addr, buf, 0x9, 2);
        Spi_ReadReg(addr, &buf[0], 0x9);
        Spi_ReadReg(addr, &buf[1], 0xA);
        //if( (addr == 1) || (addr == 8)) continue;
        //temperature[addr] = (int16_t)((buf[0] << 8) | buf[1]);
        temperature[addr] = ( (int32_t)((buf[0] << 8) | buf[1]) * 10) >> 8; //除以2的8次方，再乘以10，取一位小数
        /* Store temperature in transmit buffer */
        temp_buffer[temp_index++] = (uint8_t)(temperature[addr] & 0x00ff); //��λ
        temp_buffer[temp_index++] = (uint8_t)(temperature[addr] >> 8); //��λ
        Spi_WriteReg(addr, 0xa5, 0x12);
    }

    /* Temperature frame checksum */
    checksum = 0;
    for(i = 2; i < temp_index; i++) /* Start from length (2) to checksum position (41) */
    {
        checksum += temp_buffer[i];
    }
    temp_buffer[temp_index++] = checksum;

    pres_buffer[pres_index++] = 0x42;
    pres_buffer[pres_index++] = 0x54;
    
    pres_buffer[pres_index++] = NSA_NUMBER*4+14;//2�ֽ�ͷ��2�ֽڳ��ȣ�1�ֽ��������ͣ�F4�¶ȣ�F5ѹ����4�ֽ�ռλ����4�ֽ����кţ�1�ֽ�β�ۼӺ�
    pres_buffer[pres_index++] = MCU_ADDR;

    pres_buffer[pres_index++] = 0xf5;
    pres_buffer[pres_index++] = 0;
    pres_buffer[pres_index++] = 0;
    pres_buffer[pres_index++] = 0;
    pres_buffer[pres_index++] = 0;

    pres_buffer[pres_index++] = (seq >> 0  )& 0xff;
    pres_buffer[pres_index++] = (seq >> 8  )& 0xff;
    pres_buffer[pres_index++] = (seq >> 16 )& 0xff;
    pres_buffer[pres_index++] = (seq >> 24 )& 0xff;
    seq++;

    conversion_flag = 0x09;
    /* Read pressure for all devices */
    for (addr = 0; addr < NSA_NUMBER; addr++)
    {
        /* Request pressure conversion */
        Spi_WriteReg(addr, 0x30, conversion_flag);
    }

    /* Read pressure values */
    for (addr = 0; addr < NSA_NUMBER; addr++)
    {
        /* Wait for conversion completion */
        
        error = 0;
        do
        {
            Spi_ReadReg(addr, &conversion_flag, 0x30);
        } while( (conversion_flag == 0x09) && (error++ <=500));
        //if(error >= 500) continue;
        /* Read pressure data from 0x32 register */
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0;
        //Spi_ReadMultiReg(addr, buf, 0x6, 3);
        Spi_ReadReg(addr, &buf[0], 0x6);
        Spi_ReadReg(addr, &buf[1], 0x7);
        Spi_ReadReg(addr, &buf[2], 0x8);
        //if( (addr == 1) || (addr == 8)) continue;
        pressor[addr] = (int32_t)process_24bit_signed(buf);
        //pressor[addr] = (int32_t)((float)process_24bit_signed(buf)*0.1125f);//除以2的23次方，再乘以100，取两位小数
        /* Store pressure in transmit buffer */
        pres_buffer[pres_index++] = (uint8_t)( ((int32_t)pressor[addr] >> 0  )& 0xff); //��λ
        pres_buffer[pres_index++] = (uint8_t)( ((int32_t)pressor[addr] >> 8  )& 0xff); //�ε�λ
        pres_buffer[pres_index++] = (uint8_t)( ((int32_t)pressor[addr] >> 16 )& 0xff); //�θ�λ
        pres_buffer[pres_index++] = (uint8_t)( ((int32_t)pressor[addr] >> 24 )& 0xff); //��λ
        Spi_WriteReg(addr, 0xa5, 0x10);
    }

    /* Temperature frame checksum */
    checksum = 0;
    for(i = 2; i < pres_index; i++) /* Start from length (2) to checksum position (41) */
    {
        checksum += pres_buffer[i];
    }
    pres_buffer[pres_index++] = checksum;
    
        // 1. 定义或清空目标缓冲区
    memset(uart_dma_tx_buffer, 0, sizeof(uart_dma_tx_buffer));

    // 2. 拷贝 temp_buffer 到开头
    memcpy(uart_dma_tx_buffer, temp_buffer, temp_index);

    // 3. 将 pres_buffer 拷贝到 temp_buffer 的末尾
    memcpy(uart_dma_tx_buffer + temp_index, pres_buffer, pres_index);

    // 4. 计算总长度并发送
    uint8_t total_len = temp_index + pres_index;

    /* Send the data frames */
    //send_485_enable();
    //DAL_Delay(1);
    
    /* Send pressure frame */
    //if (DAL_UART_Transmit(&huart2, (uint8_t *)uart_dma_tx_buffer, total_len,1000) != DAL_OK)
    //{
     //   Error_Handler();

    //}
    //DAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0x2c<<1, (uint8_t *)uart_dma_tx_buffer, total_len, 1000U);
    // DAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)0x2c<<1, (uint8_t *)uart_dma_tx_buffer, total_len);

     // (disabled: now I2C1 works as SLAVE)
//DAL_Delay(1); /* Allow time for last byte to be sent */
    //recv_485_enable();
    
    return 0;
}



uint32_t clkfreq;
int main(void)
{
    volatile uint32_t addr;


    /* Device configuration */
    DAL_DeviceConfig();

    /* Start I2C1 in SLAVE mode: wait for 0x55 0xAA from master */
    (void)DAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_cmd, sizeof(i2c_rx_cmd));
    //(void)DAL_I2C_Slave_Transmit_IT(&hi2c1, pres_buffer, 42);
	
		/* Initialize SPI Chip Select pins (CS1-CS12) */
    SPI_CS_Init();
	
    clkfreq=DAL_RCM_GetSysClockFreq();
//    /* Verify SPI clock is enabled */
//    if (SPI_CheckClockEnabled(&hspi1))
//    {
//        // SPI1时钟已打开
//        //printf("SPI1 clock is enabled\n");
//			  BOARD_LED_Off(LED3);
//			  DAL_Delay(3000);
//				
//    }
//    else
//    {
//        // SPI1时钟未打开
//        //printf("SPI1 clock is NOT enabled\n");
//        Error_Handler();
//    }
//    
//    if (SPI_CheckClockEnabled(&hspi2))
//    {
//        // SPI2时钟已打开
//        //printf("SPI2 clock is enabled\n");
//			  BOARD_LED_On(LED3);
//				DAL_Delay(3000);
//    }
//    else
//    {
//        // SPI2时钟未打开
//        //printf("SPI2 clock is NOT enabled\n");
//        Error_Handler();
//    }

				// 测试SPI CLK输出 - 持续发送，方便示波器测量
    #if 0 // 设置为1启用持续发送测试，设置为0禁用（CLK波形验证完成后应设为0）
    {
        
        while(1)
        {
            // 发送数据
            //DAL_SPI_Transmit(&hspi1, &test_byte, 1, 100);
//			  DAL_SPI_Receive(&hspi1, &test_byte, 1, 100);
					 // CS拉低 - 开始传输
            DAL_GPIO_WritePin(NSA2302_CS_PORT[4], NSA2302_CS_PIN[4], GPIO_PIN_RESET);
                
            // 发送测试数据（0xAA = 10101010，便于观察波形）
						uint8_t test_byte = 0xAA;  // 测试数据：10101010，便于观察波形
            DAL_SPI_Transmit(&hspi1, &test_byte, 1, 100);
					
						uint8_t test_byte2 = 0xAA;
						DAL_SPI_Receive(&hspi1, &test_byte2, 1, 100); 
					
                
            // CS拉高 - 结束传输
            DAL_GPIO_WritePin(NSA2302_CS_PORT[4], NSA2302_CS_PIN[4], GPIO_PIN_SET);
        }
    }
    #endif
		                                                                                                                      
    /* ========================================================================
     * SPI四种模式波形测试
     * ========================================================================
     * 功能：循环测试SPI的四种模式（Mode 0/1/2/3），持续发送测试数据用于示波器观察波形
     * 
     * 测试说明：
     * 1. 程序会依次切换到Mode 0, 1, 2, 3，每种模式持续发送数据约几秒钟
     * 2. LED闪烁次数表示当前模式：Mode 0闪烁1次，Mode 1闪烁2次，以此类推
     * 3. 使用示波器观察以下信号：
     *    - SCK (PA5): SPI时钟信号
     *    - MOSI (PA7): 主设备输出数据信号
     *    - CS (PC0): 片选信号（低电平有效）
     * 4. 测试数据为0xAA (10101010)，便于观察波形（高低电平交替）
     * 
     * 四种模式的特点：
     * - Mode 0 (CPOL=0, CPHA=0): SCK空闲低，上升沿采样，下降沿输出
     * - Mode 1 (CPOL=0, CPHA=1): SCK空闲低，下降沿采样，上升沿输出
     * - Mode 2 (CPOL=1, CPHA=0): SCK空闲高，下降沿采样，上升沿输出
     * - Mode 3 (CPOL=1, CPHA=1): SCK空闲高，上升沿采样，下降沿输出
     * ======================================================================== */
    #if 0  // 设置为1启用四种模式波形测试，设置为0禁用
    {
        uint8_t test_byte = 0xAA;  // 测试数据：10101010，便于观察波形（高低电平交替）
        uint8_t spi_mode = 0;      // 当前测试的SPI模式 (0, 1, 2, 3)
        uint32_t mode_test_count = 0;
        uint32_t mode_test_duration = 500000;  // 每个模式测试持续时间（发送次数，约几秒）
        
        BOARD_LED_Off(LED3);
        DAL_Delay(500);
        
        // 循环测试四种SPI模式
        while(1)
        {
            // 切换到下一个SPI模式
            DAL_SPI1_Reconfig_Mode(spi_mode);
            DAL_Delay(10);  // 等待SPI配置稳定
            
            // LED闪烁表示当前模式：Mode 0闪烁1次，Mode 1闪烁2次，以此类推
            for(int i = 0; i < spi_mode + 1; i++)
            {
                BOARD_LED_Off(LED3);
                DAL_Delay(500);
                BOARD_LED_On(LED3);
                DAL_Delay(500);
            }
            DAL_Delay(500);  // 模式切换后的稳定延时
            
            // 持续发送测试数据，用于示波器观察波形
            // 观察点：SCK(PA5), MOSI(PA7), CS(PC0)
            mode_test_count = 0;
            while(mode_test_count < mode_test_duration)
            {
                // CS拉低 - 开始传输
                DAL_GPIO_WritePin(NSA2302_CS_PORT[0], NSA2302_CS_PIN[0], GPIO_PIN_RESET);
                
                // 发送测试数据（0xAA = 10101010，便于观察波形）
                DAL_SPI_Transmit(&hspi1, &test_byte, 1, 100);
                
                // CS拉高 - 结束传输
                DAL_GPIO_WritePin(NSA2302_CS_PORT[0], NSA2302_CS_PIN[0], GPIO_PIN_SET);
                
                mode_test_count++;
                
                // 添加小延时，让示波器能够触发和观察
                DAL_Delay(1);
            }
            
            // 切换到下一个模式
            spi_mode++;
            if(spi_mode >= 4)
            {
                spi_mode = 0;  // 循环回到Mode 0
                DAL_Delay(1000);  // 一轮测试完成，延时1秒后开始下一轮
            }
        }
    }
    #endif
    
	  
//		BOARD_LED_On(LED3);
		//BOARD_LED_Off(LED3);
		//BOARD_LED_On(LED3);
		//BOARD_LED_Off(LED3);
    #if 1
    for (addr = 0; addr < NSA_NUMBER; addr++)
    {		
        Spi_WriteReg(addr, 0x00, 0x24);
    }
    DAL_Delay(1000); // ��ʱ1�룬

    for (addr = 0; addr < NSA_NUMBER; addr++)
    {		
        Spi_WriteReg(addr, 0x00, 0x81);
    }


    for (addr = 0; addr < NSA_NUMBER; addr++)
    {
        Spi_WriteReg(addr, 0xa4, 0x00);
        Spi_WriteReg(addr, 0xa5, 0x92);
        Spi_WriteReg(addr, 0xa6, 0x60);
        Spi_WriteReg(addr, 0xa7, 0x00);

        Spi_WriteReg(addr, 0x6a, 0x40);
        Spi_WriteReg(addr, 0x6c, 0x6a);
    }
    DAL_Delay(1000); // ��ʱ1�룬�ȴ�EEPROMд��
    #endif
    //recv_485_enable();
    /* Enable USART2 IDLE DMA reception */
    //if (DAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_dma_rx_buffer, sizeof(uart_dma_rx_buffer)) != DAL_OK)
    //{
    //    Error_Handler();
    //}
		
		NSA2302_READ_CALIBRATE();
		NSA2302_READ_CALIBRATE();
		
		DAL_Delay(5000); // ��ʱ1�룬�ȴ�EEPROMд��
		memcpy(pressor_cali, pressor, sizeof(pressor_cali));
		
    uint8_t comm_addr = 0;
    /* Infinite loop */
    while (1)
    {
        //NSA2302_READ();
        //rs485_temp_collector_flag = 1;
        NSA2302_READ_CALIBRATE();
        if(rs485_temp_collector_flag == 1)
        {
            //if(FLAG_CALI == 0x01)
                NSA2302_READ_CALIBRATE();//如果要校准之后的值，寄存器都配置好了，用这一行
				//NSA2302_READ();//如果未校准，则用这一行读取原始值来校准，写寄存器
            //else
            //    NSA2302_READ();
            rs485_temp_collector_flag = 0;
						
						
            // Ensure DMA reception is restarted after sending data
            if (DAL_UART_GetState(&huart2) == DAL_UART_STATE_READY)
            {
                // Clear buffer before restarting DMA
                memset(uart_dma_rx_buffer, 0, sizeof(uart_dma_rx_buffer));
                // Restart IDLE DMA reception
                if (DAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_dma_rx_buffer, sizeof(uart_dma_rx_buffer)) != DAL_OK)
                {
                    // If restart fails, try to stop and restart again
                    DAL_UART_DMAStop(&huart2);
                    DAL_Delay(1);
                    if (DAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_dma_rx_buffer, sizeof(uart_dma_rx_buffer)) != DAL_OK)
                    {
                        Error_Handler();
                    }
                }
            }
						
            continue;
        }
				//rs485_temp_cali_reg_flag = 1;
        if(rs485_temp_cali_reg_flag == 1)
	    {
            comm_addr = uart_dma_rx_buffer[3];
            if( (uart_dma_rx_buffer[3] >= 2) && (uart_dma_rx_buffer[3] <= 6))  uart_dma_rx_buffer[3]+=1;
            else if( (uart_dma_rx_buffer[3] >= 7) && (uart_dma_rx_buffer[3] <= 11)) uart_dma_rx_buffer[3]+=2;
            Spi_WriteReg(uart_dma_rx_buffer[3], 0x00, 0x24);
            DAL_Delay(2000); // ��ʱ1�룬�ȴ�EEPROMд��

            Spi_WriteReg(uart_dma_rx_buffer[3], 0x00, 0x81);

            Spi_WriteReg(uart_dma_rx_buffer[3], 0xaa, uart_dma_rx_buffer[4]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xab, uart_dma_rx_buffer[5]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xac, uart_dma_rx_buffer[6]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xad, uart_dma_rx_buffer[7]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xae, uart_dma_rx_buffer[8]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xaf, uart_dma_rx_buffer[9]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb0, uart_dma_rx_buffer[10]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb1, uart_dma_rx_buffer[11]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb2, uart_dma_rx_buffer[12]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb3, uart_dma_rx_buffer[13]);
            
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb4, uart_dma_rx_buffer[14]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb5, uart_dma_rx_buffer[15]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb6, uart_dma_rx_buffer[16]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb7, uart_dma_rx_buffer[17]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb8, uart_dma_rx_buffer[18]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xb9, uart_dma_rx_buffer[19]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xba, uart_dma_rx_buffer[20]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xbb, uart_dma_rx_buffer[21]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xbc, uart_dma_rx_buffer[22]);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0xa2, uart_dma_rx_buffer[23]);
            
            Spi_WriteReg(uart_dma_rx_buffer[3], 0x6a, 0x40);
            Spi_WriteReg(uart_dma_rx_buffer[3], 0x6c, 0x6a);
            uart_dma_rx_buffer[3] = comm_addr;
            memcpy(uart_dma_tx_buffer_send, uart_dma_rx_buffer, 24);
            rs485_com_send_by_dma(24);
            rs485_temp_cali_reg_flag = 0;
            Write_Buffer[0] = MCU_ADDR;
            Write_Buffer[1] = 0x01;
            Flash_Write(FLASH_USER_ADDR, Write_Buffer, 4);
            continue;
	    }
        if(rs485_temp_read_cali_reg_flag == 1)
        {
            comm_addr = uart_dma_rx_buffer[3];
            if( (uart_dma_rx_buffer[3] >= 2) && (uart_dma_rx_buffer[3] <= 6))  uart_dma_rx_buffer[3]+=1;
            else if( (uart_dma_rx_buffer[3] >= 7) && (uart_dma_rx_buffer[3] <= 11)) uart_dma_rx_buffer[3]+=2;
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[4], 0xaa);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[5], 0xab);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[6], 0xac);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[7], 0xad);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[8], 0xae);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[9], 0xaf);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[10], 0xb0);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[11], 0xb1);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[12], 0xb2);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[13], 0xb3);
            
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[14], 0xb4);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[15], 0xb5);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[16], 0xb6);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[17], 0xb7);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[18], 0xb8);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[19], 0xb9);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[20], 0xba);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[21], 0xbb);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[22], 0xbc);
            Spi_ReadReg(uart_dma_rx_buffer[3], &uart_dma_rx_buffer[23], 0xa2);
            
            uart_dma_rx_buffer[3] = comm_addr;
            memcpy(uart_dma_tx_buffer_send, uart_dma_rx_buffer, 24);
            rs485_com_send_by_dma(24);
            rs485_temp_read_cali_reg_flag = 0;
            continue;
        }

    
        if(rs485_addr_flag == 1)
        {
            Write_Buffer[0] = uart_dma_rx_buffer[2];
            Write_Buffer[1] = uart_dma_rx_buffer[3];
            Flash_Write(FLASH_USER_ADDR, Write_Buffer, 4);
            rs485_addr_flag = 0; //�����ַ��־
			memcpy(uart_dma_tx_buffer_send, uart_dma_rx_buffer, sizeof(uart_dma_rx_buffer));
            rs485_com_send_by_dma(4);
						continue;
        }
    }
}
#if 0
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
    uartEventFlag = 1U;
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
        if (huart->Instance == USART2)
    {
        // Process received data based on the received size
        // The data is in uart_dma_rx_buffer
        //uint8_t i;
        
        // Check if we have enough data to detect frame headers
            // Check for address flag frame header (0x7e, 0xe7)
            if (uart_dma_rx_buffer[0] == 0x7e && uart_dma_rx_buffer[1] == 0xe7) {
                rs485_addr_flag = 1;
            }
            
            // Check other frame headers that require address matching (needs at least 3 bytes)
                if (uart_dma_rx_buffer[0] == 0x7a && uart_dma_rx_buffer[1] == 0xa7 && uart_dma_rx_buffer[2] == MCU_ADDR) {
                    rs485_temp_cali_reg_flag = 1;
                }
                if (uart_dma_rx_buffer[0] == 0x7b && uart_dma_rx_buffer[1] == 0xb7 && uart_dma_rx_buffer[2] == MCU_ADDR) {
                    rs485_temp_collector_flag = 1;
                }
                if (uart_dma_rx_buffer[0] == 0x7c && uart_dma_rx_buffer[1] == 0xc7 && uart_dma_rx_buffer[2] == MCU_ADDR) {
                    rs485_temp_read_cali_reg_flag = 1;
                }
 
        // Set a general UART event flag to indicate data has been received
        uartEventFlag = 1;
        
        // Restart the IDLE DMA reception (for case when DMA buffer is full)
        // Note: IDLE event callback will also restart DMA, but this handles buffer-full case
        if (DAL_UART_GetState(&huart2) == DAL_UART_STATE_READY)
        {
            // Clear buffer before restarting DMA
            memset(uart_dma_rx_buffer, 0, sizeof(uart_dma_rx_buffer));
            // Restart IDLE DMA reception
            if (DAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_dma_rx_buffer, sizeof(uart_dma_rx_buffer)) != DAL_OK)
            {
                // If restart fails, try to stop and restart again
                DAL_UART_DMAStop(&huart2);
                DAL_Delay(1);
                if (DAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_dma_rx_buffer, sizeof(uart_dma_rx_buffer)) != DAL_OK)
                {
                    Error_Handler();
                }
            }
        }
    }
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

/**
 * @brief  Rx Event callbacks
 *
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module
 *
 * @param  Size  Number of data received
 *
 * @retval None
 */

void DAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
    {
        // Process received data based on the received size
        // The data is in uart_dma_rx_buffer
        
        // Check if we have enough data to detect frame headers
        // Check for address flag frame header (0x7e, 0xe7)
        if (uart_dma_rx_buffer[0] == 0x7e && uart_dma_rx_buffer[1] == 0xe7) {
            rs485_addr_flag = 1;
        }
        
        // Check other frame headers that require address matching (needs at least 3 bytes)
        if (uart_dma_rx_buffer[0] == 0x7a && uart_dma_rx_buffer[1] == 0xa7 && uart_dma_rx_buffer[2] == MCU_ADDR) {
            rs485_temp_cali_reg_flag = 1;
        }
        if (uart_dma_rx_buffer[0] == 0x7b && uart_dma_rx_buffer[1] == 0xb7 && uart_dma_rx_buffer[2] == MCU_ADDR) {
            rs485_temp_collector_flag = 1;
        }
        if (uart_dma_rx_buffer[0] == 0x7c && uart_dma_rx_buffer[1] == 0xc7 && uart_dma_rx_buffer[2] == MCU_ADDR) {
            rs485_temp_read_cali_reg_flag = 1;
        }
 
        // Set a general UART event flag to indicate data has been received
        uartEventFlag = 1;
        
        // Stop DMA receive if it's still running (ensure clean state)
        if (DAL_UART_GetState(&huart2) != DAL_UART_STATE_READY)
        {
            DAL_UART_DMAStop(&huart2);
        }
        
        // Clear buffer before restarting DMA to avoid old data interference
        //memset(uart_dma_rx_buffer, 0, sizeof(uart_dma_rx_buffer));
        
        // Restart the IDLE DMA reception - use IDLE interrupt mode to handle variable length data
        if (DAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_dma_rx_buffer, sizeof(uart_dma_rx_buffer)) != DAL_OK)
        {
            // If restart fails, try to stop and restart again
            DAL_UART_DMAStop(&huart2);
            DAL_Delay(1);
            if (DAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uart_dma_rx_buffer, sizeof(uart_dma_rx_buffer)) != DAL_OK)
            {
                Error_Handler();
            }
        }
    }
}

#endif


extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;

void I2C1_EV_IRQHandler(void)
{
    DAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    DAL_I2C_ER_IRQHandler(&hi2c1);
}
volatile uint8_t i2c_tx_done = 0;   /* kept for compatibility (no longer used in slave mode) */
volatile uint32_t i2c_err = 0;

/* I2C1 SLAVE callbacks:
 * - wait for 2-byte command (0x55 0xAA)
 * - when matched, prepare TX buffer and arm slave transmit
 */
static void I2C_PrepareTxFromLatestFrames(void)
{
    uint16_t plen = (uint16_t)((uint8_t)pres_buffer[2]);

    if (plen > sizeof(pres_buffer)) plen = 0;

    i2c_tx_len = 0;

    if (plen > 0)
    {
        memcpy(i2c_tx_buf, pres_buffer, plen);
        i2c_tx_len += plen;
    }

    if (i2c_tx_len == 0)
    {
        i2c_tx_buf[0] = 0x00;
        i2c_tx_len = 1;
    }
}

void DAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;

    /* Check 0x55AA command */
    if ((i2c_rx_cmd[0] == 0x55U) && (i2c_rx_cmd[1] == 0xAAU))
    {
        i2c_ready_to_tx = 1U;
        I2C_PrepareTxFromLatestFrames();

        /* Arm slave transmit: master must issue a READ to fetch data */
        (void)DAL_I2C_Slave_Transmit_IT(&hi2c1, pres_buffer, 42);
    }
    else if ((i2c_rx_cmd[0] == 0x55U) && (i2c_rx_cmd[1] == 0xABU))
		{
			memcpy(pressor_cali, pressor, sizeof(pressor_cali));
			(void)DAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_cmd, sizeof(i2c_rx_cmd));
		}
		else {
        /* Not our command -> keep listening */
        (void)DAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_cmd, sizeof(i2c_rx_cmd));
    }
}

void DAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;

    i2c_ready_to_tx = 0U;

    /* Rearm receive for next 0x55AA command */
    (void)DAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_cmd, sizeof(i2c_rx_cmd));
    //seq++;
    //(void)DAL_I2C_Slave_Transmit_IT(&hi2c1, pres_buffer, 30);
}

void DAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;

    i2c_err = hi2c->ErrorCode;

    /* Recover by rearming receive */
    (void)DAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_cmd, sizeof(i2c_rx_cmd));
}