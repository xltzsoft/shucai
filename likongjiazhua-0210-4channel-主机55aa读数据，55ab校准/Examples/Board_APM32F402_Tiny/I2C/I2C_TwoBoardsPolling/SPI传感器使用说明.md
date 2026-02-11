# SPI传感器温度压力读写使用说明

## 1. 硬件配置

### 片选引脚分配 (已在main.c中定义)

| 传感器编号 | CS引脚 | SPI控制器 | 数组索引 |
|-----------|--------|-----------|---------|
| 传感器0   | PC0    | SPI1      | ch=0    |
| 传感器1   | PC1    | SPI1      | ch=1    |
| 传感器2   | PC2    | SPI1      | ch=2    |
| 传感器3   | PC3    | SPI1      | ch=3    |
| 传感器4   | PC4    | SPI1      | ch=4    |
| 传感器5   | PC5    | SPI1      | ch=5    |
| 传感器6   | PC6    | SPI2      | ch=6    |
| 传感器7   | PC7    | SPI2      | ch=7    |
| 传感器8   | PC8    | SPI2      | ch=8    |
| 传感器9   | PC9    | SPI2      | ch=9    |
| 传感器10  | PC10   | SPI2      | ch=10   |
| 传感器11  | PC11   | SPI2      | ch=11   |

**重要说明**：
- 传感器0-5 使用 **SPI1** (PA5:SCK, PA6:MISO, PA7:MOSI)
- 传感器6-11 使用 **SPI2** (PB13:SCK, PB14:MISO, PB15:MOSI)
- CS引脚已在 `apm32f4xx_gpio_cfg.c` 中初始化完成

## 2. 现有API函数说明

### 2.1 写寄存器函数

```c
void Spi_WriteReg(uint8_t ch, uint8_t reg, uint8_t data)
```

**参数**：
- `ch`: 传感器通道号 (0-11，对应传感器0-11)
- `reg`: 寄存器地址
- `data`: 要写入的数据

**功能**：
- 自动选择对应的SPI接口（ch<6用SPI1，ch>=6用SPI2）
- 自动控制对应的CS引脚
- 发送写命令、寄存器地址、数据

**实现原理**：
```c
void Spi_WriteReg(uint8_t ch, uint8_t reg, uint8_t data)
{
    uint8_t val;
    SPI_HandleTypeDef *hspi_temp = &hspi1;
    if(ch >= nsa_device_num) return;
    if(ch >= 6) hspi_temp = &hspi2;  // ch 6-11 使用SPI2
    
    // CS拉低 - 选中从设备
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_RESET);
    
    val = 0x00;  // 写命令
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1);
    
    val = reg;   // 寄存器地址
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1);
    
    val = data;  // 数据
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1);
    
    // CS拉高 - 释放从设备
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_SET);
}
```

### 2.2 读寄存器函数

```c
void Spi_ReadReg(uint8_t ch, uint8_t *data, uint8_t reg)
```

**参数**：
- `ch`: 传感器通道号 (0-11)
- `data`: 指向接收数据的指针
- `reg`: 寄存器地址

**功能**：
- 自动选择对应的SPI接口
- 自动控制对应的CS引脚
- 发送读命令、寄存器地址，接收数据

**实现原理**：
```c
void Spi_ReadReg(uint8_t ch, uint8_t *data, uint8_t reg)
{
    uint8_t val;
    SPI_HandleTypeDef *hspi_temp = &hspi1;
    if(ch >= nsa_device_num) return;
    if(ch >= 6) hspi_temp = &hspi2;  // ch 6-11 使用SPI2
    
    // CS拉低 - 选中从设备
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_RESET);
    
    val = 0x80;  // 读命令（最高位为1表示读）
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1);
    
    val = reg;   // 寄存器地址
    DAL_SPI_Transmit(hspi_temp, &val, sizeof(val), 1);
    
    DAL_SPI_Receive(hspi_temp, data, 1, 1);  // 接收数据
    
    // CS拉高 - 释放从设备
    DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_SET);
}
```

## 3. 使用示例

### 3.1 单个传感器读写示例

```c
// 示例1：写入传感器0的寄存器
void example_write_sensor0(void)
{
    uint8_t ch = 0;        // 传感器0 (使用PC0作为CS，SPI1)
    uint8_t reg = 0x00;    // 寄存器地址
    uint8_t data = 0x24;   // 要写入的数据
    
    Spi_WriteReg(ch, reg, data);
}

// 示例2：读取传感器0的寄存器
void example_read_sensor0(void)
{
    uint8_t ch = 0;        // 传感器0
    uint8_t reg = 0x30;    // 寄存器地址
    uint8_t read_data = 0;
    
    Spi_ReadReg(ch, &read_data, reg);
    
    // 此时read_data中包含了读取的数据
    printf("Sensor 0, Reg 0x%02X: 0x%02X\n", reg, read_data);
}

// 示例3：操作传感器7（使用SPI2）
void example_sensor7(void)
{
    uint8_t ch = 7;        // 传感器7 (使用PC7作为CS，SPI2)
    uint8_t reg = 0x00;
    uint8_t data = 0x81;
    
    // 写入
    Spi_WriteReg(ch, reg, data);
    DAL_Delay(10);
    
    // 读取验证
    uint8_t read_data = 0;
    Spi_ReadReg(ch, &read_data, reg);
}
```

### 3.2 批量初始化所有传感器

```c
void init_all_sensors(void)
{
    uint8_t ch;
    
    // 初始化所有12个传感器
    for (ch = 0; ch < 12; ch++)
    {
        // 写入配置寄存器
        Spi_WriteReg(ch, 0x00, 0x24);  // 第一步配置
        DAL_Delay(10);
    }
    
    DAL_Delay(1000);  // 等待传感器稳定
    
    for (ch = 0; ch < 12; ch++)
    {
        Spi_WriteReg(ch, 0x00, 0x81);  // 第二步配置
        DAL_Delay(10);
    }
}
```

### 3.3 读取温度数据（参考main.c中的实现）

```c
// 根据main.c中的NSA2302_READ_CALIBRATE函数改编
void read_temperature_all_sensors(void)
{
    uint8_t ch;
    uint8_t conversion_flag;
    uint8_t buf[3];
    int16_t temperature[12];
    
    // 1. 启动所有传感器的温度转换
    conversion_flag = 0x0A;  // 温度转换命令
    for (ch = 0; ch < 12; ch++)
    {
        Spi_WriteReg(ch, 0x30, conversion_flag);
    }
    
    // 2. 等待转换完成并读取数据
    for (ch = 0; ch < 12; ch++)
    {
        // 等待转换完成（轮询状态寄存器）
        do
        {
            Spi_ReadReg(ch, &conversion_flag, 0x30);
            DAL_Delay(1);
        } while ((conversion_flag & 0x08) != 0);  // 等待转换完成标志
        
        // 读取温度数据（3字节）
        Spi_ReadReg(ch, &buf[0], 0x31);  // 高字节
        Spi_ReadReg(ch, &buf[1], 0x32);  // 中字节
        Spi_ReadReg(ch, &buf[2], 0x33);  // 低字节
        
        // 合成16位温度数据
        temperature[ch] = (int16_t)((buf[0] << 8) | buf[1]);
        
        printf("Sensor %d Temperature: %d\n", ch, temperature[ch]);
    }
}
```

### 3.4 读取压力数据

```c
void read_pressure_all_sensors(void)
{
    uint8_t ch;
    uint8_t conversion_flag;
    uint8_t buf[3];
    int32_t pressure[12];
    
    // 1. 启动所有传感器的压力转换
    conversion_flag = 0x09;  // 压力转换命令
    for (ch = 0; ch < 12; ch++)
    {
        Spi_WriteReg(ch, 0x30, conversion_flag);
    }
    
    // 2. 等待转换完成并读取数据
    for (ch = 0; ch < 12; ch++)
    {
        // 等待转换完成
        do
        {
            Spi_ReadReg(ch, &conversion_flag, 0x30);
            DAL_Delay(1);
        } while ((conversion_flag & 0x08) != 0);
        
        // 读取压力数据（3字节）
        Spi_ReadReg(ch, &buf[0], 0x28);  // 高字节
        Spi_ReadReg(ch, &buf[1], 0x29);  // 中字节
        Spi_ReadReg(ch, &buf[2], 0x2a);  // 低字节
        
        // 合成24位压力数据（有符号）
        pressure[ch] = process_24bit_signed(buf);
        
        printf("Sensor %d Pressure: %ld\n", ch, pressure[ch]);
    }
}
```

### 3.5 循环读取所有传感器

```c
void continuous_read_all_sensors(void)
{
    while(1)
    {
        // 读取温度
        read_temperature_all_sensors();
        
        DAL_Delay(100);
        
        // 读取压力
        read_pressure_all_sensors();
        
        DAL_Delay(500);  // 每500ms读取一次
    }
}
```

## 4. 常用寄存器地址（NSA2302传感器）

根据代码中的使用情况，常用寄存器如下：

| 寄存器地址 | 功能说明 | 读/写 |
|-----------|---------|------|
| 0x00      | 配置寄存器 | 读/写 |
| 0x30      | 转换控制/状态寄存器 | 读/写 |
| 0x28-0x2A | 压力数据（3字节）| 只读 |
| 0x31-0x33 | 温度数据（3字节）| 只读 |
| 0x6A      | 用户配置寄存器1 | 读/写 |
| 0xA4-0xA7 | 校准寄存器 | 读/写 |
| 0xBB-0xBC | 其他配置寄存器 | 读/写 |

## 5. 完整的初始化和读取流程（来自main.c）

```c
void sensor_init_and_read_example(void)
{
    uint8_t ch;
    
    // 步骤1：第一阶段初始化
    for (ch = 0; ch < 12; ch++)
    {		
        Spi_WriteReg(ch, 0x00, 0x24);
    }
    DAL_Delay(1000);  // 延时1秒

    // 步骤2：第二阶段初始化
    for (ch = 0; ch < 12; ch++)
    {		
        Spi_WriteReg(ch, 0x00, 0x81);
    }

    // 步骤3：配置传感器参数
    for (ch = 0; ch < 12; ch++)
    {
        Spi_WriteReg(ch, 0xa4, 0x00);
        Spi_WriteReg(ch, 0xa5, 0x10);
        Spi_WriteReg(ch, 0xa6, 0x83);
        Spi_WriteReg(ch, 0xa7, 0x83);
        Spi_WriteReg(ch, 0x6a, 0x40);
    }
    
    // 步骤4：开始连续读取
    while(1)
    {
        // 读取温度和压力
        read_temperature_all_sensors();
        read_pressure_all_sensors();
        
        DAL_Delay(100);  // 100ms间隔
    }
}
```

## 6. 调试技巧

### 6.1 单个传感器测试
首先测试单个传感器，确认通信正常：

```c
void debug_single_sensor(void)
{
    uint8_t ch = 0;  // 测试传感器0
    uint8_t write_data = 0x55;
    uint8_t read_data = 0;
    
    // 写入测试
    Spi_WriteReg(ch, 0x00, write_data);
    DAL_Delay(10);
    
    // 读取验证
    Spi_ReadReg(ch, &read_data, 0x00);
    
    // 设置断点在这里，查看read_data的值
    if (read_data == write_data)
    {
        BOARD_LED_Off(LED3);  // 成功指示
    }
    else
    {
        BOARD_LED_On(LED3);   // 失败指示
    }
}
```

### 6.2 扫描所有CS通道（代码中已实现）

你的代码中已经有一个很好的扫描示例：

```c
#if 1  // 设置为1启用CS扫描
{
    uint8_t ch;
    uint8_t test_data = 0xAA;
    uint8_t read_data = 0;
    SPI_HandleTypeDef *hspi_scan;
    
    for (ch = 0; ch < 12; ch++)
    {
        hspi_scan = (ch < 6) ? &hspi1 : &hspi2;
        
        // 选中从设备
        DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_RESET);
        DAL_Delay(2);
        
        // 发送接收测试数据
        DAL_SPI_Transmit(hspi_scan, &test_data, 1, 100);
        DAL_SPI_Receive(hspi_scan, &read_data, 1, 100);
        
        // 释放从设备
        DAL_GPIO_WritePin(NSA2302_CS_PORT[ch], NSA2302_CS_PIN[ch], GPIO_PIN_SET);
        DAL_Delay(10);
        
        // 在这里设置断点，查看每个通道的read_data
    }
}
#endif
```

### 6.3 使用逻辑分析仪
- 监控SCK、MISO、MOSI信号
- 监控对应的CS信号
- 验证时序是否正确

## 7. 注意事项

1. **通道编号从0开始**：
   - 代码中使用 `ch = 0-11` 表示12个传感器
   - 对应物理CS引脚PC0-PC11

2. **SPI接口自动选择**：
   - `Spi_WriteReg()` 和 `Spi_ReadReg()` 会根据通道号自动选择SPI1或SPI2
   - ch 0-5 使用SPI1
   - ch 6-11 使用SPI2

3. **CS引脚时序**：
   - 函数内部已自动处理CS的拉低和拉高
   - 用户无需手动控制CS引脚

4. **设备数量配置**：
   - `nsa_device_num` 变量定义了实际使用的传感器数量
   - 代码中设置为12（最大值）

5. **读写命令格式**：
   - 写命令：0x00 + 寄存器地址 + 数据
   - 读命令：0x80 + 寄存器地址 + 接收数据
   - 0x80表示读操作（最高位为1）

## 8. 快速开始

最简单的使用方式：

```c
int main(void)
{
    // 系统初始化（已在代码中完成）
    DAL_DeviceConfig();
    
    // CS引脚已在GPIO配置中自动初始化
    
    // 写入传感器0
    Spi_WriteReg(0, 0x00, 0x24);
    
    // 读取传感器0
    uint8_t data;
    Spi_ReadReg(0, &data, 0x00);
    
    // 使用data...
}
```

就这么简单！函数会自动处理SPI选择和CS控制。
