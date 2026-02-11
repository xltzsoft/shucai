#ifndef AS5048A_H
#define AS5048A_H

#include <Arduino.h>
#include <SPI.h>

// AS5048A 寄存器地址
#define AS5048A_REG_NOP       0x0000  // 空操作
#define AS5048A_REG_CLEAR_ERR 0x0001  // 清除错误标志
#define AS5048A_REG_PROG_CTRL 0x0003  // 编程控制
#define AS5048A_REG_OTP_HIGH  0x0016  // OTP 高字节
#define AS5048A_REG_OTP_LOW   0x0017  // OTP 低字节
#define AS5048A_REG_DIAG_AGC  0x3FFD  // 诊断 + AGC 值
#define AS5048A_REG_MAGNITUDE 0x3FFE  // CORDIC 幅度
#define AS5048A_REG_ANGLE     0x3FFF  // 角度值（14位）

#define AS5048A_RESOLUTION    16384   // 2^14

class AS5048A {
public:
    /**
     * @brief 构造函数
     * @param csPin 片选引脚
     */
    AS5048A(uint8_t csPin);

    /**
     * @brief 初始化 SPI 及 AS5048A
     * @param spiPort SPI 对象引用（默认为 SPI）
     * @param sckPin  SCK 引脚
     * @param misoPin MISO 引脚
     * @param mosiPin MOSI 引脚
     */
    void begin(SPIClass &spiPort = SPI, int8_t sckPin = -1, int8_t misoPin = -1, int8_t mosiPin = -1);

    /**
     * @brief 获取原始角度值 (0 ~ 16383)
     * @return 14 位原始角度值
     */
    uint16_t getRawAngle();

    /**
     * @brief 获取角度值（度数，0.0 ~ 360.0）
     * @return 角度值，单位：度
     */
    float getAngleDegrees();

    /**
     * @brief 获取诊断信息和 AGC 值
     * @return 诊断 + AGC 寄存器原始值
     */
    uint16_t getDiagnostics();

    /**
     * @brief 获取 CORDIC 幅度值
     * @return 幅度寄存器原始值
     */
    uint16_t getMagnitude();

    /**
     * @brief 检查是否有错误
     * @return true 表示有错误
     */
    bool hasError();

    /**
     * @brief 清除错误标志
     */
    void clearError();

private:
    uint8_t  _csPin;
    SPIClass *_spi;
    bool     _errorFlag;

    /**
     * @brief 读取寄存器
     * @param reg 寄存器地址（14位）
     * @return 寄存器值（14位数据）
     */
    uint16_t readRegister(uint16_t reg);

    /**
     * @brief 发送 SPI 16 位帧并接收响应
     * @param cmd 16 位命令帧
     * @return 16 位响应帧
     */
    uint16_t spiTransfer(uint16_t cmd);

    /**
     * @brief 计算偶校验位
     * @param value 要计算校验的值
     * @return 偶校验位（0 或 1）
     */
    static uint8_t calcEvenParity(uint16_t value);
};

#endif // AS5048A_H
