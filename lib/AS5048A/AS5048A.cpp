#include "AS5048A.h"

// SPI 时钟频率 1MHz（AS5048A 最高支持 10MHz，1MHz 更稳定）
#define AS5048A_SPI_SPEED 1000000

AS5048A::AS5048A(uint8_t csPin)
    : _csPin(csPin), _spi(nullptr), _errorFlag(false)
{
}

void AS5048A::begin(SPIClass &spiPort, int8_t sckPin, int8_t misoPin, int8_t mosiPin)
{
    _spi = &spiPort;

    // 配置 CS 引脚
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    // 如果指定了自定义引脚，则使用自定义引脚初始化 SPI
    if (sckPin >= 0 && misoPin >= 0 && mosiPin >= 0) {
        _spi->begin(sckPin, misoPin, mosiPin, -1);  // CS 由软件控制
    } else {
        _spi->begin();
    }

    // 发送一次空操作以清空通信管线
    readRegister(AS5048A_REG_NOP);
    // 清除可能存在的错误标志
    clearError();
}

uint16_t AS5048A::getRawAngle()
{
    return readRegister(AS5048A_REG_ANGLE);
}

float AS5048A::getAngleDegrees()
{
    uint16_t raw = getRawAngle();
    return (float)raw * 360.0f / (float)AS5048A_RESOLUTION;
}

uint16_t AS5048A::getDiagnostics()
{
    return readRegister(AS5048A_REG_DIAG_AGC);
}

uint16_t AS5048A::getMagnitude()
{
    return readRegister(AS5048A_REG_MAGNITUDE);
}

bool AS5048A::hasError()
{
    return _errorFlag;
}

void AS5048A::clearError()
{
    // 读取清除错误寄存器
    readRegister(AS5048A_REG_CLEAR_ERR);
    _errorFlag = false;
}

uint16_t AS5048A::readRegister(uint16_t reg)
{
    // 构建读命令帧：bit14 = 1 (读), bit13:0 = 寄存器地址
    uint16_t cmd = reg | 0x4000;  // 设置读标志
    cmd |= ((uint16_t)calcEvenParity(cmd) << 15);  // 设置偶校验位

    // 第一次传输：发送读命令
    spiTransfer(cmd);

    // 第二次传输：发送 NOP 获取数据
    uint16_t nopCmd = AS5048A_REG_NOP | 0x4000;  // NOP 读
    nopCmd |= ((uint16_t)calcEvenParity(nopCmd) << 15);
    uint16_t response = spiTransfer(nopCmd);

    // 检查错误标志（bit14）
    if (response & 0x4000) {
        _errorFlag = true;
        return 0;
    }

    // 返回 14 位数据（bit13:0）
    return response & 0x3FFF;
}

uint16_t AS5048A::spiTransfer(uint16_t cmd)
{
    // SPI Mode 1: CPOL=0, CPHA=1
    _spi->beginTransaction(SPISettings(AS5048A_SPI_SPEED, MSBFIRST, SPI_MODE1));
    digitalWrite(_csPin, LOW);
    
    // 发送高字节再发送低字节（16位传输）
    uint8_t highByte = _spi->transfer((cmd >> 8) & 0xFF);
    uint8_t lowByte  = _spi->transfer(cmd & 0xFF);

    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    return ((uint16_t)highByte << 8) | lowByte;
}

uint8_t AS5048A::calcEvenParity(uint16_t value)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (value & (1 << i)) {
            cnt++;
        }
    }
    return cnt & 0x01;  // 返回 0 表示已是偶数个1，返回 1 表示需要添加1
}
