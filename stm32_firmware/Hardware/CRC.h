#ifndef __CRC_H
#define __CRC_H

#include <stdint.h>

// 函数声明：计算Modbus CRC-16
uint16_t Calculate_CRC16(uint8_t *data, uint16_t length);

#endif
