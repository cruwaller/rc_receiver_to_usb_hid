#pragma once
#include <stdint.h>

uint8_t CalcCRC(uint8_t data, uint8_t crc);
uint8_t CalcCRClen(uint8_t const *data, uint16_t length, uint8_t crc);

uint8_t CalcCRC8(uint8_t const data, uint8_t crc, uint8_t const poly);
uint8_t CalcCRC8len(uint8_t const *data, uint16_t length,
                    uint8_t crc, uint8_t const poly);
