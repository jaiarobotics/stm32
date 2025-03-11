/*
 * crc32.c
 *
 *  Created on: Mar 11, 2025
 *      Author: MatthewFerro
 */

#include "crc32.h"

// CRC32 lookup table
static uint32_t crc32_table[256];

// Initializes the CRC32 lookup table (call once)
void init_crc32_table(void) {
    uint32_t crc;
    for (uint32_t i = 0; i < 256; i++) {
        crc = i;
        for (uint32_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
        crc32_table[i] = crc;
    }
}

// Computes CRC32
uint32_t compute_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crc32_table[index];
    }
    return crc ^ 0xFFFFFFFF;
}
