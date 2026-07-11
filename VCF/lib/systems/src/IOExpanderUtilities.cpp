#include "IOExpanderUtilities.h"


/*
 * Retrieves the bit from the data frame.
 * Port A = 0, and is the lower byte of data. Port B = 1, and is the higher byte of data.
 */
bool IOExpanderUtilities::getBit(uint16_t data, bool port, uint8_t bit)
{
    return (data >> ((uint16_t) port * 8 + bit)) & 1; // NOLINT 8 is num of bits in byte
}