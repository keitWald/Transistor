#include "vofa.h"
#include <string.h>
/* Pure JustFloat packing only. UART7 TX/RX and commands live in VOFA_Task.c. */
uint16_t Vofa_PackJustFloat(uint8_t output[64], const float *data, uint8_t count)
{
    if (output == NULL || data == NULL || count == 0 || count > 15) return 0;
    memcpy(output, data, (size_t)count * sizeof(float));
    const uint8_t tail[4] = {0, 0, 0x80, 0x7F};
    memcpy(output + count * sizeof(float), tail, sizeof(tail));
    return (uint16_t)(count * sizeof(float) + sizeof(tail));
}

