/*
 *  supports max 14 channels in this lib (with messagelength of 0x20 there is room for 14 channels)

  Example set of bytes coming over the iBUS line for setting servos:
    20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
  Explanation
    Protocol length: 20
    Command code: 40
    Channel 0: DB 5  -> value 0x5DB
    Channel 1: DC 5  -> value 0x5Dc
    Channel 2: 54 5  -> value 0x554
    Channel 3: DC 5  -> value 0x5DC
    Channel 4: E8 3  -> value 0x3E8
    Channel 5: D0 7  -> value 0x7D0
    Channel 6: D2 5  -> value 0x5D2
    Channel 7: E8 3  -> value 0x3E8
    Channel 8: DC 5  -> value 0x5DC
    Channel 9: DC 5  -> value 0x5DC
    Channel 10: DC 5 -> value 0x5DC
    Channel 11: DC 5 -> value 0x5DC
    Channel 12: DC 5 -> value 0x5DC
    Channel 13: DC 5 -> value 0x5DC
    Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
 */
#include <stdio.h>

int main()
{
    __uint8_t buffer[] = {
        0x20, 0x40,
        0xDB, 0x05, // 1
        0xDC, 0x05, // 2
        0x54, 0x05, // 3
        0xDC, 0x05, // 4
        0xE8, 0x03, // 5
        0xD0, 0x07, // 6
        0xD2, 0x05, // 7
        0xE8, 0x03, // 8
        0xDC, 0x05, // 9
        0xDC, 0x05, // 10
        0xDC, 0x05, // 11
        0xDC, 0x05, // 12
        0xDC, 0x05, // 13
        0xDC, 0x05, // 14
        0xDA, 0xF3, // CHECKSUM
    };

    __uint16_t storedChecksum = buffer[31] << 8 | buffer[30];
    __uint16_t checksum = 0xffff; // Initialize to 0xFFFF

    for (int i = 0; i < 30; i++)
    {
        checksum -= buffer[i];
    }

    printf("Size Of Buffer : %ld\n", sizeof(buffer) / sizeof(__uint8_t));
    printf("Calculated Checksum : %d (0x%X)\n", checksum, checksum);
    printf("Stored Checksum : %d (0x%X)\n", storedChecksum, storedChecksum);
    return 0;
}
