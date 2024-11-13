#include <stdio.h>
#include <string.h>
#include <math.h>

int main()
{
    //    Received: UT: 27171, X1: 7465, X2: -2384, B5: 5081, Calculated Temp:  C
    __int32_t UT = 27171;
    __int32_t X1 = 7465;
    __int32_t X2 = -2384;
    __int32_t B5 = 5081;
    float temperature = ((B5 + 8) >> 4) / 10.0;
    float temperature1 = ((B5 + 8) / pow(2, 4)) / 10.0;
    printf("%d\n", (((12 * 4 + 34) << 3) + 2) >> 2);
    printf("%d\n", (((12 * 4 + 34) << 3) + 2) / 4);
}