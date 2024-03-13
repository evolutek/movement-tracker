#include "stm32g4xx_hal.h"
#include "time.h"

timestamp_t getCurrentTime(void)
{
    return HAL_GetTick();
}

timestamp_t getElapsedTime(timestamp_t a, timestamp_t b)
{
    if (a < b) return (b - a);
    else return (a - b);
}

bool isTimeDeltaElapsed(timestamp_t t, timestamp_t delta)
{
    return (getElapsedTime(t, getCurrentTime()) >= delta);
}

void waitFor(timestamp_t time)
{
    timestamp_t start = getCurrentTime();
    while (!isTimeDeltaElapsed(start, time));
}
