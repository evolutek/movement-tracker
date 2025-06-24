#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t timestamp_t;

timestamp_t getCurrentTime(void);
timestamp_t getElapsedTime(timestamp_t a, timestamp_t b);
bool isTimeDeltaElapsed(timestamp_t t, timestamp_t delta);
void waitFor(timestamp_t time);

#endif  /* TIMESTAMP_H */
