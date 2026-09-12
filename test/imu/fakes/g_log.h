// Logs go to stderr so they never enter the compared output.
#pragma once
#include <stdio.h>
#define LOG_PRINT(...) fprintf(stderr, "%s", __VA_ARGS__)
#define LOG_PRINTLN(s) fprintf(stderr, "%s\n", s)
#define LOG_PRINTF(...) fprintf(stderr, __VA_ARGS__)
#define LOG_FLUSH()
