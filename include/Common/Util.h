#pragma once

#include <cinttypes>

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])
#define NAVIO2 3
#define NAVIO 1

int write_file(const char* path, const char* fmt, ...);
int read_file(const char* path, const char* fmt, ...);
bool check_apm();
int get_navio_version();

/* Decode IEEE 754 single precision floating point number. */
float decodeBinary32(uint32_t bin);
