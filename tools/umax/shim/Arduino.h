// Just enough Arduino for micro-Max (umax/engine) to build on the PC (umax_cli). millis() is the node clock in
// umax_cli.cpp. MSVC's long is 32 bits like the AVR's, so the hash keys come out the same.
#pragma once
#include <cstdint>
#include <cstring>
#define PROGMEM
uint32_t millis();
