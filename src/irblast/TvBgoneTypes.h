#pragma once

#include <Arduino.h>
#include <pgmspace.h>

// Oryginalna baza TV-B-Gone liczona byla dla AVR 16 MHz.
// Zachowujemy to samo mapowanie, aby wartosci timer_val pasowaly do danych.
#define freq_to_timerval(x) ((16000000UL / 8UL / (x)) - 1UL)
#define NUM_ELEM(x) (sizeof(x) / sizeof(*(x)))

struct IrCode {
  uint8_t timer_val;
  uint8_t numpairs;
  uint8_t bitcompression;
  const uint16_t *times;
  const uint8_t *codes;
};
