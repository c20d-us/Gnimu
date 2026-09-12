#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
unsigned long millis();

// Arduino's Stream, as far as this firmware uses it: g_gnss_port.h hands one
// back and the GNSS driver drains it. Defined here, in the base fake, so every
// harness sees exactly one definition.
class Stream {
public:
  virtual ~Stream() {}
  virtual int available() = 0;
  virtual int read() = 0;
};
void delay(unsigned long ms);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int val);
int digitalRead(int pin);
// Board pin names the real config.h files reference. Values are irrelevant.
#define D0 0
#define D1 1
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define D8 8
#define D9 9
#define D10 10
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define PIN_LSM6DS3TR_C_POWER 40
#define PIN_LSM6DS3TR_C_INT1 41
#define PIN_VBAT 42
#define VBAT_ENABLE 43
#define PIN_CHARGING_CURRENT 44
#define LED_RED 45
#define LED_GREEN 46
#define LED_BLUE 47
#define PIN_WIRE_SDA 48
#define PIN_WIRE_SCL 49
