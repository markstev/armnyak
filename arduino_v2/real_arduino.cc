#include "Arduino.h"
#include "real_arduino.h"

RealArduino::~RealArduino() {}

void RealArduino::digitalWrite(const unsigned int pin, bool value) {
  return ::digitalWrite(pin, value);
}

bool RealArduino::digitalRead(const unsigned int pin) {
  return ::digitalRead(pin);
}

unsigned long RealArduino::micros() {
  return ::micros();
}

void RealArduino::write(const unsigned char c) {
  Serial.write(c);
}

int RealArduino::read() {
  return Serial.read();
}

bool RealArduino::available() {
  return Serial.available();
}
