#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif 
#include <Wire.h>  // Needed by cmake to generate the pressure sensor deps. (Gross!)

#include "../arduinoio/lib/uc_module.h"
#include "../arduinoio/lib/serial_module.h"
#include "../arduinoio/lib/arduinoio.h"
//#include "led_module.h"
#include "motor_bank_module.h"

const int SERIAL_RX_PIN = 0;
const int SERIAL_TX_PIN = 1;
const int LED_SIGNAL_PIN = 51;

arduinoio::ArduinoIO arduino_io;
void setup() {
  const uint16_t kNumLeds = 300;
  //arduino_io.Add(new nebree8::LedModule(kNumLeds, LED_SIGNAL_PIN));
  Serial.begin(9600);
  arduino_io.Add(new arduinoio::SerialRXModule(0));
  arduino_io.Add(new armnyak::MotorBankModule());
}

void loop() {
  arduino_io.HandleLoopMessages();
}
