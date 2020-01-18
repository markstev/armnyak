#ifndef real_arduino_h_
#define real_arduino_h_

#include <includes/arduinoio3/arduino.h>

class RealArduino : public arduinoio::ArduinoInterface {
 public:
   ~RealArduino();

  void digitalWrite(const unsigned int pin, bool value) override;
  bool digitalRead(const unsigned int pin) override;
  unsigned long micros() override;

  void write(const unsigned char c) override;
  int read() override;
  bool available() override;
};

#endif // real_arduino_h_
