#include "Servo.h"

Servo::Servo() : _pin(-1), _last_us(1500), _attached(false) {}

int Servo::attach(int pin) {
  _pin = pin;
  _attached = true;
  return _pin;
}

void Servo::detach() {
  _attached = false;
  _pin = -1;
}

void Servo::write(int value) {
  // Interpret value as degrees (0-180). Convert to microseconds (1000-2000)
  if (value < 0) value = 0;
  if (value > 180) value = 180;
  _last_us = 1000 + (value * 1000) / 180;
}

void Servo::writeMicroseconds(int us) {
  _last_us = us;
}

int Servo::read() const {
  // Convert microseconds back to approximate degrees
  return ((_last_us - 1000) * 180) / 1000;
}

int Servo::readMicroseconds() const {
  return _last_us;
}

bool Servo::attached() const {
  return _attached;
}
