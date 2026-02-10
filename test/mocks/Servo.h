#pragma once

class Servo {
public:
  Servo();
  int attach(int pin);
  void detach();
  void write(int value); // degrees or microseconds depending on usage
  void writeMicroseconds(int us);
  int read() const;
  int readMicroseconds() const;
  bool attached() const;
private:
  int _pin;
  int _last_us;
  bool _attached;
};
