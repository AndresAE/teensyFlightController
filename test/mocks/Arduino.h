// Minimal Arduino API mock for host unit tests
#pragma once

#include <cstdint>
#include <string>
#include <iostream>

using byte = uint8_t;
using boolean = bool;

class Stream {
public:
  virtual Stream &print(const std::string &s) { std::cout << s; return *this; }
  virtual Stream &print(const char *s) { std::cout << s; return *this; }
  template<typename T>
  Stream &print(T v) { std::cout << v; return *this; }
  virtual Stream &println() { std::cout << std::endl; return *this; }
  template<typename T>
  Stream &println(T v) { std::cout << v << std::endl; return *this; }
};

// basic Arduino functions used by the project
extern Stream Serial;
extern Stream Serial1;

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void pinMode(int pin, int mode);
int digitalRead(int pin);
int analogRead(int pin);

#define INPUT 0
#define OUTPUT 1
