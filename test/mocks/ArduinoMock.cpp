#include "Arduino.h"
#include <chrono>
#include <thread>

static auto _start = std::chrono::steady_clock::now();

class StdStream : public Stream {};

static StdStream _serial_instance;
Stream &Serial = _serial_instance;
Stream &Serial1 = _serial_instance;

unsigned long millis() {
  using namespace std::chrono;
  return (unsigned long)duration_cast<milliseconds>(steady_clock::now() - _start).count();
}

unsigned long micros() {
  using namespace std::chrono;
  return (unsigned long)duration_cast<microseconds>(steady_clock::now() - _start).count();
}

void delay(unsigned long ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void pinMode(int /*pin*/, int /*mode*/) { }
int digitalRead(int /*pin*/) { return 0; }
int analogRead(int /*pin*/) { return 0; }
