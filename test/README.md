Making tests that depend on Arduino APIs (host build)

Build (POSIX / WSL / MSYS):

```
g++ -std=c++17 -Itest/mocks -Isrc $(find src -name '*.cpp' -print) test/aptTest.cpp -o test_app
```

Build (PowerShell):

```
$cpps = Get-ChildItem -Recurse -Filter *.cpp -Path src | ForEach-Object { $_.FullName }
g++ -std=c++17 -Itest/mocks -Isrc $cpps test/aptTest.cpp -o test_app
```

Notes:
- The mock `Arduino.h` in `test/mocks` provides minimal `Stream`, `Serial`, `millis()`, `micros()`, `delay()` and pin stubs.
- Add more functions to the mock if you hit undefined symbols when building.
- This builds for your host machine (x86). It is not a Teensy/ARM build.
