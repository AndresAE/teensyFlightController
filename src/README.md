![image not loaded](../doc/Stability_vs_Body_Axis.png)

# flightControlTeensy

## 6DOF Flight Controller for use with:
- Teensy 4.0
- BNO 005
- Adafruit GPS Breakout v3
- Telemetry Radio (optional hardwire)

## To-Do:
- [x] Implement MSN
- [x] Check loop timing and optimize
- [x] Calibrate limits and gains
- [x] Add visualization tool
- [x] add PyQtGraph capabilites to visualization
- [x] recolor PyQtGraph
- [x] pinInterrupt for pulseIn
- [x] pinInterup for servo commands
- [x] Check servos
- [x] Add config file for easy parameter changes
- [x] check TM with new architecture
- [x] hardware abstraction layer to make IMU, GPS, ans servo independent
- [ ] zero protected divides, common?
- [ ] initialize APT params based on effector size
- [ ] initialize GDC cpp params based on input size
- [c] kalman filter for bad intermittent GPS
- [ ] time of validity for IMU, GPS, GDC commands
- [ ] 'Direct' law for GDC commands
- [ ] consistent matrix naming in common
- [ ] check transformation math NAV
- [ ] check ecef2lla math in common
- [ ] minimize instances of variables in nav
- [ ] fix method instance issue
