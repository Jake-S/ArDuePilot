ArDuePilot
==========

Arduino Due based multirotor autopilot software.


Installation Guide:

REQUIRES ARDUINO IDE 1.5.4+ (interrupts now work right...)

/Arduino_Due_OFP/Arduino_Due_OFP.ino gets installed on main Arduino Due

/ArduCAM_OSDnoMav/ArduCAM_OSDnoMav.ino gets installed on ATmega328 chip on the minimOSD

/IMU_Razor_Code/IMU_Razor_Code.ino gets installed on the ATmega328 chip on the 9DOF Razor IMU

/test2/test2.ino is test scripts to send data to OSD, will eventually get put into Arduino Due OFP once fully working


ISSUES:
I have the arduino sending telemetry/OSD stream using my own incredibly simple protocal. The plan is to get Mavlink ported over to the ARM processor which the main issue is the stupid FastSerial library that they use. Either modify FastSerial or just get it to use the standard arduino serial library instead.

IMU kalman filter upgrade to EKF or UKF, also implement GPS into it.
