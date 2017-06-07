# vcQuadcopter
Quadcopter Project based on STM32F4 Discovery Board

1- The software is developed in IAR Embedded Workbench.

2- Software reads the sensors on AltIMU-10 v5 which are LSM6DS33 (Gyro, Accelerometer), LIS3MDL (Compass) and LPS25H (Altimeter)

3- Drives the ESCs connected to 4 brushless DC motors.

4- Receives 4 different Transmitter PWM inputs which are to control Throttle (moving the copter up and down), Yaw (rotating the copter right or left), Pitch (moving the copter forward and backward) and Roll (moving the copter side to side).

5- Applies PID Control by X Flight Orientation

 CW motors    RearRight,FronLeft
 CCW motors   FrontRight,RearLeft

           Front
           +1 pitch
    FronLeft  FrontRight
 -1 roll  \\-//     +1 roll   right
          //-\\
    RearLeft  RearRight
           -1 pitch
		   
6- Motor Control Eqautions

    FrontLeft   = throttle + pitch + roll - yaw
    FrontRight  = throttle + pitch - roll + yaw
    RearLeft    = throttle - pitch + roll + yaw
    RearRight   = throttle - pitch - roll - yaw
    
    Pitch Down (forward flight):
      FrontLeft Motor and FrontRight Motor decrease thrust
      RearLeft Motor and RearRight Motor increase thrust
    Pitch Up (back flight):
      FrontLeft Motor and FrontRight Motor increase thrust
      RearLeft Motor and RearRight Motor decrease thrust
    Roll Left (left flight):
      FrontRight Motor and RearLeft Motor increase thrust
      FrontLeft Motor and RearRight Motor decrease thrust
    Roll Right (right flight):
      FrontRight Motor and RearLeft Motor decreases thrust
      FrontLeft Motor and RearRight Motor increases thrust
    Yaw Left (turn left):
      FrontRight Motor and RearRight Motor increase thrust
      FrontLeft Motor and RearLeft Motor decrease thrust
    Yaw Right (turn right):
      FrontRight Motor and RearRight Motor decrease thrust
      FrontLeft Motor and RearLeft Motor increase thrust