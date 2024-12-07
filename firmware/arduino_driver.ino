// Includes required to use Roboclaw library
#include "RoboClaw.h"

// Use HardwareSerial (Serial1 on Uno R4 Minima)
RoboClaw roboclaw(&Serial1, 38400);

#define address 0x80

// Velocity PID coefficients
#define Kp 1.85
#define Ki 0.26
#define Kd 0.0
#define qpps 9937

enum MOTOR
{
  LEFT,
  RIGHT
};

void setup() {
  // Open Serial for debugging
  Serial.begin(115200);

  // Open Serial1 for RoboClaw communication
  Serial1.begin(38400);

  // Set PID Coefficients
  roboclaw.SetM1VelocityPID(address, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address, Kd, Kp, Ki, qpps);  
}

void command_motor(const MOTOR &motor, uint32_t speed)
{
  if(MOTOR::RIGHT == motor)
  {
   roboclaw.SpeedM1(address, speed);
  }
  
  if(MOTOR::LEFT == motor)
  {
    roboclaw.SpeedM2(address, speed);
  }
}

void loop() {
  if(Serial.available() >= 2) 
  {
    String command = Serial.readStringUntil('\n');
    int comma_index = command.indexOf(',');

    int32_t left_32bit = command.substring(0,comma_index).toInt();
    int32_t right_32bit = command.substring(comma_index+1).toInt();

    command_motor(MOTOR::LEFT, left_32bit);
    command_motor(MOTOR::RIGHT,right_32bit);
  }
}
