/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// It's desirable that everything possible under private except
// for methods that implement subsystem capabilities

#ifndef Drive_H
#define Drive_H

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc\Joystick.h"
#include "frc\WPILib.h"
#include "frc\VictorSP.h"
#include "frc\Encoder.h"
#include "frc\AnalogInput.h"
#include "frc\AnalogOutput.h"
#include "frc\AnalogGyro.h"
#include "frc\Compressor.h"
#include "frc\DoubleSolenoid.h"
using namespace std;

class Drive : public frc::Subsystem {

 private:
frc::VictorSP * Leftdrive;
frc::VictorSP * Rightdrive;
frc::AnalogOutput * Leds;
frc::AnalogGyro * Gyro;
frc::Compressor * Compressor;
frc::DoubleSolenoid * frontclimbSolenoid;
frc::DoubleSolenoid * backclimbSolenoid;
double Threshold(double in,double thres);

  frc::VictorSP * Trollyclimb;
  //frc::VictorSP * Rightclimb;
  frc::Encoder * Right_encoder;
  frc::Encoder * Left_encoder;

frc::AnalogInput * FrontDistance;

 public:
  Drive();
  void InitDefaultCommand() override;
// User Wrtitten Functions Definitions here.
  void Joystick_drive(double LeftStick,double RightStick);

  bool Camera_Centering(double Leftstick, float camera_x);
  
  bool Camera_Centering_Distance(float camera_x, float camera_size);

  void Climb_Extend(bool button_lb, bool button_rb, double leftjoystick);

  void drive_PID(double setpoint_left_pos, double setpoint_right_pos, double setpoint_left_speed, double setpoint_right_speed, double heading, int count);

  void encoder_drive(double setpoint, int count);

  void gyro_drive(double setpoint);

  bool platform_adjust();

  void OrangeLeds();
  void PartyLeds();
  void WhiteLeds();
  void BlueLeds();
  void YellowLeds();
  void OffLeds();

  void Dashboard();
};
#endif 