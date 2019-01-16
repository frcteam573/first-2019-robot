/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc\Joystick.h"
#include <WPILib.h>

class Drive : public frc::Subsystem {
 private:

  frc::Talon Leftdrive { 0 };
  frc::Talon Rightdrive { 1 };
  frc::Talon Rightclimb { 4 };
  frc::Talon Leftclimb { 5 };
  //frc::Encoder Leftdrive_encoder { 2, 3, false, Encoder::k4X};
  //frc::Encoder Rightdrive_encoder { 0, 1, false, Encoder::k4X};
  frc::ADXRS450_Gyro MyGyro{};
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  Drive();
  void InitDefaultCommand() override;
  
  void Joystick_drive(double LeftStick,double RightStick);
  
  

};
