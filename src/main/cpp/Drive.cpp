/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Drive.h"
#include "Robot.h"
#include "frc\Joystick.h"
#include <WPILib.h>

Drive::Drive() : Subsystem("Drive") {}

void Drive::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void Drive::Joystick_drive(double LeftStick,double RightStick) {


 

  
  Leftdrive.Set(LeftStick);

  Rightdrive.Set(RightStick);
  auto leftinstr = std::to_string(LeftStick);
  auto rightinstr = std::to_string(RightStick);

frc::SmartDashboard::PutString("DB/String 0",leftinstr);
 frc::SmartDashboard::PutString("DB/String 1",rightinstr);

}

// Put methods for controlling this subsystem
// here. Call these from Commands.
