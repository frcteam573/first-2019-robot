/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc\Joystick.h"
#include "frc\WPILib.h"
#include "frc\VictorSP.h"
#include "Drive.h"
using namespace std;

Drive::Drive() : Subsystem("Drive") {
Leftdrive = new frc::VictorSP(0);
Rightdrive = new frc::VictorSP(1);

}

void Drive::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}


void Drive::Joystick_drive(double LeftStick,double RightStick) {

// Set motor values
LeftStick = LeftStick * LeftStick * LeftStick;
RightStick = RightStick * RightStick * RightStick;

Leftdrive->Set(LeftStick);
Rightdrive->Set(RightStick);

// Convert double to strings
  auto leftinstr = std::to_string(LeftStick);
  auto rightinstr = std::to_string(RightStick);

// Push string values to Dashboard
  frc::SmartDashboard::PutString("DB/String 0",leftinstr);
  frc::SmartDashboard::PutString("DB/String 1",rightinstr);


}