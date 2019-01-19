/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc\Joystick.h"
#include "frc\WPILib.h"
#include "frc\VictorSP.h"
#include "frc\DoubleSolenoid.h"
#include "Appendage.h"

using namespace std;

Appendage::Appendage() : Subsystem("Appendage") {
//Leftdrive = new frc::VictorSP(0);
//spatuclawSolenoid = new frc::DoubleSolenoid(1, 0, 1);
}

void Appendage::spatuclawExtend() {

//  spatuclawSolenoid->Set(frc::DoubleSolenoid::Value::kForward);

}

void Appendage::spatuclawRetract() {

  //spatuclawSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);

}

void Appendage::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}