/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// It's desirable that everything possible under private except
// for methods that implement subsystem capabilities

#ifndef Appendage_H
#define Appendage_H

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc\Joystick.h"
#include "frc\WPILib.h"
#include "frc\VictorSP.h"
#include "frc\DoubleSolenoid.h"

using namespace std;

class Appendage : public frc::Subsystem {

 private:
 //Declare Motor and sensors used in appendage functions here.
 // frc::DoubleSolenoid * spatuclawSolenoid;
  //frc::VictorSP * Rightclimb;

 public:
  Appendage();
  void InitDefaultCommand() override;
  void spatuclawRetract();
  void spatuclawExtend();
// User Wrtitten Functions Definitions here.
 
};
#endif 