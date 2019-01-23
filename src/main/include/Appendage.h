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
#include "frc\Encoder.h"

using namespace std;

class Appendage : public frc::Subsystem {

 private:
 //Declare Motor and sensors used in appendage functions here.
 frc::DoubleSolenoid * spatuclawSolenoid;
 frc::DoubleSolenoid * spatuclawOpenClose;
 frc::DoubleSolenoid * punchySolenoid;
 frc::VictorSP * LeftClaw;
 frc::VictorSP * RightClaw;
 frc::VictorSP * elevator;
 frc::Encoder * elevator_encoder;
  //frc::VictorSP * Rightclimb;

 public:
  Appendage();
  void spatuclawRetract();
  void spatuclawExtend();
  void spatuclawOpen();
  void spatuclawClose();
  void punchyIn();
  void punchyOut();
  void spatuclawIn();
  void spatuclawOut();
  void spatuclawStop();
  void elevator_joystick(double LeftStick);
  double Threshold(double in,double thres);
  void elevator_PID(double setpoint);
  double Deadband(double in, double thres);
// User Wrtitten Functions Definitions here.
 
};
#endif 