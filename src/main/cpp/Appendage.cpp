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
spatuclawSolenoid = new frc::DoubleSolenoid(1, 5, 6);
spatuclawOpenClose = new frc::DoubleSolenoid(1, 0, 1);
punchySolenoid = new frc::DoubleSolenoid(1, 2, 3);
LeftClaw = new frc::VictorSP(8);
LeftClaw->SetInverted(true);
RightClaw = new frc::VictorSP(9);
elevator = new frc::VictorSP(6);
}

void Appendage::spatuclawExtend() {

  spatuclawSolenoid->Set(frc::DoubleSolenoid::Value::kForward);

}

void Appendage::spatuclawRetract() {

  spatuclawSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);

}

void Appendage::spatuclawOpen() {
  spatuclawOpenClose->Set(frc::DoubleSolenoid::Value::kForward);
}

void Appendage::spatuclawClose() {
  spatuclawOpenClose->Set(frc::DoubleSolenoid::Value::kReverse);
}

void Appendage::punchyOut() {
  punchySolenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void Appendage::punchyIn() {
  punchySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}

void Appendage::spatuclawIn() {
  LeftClaw->Set(0.8);
  RightClaw->Set(0.8);
}

void Appendage::spatuclawOut() {
  LeftClaw->Set(-0.8);
  RightClaw->Set(-0.8);
}

void Appendage::spatuclawStop() {
  LeftClaw->Set(0);
  RightClaw->Set(0);
}

double Appendage::Threshold(double in,double thres){

  double out = in;
  if (in>thres){
    out = thres;
  }
  if(in<-1*thres){
    out = -1*thres;
  }
  return out;
}

void Appendage::elevator_joystick( double LeftStick) {
  LeftStick = LeftStick * LeftStick * LeftStick;
  LeftStick = Threshold(LeftStick, 0.9);
  elevator->Set(LeftStick);
}