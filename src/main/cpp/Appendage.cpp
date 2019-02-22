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
#include "frc\Encoder.h"

using namespace std;

Appendage::Appendage() : Subsystem("Appendage") {
//Leftdrive = new frc::VictorSP(0);
spatuclawSolenoid = new frc::DoubleSolenoid(1, 0, 1);
spatuclawOpenClose = new frc::DoubleSolenoid(1, 2, 3);
punchySolenoid = new frc::DoubleSolenoid(1, 4, 5);
extendSolenoid = new frc::DoubleSolenoid(1, 7, 8);
LeftClaw = new frc::VictorSP(8);
LeftClaw->SetInverted(true);
RightClaw = new frc::VictorSP(9);
elevator = new frc::VictorSP(6);
elevator_encoder = new frc::Encoder( 6, 7, false, frc::Encoder::k4X);
}

void Appendage::spatuclawExtend() {

  spatuclawSolenoid->Set(frc::DoubleSolenoid::Value::kForward);

}

void Appendage::spatuclawRetract() {

  spatuclawSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);

}

bool Appendage::spatuclawOpen() {
  spatuclawOpenClose->Set(frc::DoubleSolenoid::Value::kForward);
  return true;
}

bool Appendage::spatuclawClose() {
  spatuclawOpenClose->Set(frc::DoubleSolenoid::Value::kReverse);
  return false;
}

void Appendage::punchyOut() {
  punchySolenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void Appendage::punchyIn() {
  punchySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}

void Appendage::extensionOut() {
  extendSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void Appendage::extensionIn() {
  extendSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
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

double Appendage::Deadband(double in, double thres){
  double out = in;
  if (in<thres and in>-1*thres){
    out = 0;
  }
  
  return out;
}

void Appendage::elevator_joystick( double LeftStick) {
  LeftStick = LeftStick * LeftStick * LeftStick;
  LeftStick = Threshold(LeftStick, 0.9);
  elevator->Set(LeftStick);
  double encoder_val = elevator_encoder->Get();
  auto encoder_valstr = std::to_string(encoder_val);
  frc::SmartDashboard::PutString("DB/String 3",encoder_valstr);
}

void Appendage::elevator_PID(double setpoint) {
  double encoder_val = elevator_encoder->Get();
  double error = setpoint - encoder_val;
  error = Deadband(error, 100);
  double kpe = .025;
  double output_e = error * kpe;
  output_e = Threshold(output_e, 0.9);
  elevator->Set(output_e);
  auto encoder_valstr = std::to_string(encoder_val);
  frc::SmartDashboard::PutString("DB/String 3",encoder_valstr);
}

void Appendage::Dashboard(){

  double elevator_encoder_val = elevator_encoder->Get();

  auto elevator_encoderstr = std::to_string(elevator_encoder_val);
  frc::SmartDashboard::PutString("Elevator Encoder",elevator_encoderstr);
 
}