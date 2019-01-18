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
Leftdrive->SetInverted(true);
Rightdrive = new frc::VictorSP(1);
Leftclimb = new frc::VictorSP(4);
Rightclimb = new frc::VictorSP(5);
Leftclimb->SetInverted(true);

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
/*
float Drive::Read_in_camera_VT() { // vision target 

  std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
  table->PutNumber("ledMode", 0);
  table->PutNumber("camMode", 0);
  table->PutNumber("pipeline", 1); //Vision Target pipeline

  float camera_x = table->GetNumber("tx", 0);
  float camera_exist = table->GetNumber("tv", 0);
  float image_size = table->GetNumber("ta", 0);

  return camera_x;
}*/


double Drive::Threshold(double in,double thres){

  double out = in;
  if (in>thres){
    out = thres;
  }
  if(in<-1*thres){
    out = -1*thres;
  }
  return out;
}

void Drive::Camera_Centering(double Leftstick, float camera_x){

  double error = 0 - camera_x;
  double kp_c = .1;
  double output = kp_c * error;
  Leftstick = Threshold(Leftstick,0.75);

  Leftdrive->Set(Leftstick+output);
  Rightdrive->Set(Leftstick-output);
}

//Climber code
void Drive::Climb_Extend(bool button_lb, bool button_rb, bool button_start, bool button_back){

  if (button_lb && button_rb){

    Leftclimb->Set(1);
    Rightclimb->Set(1);
  }
  else if (button_start && button_back){
    Leftclimb->Set(-.5);
    Rightclimb->Set(-.5);
  }
  else{
    Leftclimb->Set(0);
    Rightclimb->Set(0);
  }

}