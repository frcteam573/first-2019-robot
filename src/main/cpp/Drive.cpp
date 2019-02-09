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
#include "frc\Encoder.h"
#include "frc\AnalogInput.h"
#include "frc\AnalogOutput.h"
#include "frc\ADXRS450_Gyro.h"
#include "frc\SPI.h"
#include "frc\Compressor.h"

using namespace std;

double leftdriveold;
double rightdriveold;
Drive::Drive() : Subsystem("Drive") {
Leftdrive = new frc::VictorSP(1);
Leftdrive->SetInverted(true);
Rightdrive = new frc::VictorSP(0);
Leftclimb = new frc::VictorSP(4);
Rightclimb = new frc::VictorSP(5);
Leftclimb->SetInverted(true);
Left_encoder = new frc::Encoder( 2, 3, false, frc::Encoder::k4X);
Right_encoder = new frc::Encoder( 0, 1, false, frc::Encoder::k4X);
FrontDistance = new frc::AnalogInput(2);
Leds = new frc::AnalogOutput(0);
Gyro = new frc::ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);
Compressor = new frc::Compressor(1);
}

void Drive::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}


void Drive::Joystick_drive(double LeftStick,double RightStick) {

// Set motor values
LeftStick = LeftStick * LeftStick * LeftStick;
RightStick = RightStick * RightStick * RightStick;

if (LeftStick > (leftdriveold + 0.3)){
  LeftStick = leftdriveold + 0.3;
}
else if (LeftStick < (leftdriveold - 0.3)){
  LeftStick = leftdriveold - 0.3;
}

if (RightStick > (rightdriveold + 0.3)){
  RightStick = rightdriveold + 0.3;
}
else if (RightStick < (rightdriveold - 0.3)){
  RightStick = rightdriveold - 0.3;
}

Leftdrive->Set(LeftStick);
Rightdrive->Set(RightStick);
leftdriveold = LeftStick;
rightdriveold = RightStick;

if (abs(LeftStick) > 0.8 and abs(RightStick) > 0.8){
  Compressor->Stop();
}
else {
  Compressor->Start();
}
// Convert double to strings
  auto leftinstr = std::to_string(LeftStick);
  auto rightinstr = std::to_string(RightStick);

// Push string values to Dashboard
  frc::SmartDashboard::PutString("DB/String 0",leftinstr);
  frc::SmartDashboard::PutString("DB/String 1",rightinstr);

double AnalogIn = FrontDistance->GetVoltage();
auto Analoginstr = std::to_string(AnalogIn);
frc::SmartDashboard::PutString("DB/String 9",Analoginstr);

}

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

bool Drive::Camera_Centering(double Leftstick, float camera_x){

  double error = 0 - camera_x;
  double kp_c = .025;
  double output = kp_c * error;
  Leftstick = Threshold(Leftstick,0.75);

  Leftdrive->Set(Leftstick+output);
  Rightdrive->Set(Leftstick-output);
  

  double AnalogIn = FrontDistance->GetVoltage();
  bool distance_tf = false;
  if (AnalogIn < 0.7){
    distance_tf = true;
  }
  else {
    distance_tf = false;
  }
  return distance_tf;
}

bool Drive::Camera_Centering_Distance( float camera_x, float camera_size){

  double error = 0 - camera_x;
  double kp_c = .025;
  double output = kp_c * error;
  
  double image_size_max = 3;
  double error_size = camera_size-(image_size_max+2);
  

  double AnalogIn = FrontDistance->GetVoltage();
  auto Analoginstr = std::to_string(AnalogIn);
  frc::SmartDashboard::PutString("DB/String 9",Analoginstr);
  double output_image;
  if (camera_size > image_size_max or camera_size == 0){
    error_size = AnalogIn - 0.67;
    double k_image = -.35;
    output_image = k_image * error_size;
  }
  else {
    double k_image = .09;
    output_image = k_image * error_size;
    
  }
  output_image = Threshold(output_image,0.95);

  Leftdrive->Set(output_image+output);
  Rightdrive->Set(output_image-output);
  bool distance_tf = false;
  if (AnalogIn < 0.7){
    distance_tf = true;
  }
  else {
    distance_tf = false;
  }
  return distance_tf;
}

//Climber code
void Drive::Climb_Extend(bool button_lb, bool button_rb, bool button_start, bool button_back){

  if (button_lb && button_rb){

    Leftclimb->Set(1);
    Rightclimb->Set(1);
    Compressor->Stop();
  }
  else if (button_start && button_back){
    Leftclimb->Set(-.5);
    Rightclimb->Set(-.5);
  }
  else{
    Leftclimb->Set(0);
    Rightclimb->Set(0);
    Compressor->Start();
  }


}

//void Drive::


void Drive::drive_PID(double setpoint_left_pos, double setpoint_right_pos, double setpoint_left_speed, double setpoint_right_speed, double heading, int count) {
  
  if(count ==0){
    //Gyro->Reset();
    Left_encoder->Reset();
    Right_encoder->Reset();
  }
  double encoder_val_left = Left_encoder->Get();
  double encoder_val_right = Right_encoder->Get();
  //double encoder_speed_left = Left_encoder->GetRate();
  //double encoder_speed_right = Right_encoder->GetRate();
  double gyro_val = Gyro->GetAngle();

  double error_left_pos = setpoint_left_pos - encoder_val_left;
  double error_right_pos = setpoint_right_pos - encoder_val_right;
  //double error_left_speed = setpoint_left_speed - encoder_speed_left;
  //double error_right_speed = setpoint_right_speed - encoder_speed_right;
  double error_heading = heading - gyro_val;

  double max_speed = 13; //ft/s
  double kp_speed = 0/max_speed;
  double kp_pos = 0;
  double kph = -0.025;

  double output_left = (error_left_pos * kp_pos) + kp_speed*setpoint_left_speed;
  double output_right = (error_right_pos * kp_pos) + kp_speed*setpoint_right_speed;

  double turn_val = kph * error_heading;
  //double output_left = (error_left_pos * kp_pos) + (error_left_speed * kp_speed) * .05;
  //double output_right = (error_right_pos * kp_pos) + (error_right_speed * kp_speed) * .05;

  Leftdrive->Set(output_left + turn_val);
  Rightdrive->Set(output_right - turn_val);

  auto Left_encoderstr = std::to_string(encoder_val_left);
  frc::SmartDashboard::PutString("DB/String 6",Left_encoderstr);
  auto Right_encoderstr = std::to_string(error_left_pos);
  frc::SmartDashboard::PutString("DB/String 7",Right_encoderstr);
  Right_encoderstr = std::to_string(setpoint_left_pos);
  frc::SmartDashboard::PutString("DB/String 8",Right_encoderstr);
  Right_encoderstr = std::to_string(setpoint_left_speed);
  frc::SmartDashboard::PutString("DB/String 9",Right_encoderstr);
}


bool Drive::platform_adjust(){
  double AnalogIn = FrontDistance->GetVoltage();
  double error = 1.26 - AnalogIn;
  double kp_c = .7;
  double output = kp_c * error;
  
  output = Threshold(output,0.95);

  Leftdrive->Set(output);
  Rightdrive->Set(output);
  bool distance_tf = false;
  if (AnalogIn < 1.35 and AnalogIn > 1.15){
    distance_tf = true;
  }
  else {
    distance_tf = false;
  }
  return distance_tf;
}

void Drive::OrangeLeds() {

  Leds->SetVoltage(0.55);
}
void Drive::PartyLeds() {

  Leds->SetVoltage(1.1);
}
void Drive::WhiteLeds() {

  Leds->SetVoltage(1.65);
}
void Drive::BlueLeds() {

  Leds->SetVoltage(2.2);
}
void Drive::YellowLeds() {

  Leds->SetVoltage(2.75);
}
void Drive::OffLeds() {

  Leds->SetVoltage(3.3);
}

void Drive::Dashboard(){

  double encoder_val_left = Left_encoder->Get();
  double encoder_val_right = Right_encoder->Get();

  auto Left_encoderstr = std::to_string(encoder_val_left);
  frc::SmartDashboard::PutString("Left Encoder",Left_encoderstr);

  auto Right_encoderstr = std::to_string(encoder_val_right);
  frc::SmartDashboard::PutString("Right Encoder",Right_encoderstr);


  double gyro_val = Gyro->GetAngle();
  auto gyro_valstr = std::to_string(gyro_val);
  frc::SmartDashboard::PutString("Gyro",gyro_valstr);

  double AnalogIn = FrontDistance->GetVoltage();
  if (AnalogIn < 0.7){
    frc::SmartDashboard::PutBoolean("In Range", true);
  }
  else {
    frc::SmartDashboard::PutBoolean("In Range", false);
  }
  auto FrontDistancestr = std::to_string(AnalogIn);
  frc::SmartDashboard::PutString("Laser Range Finder",FrontDistancestr);

  

}

