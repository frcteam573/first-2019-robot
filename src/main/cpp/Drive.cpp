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
#include "frc\AnalogGyro.h"
#include "frc\SPI.h"
#include "frc\Compressor.h"
#include "frc\DoubleSolenoid.h"

using namespace std;

double leftdriveold;
double rightdriveold;
Drive::Drive() : Subsystem("Drive") {
Leftdrive = new frc::VictorSP(1);
Leftdrive->SetInverted(true);
Rightdrive = new frc::VictorSP(0);
//Leftclimb = new frc::VictorSP(4);
//Trollyclimb = new frc::VictorSP(4);
//Trollyclimb->SetInverted(true);
Left_encoder = new frc::Encoder( 11, 12, false, frc::Encoder::k4X);//ACTUALLY 2 AND 3 THIS IS FOR TESTING
Right_encoder = new frc::Encoder( 9, 10, false, frc::Encoder::k4X);//ACTUALLY 0 AND 1 THIS IS FOR TESTING
FrontDistance = new frc::AnalogInput(2);
Leds = new frc::AnalogOutput(0);
Gyro = new frc::AnalogGyro(1);
Compressor = new frc::Compressor(1);
frontclimbSolenoid = new frc::DoubleSolenoid(2, 0, 1);
backclimbSolenoid = new frc::DoubleSolenoid(2, 2, 3);
right_arm_encoder = new frc::Encoder(1, 0, false, frc::Encoder::k4X);//ACTUALLY 10 AND 9 THIS IS FOR TESTING
left_arm_encoder = new frc::Encoder(4, 5, false, frc::Encoder::k4X);
back_encoder = new frc::Encoder(3, 2, false, frc::Encoder::k4X);////ACTUALLY 11 AND 12 THIS IS FOR TESTING
right_arm = new frc::VictorSP(2);
left_arm = new frc::VictorSP(3);
left_arm->SetInverted(true);
back = new frc::VictorSP(4);
back->SetInverted(true);
right_arm_drive = new frc::VictorSP(5);
left_arm_drive = new frc::VictorSP(8);
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

/*if (abs(LeftStick) > 0.8 and abs(RightStick) > 0.8){
  Compressor->Stop();
}
else {
  Compressor->Start();
}*/
// Convert double to strings
  auto leftinstr = std::to_string(LeftStick);
  auto rightinstr = std::to_string(RightStick);

// Push string values to Dashboard
 // frc::SmartDashboard::PutString("DB/String 0",leftinstr);
  //frc::SmartDashboard::PutString("DB/String 1",rightinstr);

double AnalogIn = FrontDistance->GetVoltage();
auto Analoginstr = std::to_string(AnalogIn);
//frc::SmartDashboard::PutString("DB/String 9",Analoginstr);

}


void Drive::Joystick_drive_slow(double LeftStick,double RightStick) {

// Set motor values
LeftStick = LeftStick * 0.8;
RightStick = RightStick * 0.8;
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

/*if (abs(LeftStick) > 0.8 and abs(RightStick) > 0.8){
  Compressor->Stop();
}
else {
  Compressor->Start();
}*/
// Convert double to strings
  auto leftinstr = std::to_string(LeftStick);
  auto rightinstr = std::to_string(RightStick);

// Push string values to Dashboard
 // frc::SmartDashboard::PutString("DB/String 0",leftinstr);
  //frc::SmartDashboard::PutString("DB/String 1",rightinstr);

double AnalogIn = FrontDistance->GetVoltage();
auto Analoginstr = std::to_string(AnalogIn);
//frc::SmartDashboard::PutString("DB/String 9",Analoginstr);

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
  //frc::SmartDashboard::PutString("DB/String 9",Analoginstr);
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
/*void Drive::Climb_Extend(bool button_lb, bool button_rb, double leftjoystick){

  if (button_lb){
   frontclimbSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
  }
  else {
    frontclimbSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
  }

  if (button_rb){
    backclimbSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    leftjoystick = leftjoystick*leftjoystick*leftjoystick; 
    Trollyclimb->Set(leftjoystick);
  }
  else{
    backclimbSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    Trollyclimb->Set(0);
  }
  }*/


bool Drive::climb_setpoint_PID(double left_set, double right_set, double back_set){
  
  double enc_left_arm = left_arm_encoder->Get();
  double enc_right_arm = right_arm_encoder->Get();
  double enc_back = back_encoder->Get();

  double error_left_arm = left_set - enc_left_arm;
  double error_right_arm = right_set - enc_right_arm;
  double error_back = back_set - enc_back;

  bool near_set = false;

  if (abs(error_left_arm) < 1000 and abs(error_right_arm) < 1000 and abs(error_back) < 1000){
    near_set = true;
  }

  double kpa = -0.00001;
  double kpb = -0.00001;

  double output_left_arm = (error_left_arm * kpa);
  double output_right_arm = (error_right_arm * kpa);
  double output_back = (error_back * kpb);

  output_left_arm = Threshold(output_left_arm, 0.99);
  output_right_arm = Threshold(output_right_arm, 0.99);
  output_back = Threshold(output_back, 0.99);

  right_arm->Set(output_right_arm);
  left_arm->Set(output_left_arm);
  back->Set(output_back);

  return near_set;

}

bool Drive::climb_setpoint_PID_retract_arms(double left_set, double right_set){
  
  double enc_left_arm = left_arm_encoder->Get();
  double enc_right_arm = right_arm_encoder->Get();
  
  double error_left_arm = left_set - enc_left_arm;
  double error_right_arm = right_set - enc_right_arm;
  
  bool near_set = false;

  double kpa = -0.00001;
  

  double output_left_arm = (error_left_arm * kpa);
  double output_right_arm = (error_right_arm * kpa);
  

  output_left_arm = Threshold(output_left_arm, 0.8);
  output_right_arm = Threshold(output_right_arm, 0.8);
 

  right_arm->Set(output_right_arm);
  left_arm->Set(output_left_arm);
  

  return near_set;

}


bool Drive::climb_setpoint_PID_retract_back(double back_set){
  
  double enc_back = back_encoder->Get();

  double error_back = back_set - enc_back;

  bool near_set = false;

  
  double kpb = -0.00001;

  double output_back = (error_back * kpb);

  output_back = Threshold(output_back, 0.8);

  back->Set(output_back);

  return near_set;

}


void Drive::climb_PID(double left_set, double right_set, double back_set){
  double enc_left_arm = left_arm_encoder->Get();
  double enc_right_arm = right_arm_encoder->Get();
  double enc_back = back_encoder->Get();

  double error_left_arm = left_set - enc_left_arm;
  double error_right_arm = right_set - enc_right_arm;
  double error_back = back_set - enc_back;

  bool near_set = false;

  if (abs(error_left_arm) < 1000 and abs(error_right_arm) < 1000 and abs(error_back) < 1000){
    near_set = true;
  }

  double kpa = -0.0001;
  double kpb = -0.0001;

  double output_left_arm = (error_left_arm * kpa);
  double output_right_arm = (error_right_arm * kpa);
  double output_back = (error_back * kpb);

  output_left_arm = Threshold(output_left_arm, 0.99);
  output_right_arm = Threshold(output_right_arm, 0.99);
  output_back = Threshold(output_back, 0.99);

  right_arm->Set(output_right_arm);
  left_arm->Set(output_left_arm);
  back->Set(output_back);

  /*double left_set_rate = 0.785398163397448/2;//radians/sec pi/8
  double right_set_rate = 0.785398163397448/2; //radians/sec pi/8
  double back_set_rate = 6; // inches/sec

  double left_set = 1.5707963267948966; //radians pi/2
  double right_set = 1.5707963267948966; //radians pi/2
  double back_set = 20; //inches
  if (level_2){
    left_set = 205000;//0.3490658503988659;
    right_set = 205000;//0.3490658503988659;
    back_set = 110000;
  }

  double enc_left = left_arm_encoder->Get();
  double enc_right = right_arm_encoder->Get();
  double enc_back = back_encoder->Get();

  double enc_left_rate = left_arm_encoder->GetRate();
  double enc_right_rate = right_arm_encoder->GetRate();
  double enc_back_rate = back_encoder->GetRate();

  //enc_left = enc_left * 1024 / 866.25; // radians/sec
  //enc_right = enc_right * 1024 / 866.25; // radians/sec
  //enc_back = enc_back / 2.8274333882308 /35  ; //inches/sec

  enc_left_rate = enc_left_rate  / 866.25; // radians/sec
  enc_right_rate = enc_right_rate / 866.25; // radians/sec
  enc_back_rate = enc_back_rate / 2.8274333882308 /35  ; //inches/sec
  
  double error_left_rate = left_set - enc_left;
  double error_right_rate = right_set - enc_right;
  double error_back_rate = back_set_rate - enc_back_rate;
  
  double kpa = -0.001;
  double kpb = -0.0001;


  double error_back = back_set - enc_back;
  double output_back = (error_back * kpb);
  

  double output_left_arm = (error_left_rate * kpa);
  double output_right_arm = (error_right_rate * kpa);
  //double output_back = (error_back_rate * kpb);

  output_left_arm = Threshold(output_left_arm, 0.8);
  output_right_arm = Threshold(output_right_arm, 0.8);
  output_back = Threshold(output_back, 0.8);

  /*right_arm->Set(output_right_arm);
  left_arm->Set(output_left_arm);
  back->Set(output_back);*/

/*
  if (enc_back >= back_set-10000){
    

    double error_back = back_set - enc_back;
    double kpb = 0.001;
    double output_back = (error_back * kpb);
    output_back = Threshold(output_back, 0.8);

    back->Set(output_back);
  }
  else{
    back->Set(output_back);
  }*/

  /*if (enc_left >= left_set){
    left_arm->Set(0);
  }
  else{
    left_arm->Set(output_left_arm);
  }

  if (enc_right >= right_set){
    right_arm->Set(0);
  }
  else{
    right_arm->Set(output_right_arm);
  }*/

}

void Drive::climb_drive(double LeftStick, double RightStick){
  LeftStick = LeftStick * LeftStick * LeftStick;
  RightStick = RightStick * RightStick * RightStick;
  right_arm_drive->Set(RightStick);
  left_arm_drive->Set(LeftStick);
}


void Drive::climb_stop(){
  left_arm->Set(0);
  right_arm->Set(0);
  back->Set(0);
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
  double kp_speed = -1/max_speed;
  double kp_pos = -0.025;
  double kph = -0.01;  //0.01;

  double output_left = (error_left_pos * kp_pos) + kp_speed*setpoint_left_speed;
  double output_right = (error_right_pos * kp_pos) + kp_speed*setpoint_right_speed;

  double turn_val = kph * error_heading;
  //double output_left = (error_left_pos * kp_pos) + (error_left_speed * kp_speed) * .05;
  //double output_right = (error_right_pos * kp_pos) + (error_right_speed * kp_speed) * .05;

  Leftdrive->Set(output_left + turn_val);
  Rightdrive->Set(output_right - turn_val);

  /*auto Left_encoderstr = std::to_string(output_left);
  frc::SmartDashboard::PutString("DB/String 6",Left_encoderstr);
  auto Right_encoderstr = std::to_string(error_left_pos);
  frc::SmartDashboard::PutString("DB/String 7",Right_encoderstr);
  Right_encoderstr = std::to_string(setpoint_left_pos);
  frc::SmartDashboard::PutString("DB/String 8",Right_encoderstr);
  Right_encoderstr = std::to_string(setpoint_left_speed);
  frc::SmartDashboard::PutString("DB/String 9",Right_encoderstr);*/
}

void Drive::encoder_drive(double setpoint, int count, double thresh_speed){
  double gyro_val_constant;
  if(count == -40){
    //Gyro->Reset();
    Left_encoder->Reset();
    Right_encoder->Reset();
    gyro_val_constant = Gyro->GetAngle();
  }
  double encoder_val_left = Left_encoder->Get();
  double encoder_val_right = Right_encoder->Get();
  //double encoder_speed_left = Left_encoder->GetRate();
  //double encoder_speed_right = Right_encoder->GetRate();
  double gyro_val = Gyro->GetAngle();

  double error_left_pos = setpoint - encoder_val_left;
  double error_right_pos = setpoint - encoder_val_right;
  //double error_left_speed = setpoint_left_speed - encoder_speed_left;
  //double error_right_speed = setpoint_right_speed - encoder_speed_right;
  double error_heading = gyro_val_constant - gyro_val;

  double kp_pos = -0.025;
  double kph = -0.01;

  double output_left = Threshold((error_left_pos * kp_pos), thresh_speed) ;
  double output_right = Threshold((error_right_pos * kp_pos), thresh_speed) ;

  double turn_val = kph * error_heading;
  //double output_left = (error_left_pos * kp_pos) + (error_left_speed * kp_speed) * .05;
  //double output_right = (error_right_pos * kp_pos) + (error_right_speed * kp_speed) * .05;
  
  Leftdrive->Set(output_left + turn_val);
  Rightdrive->Set(output_right - turn_val);
}

void Drive::gyro_drive(double setpoint){
  
  
  double gyro_val = Gyro->GetAngle();
  /*if (gyro_val < 0){
    setpoint = -1 * setpoint;
  }*/

  
  double error_heading = setpoint - gyro_val;

  
  double kph = -0.01;

  
  double turn_val = kph * error_heading;
 
  Leftdrive->Set(0 + turn_val);
  Rightdrive->Set(0 - turn_val);
}



void Drive::arm_joystick(double LeftStick, double RightStick){
  left_arm->Set(LeftStick);
  right_arm->Set(LeftStick);
  back->Set(RightStick);
}

/*bool Drive::platform_adjust(){
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
}*/

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
  double right_arm_enc = right_arm_encoder->Get();
  double left_arm_enc = left_arm_encoder->Get();
  double back_enc = back_encoder->Get();


  auto Left_encoderstr = std::to_string(encoder_val_left);
  frc::SmartDashboard::PutString("Left Encoder",Left_encoderstr);

  auto Right_encoderstr = std::to_string(encoder_val_right);
  frc::SmartDashboard::PutString("Right Encoder",Right_encoderstr);

  auto Left_arm_encoderstr = std::to_string(left_arm_enc);
  frc::SmartDashboard::PutString("Left Arm Encoder",Left_arm_encoderstr);

  auto Right_arm_encoderstr = std::to_string(right_arm_enc);
  frc::SmartDashboard::PutString("Right Arm Encoder",Right_arm_encoderstr);

  auto Back_encoderstr = std::to_string(back_enc);
  frc::SmartDashboard::PutString("Back Encoder",Back_encoderstr);


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

void Drive::GyroReset(){
  Gyro->Reset();
}