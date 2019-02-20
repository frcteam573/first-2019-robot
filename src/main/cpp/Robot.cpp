/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "frc\Joystick.h"
#include "frc\WPILib.h"
#include "frc\Talon.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "NetworkTables/NetworkTable.h"
#include "NetworkTables/NetworkTableInstance.h"
#include "Drive.h"
#include "Appendage.h"
#include "Log.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include "Auto.h"

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  MyLog.Create();
  MyAppendage.spatuclawRetract();
  MyAppendage.spatuclawClose();
  MyAppendage.punchyIn();
  MyDrive.OrangeLeds();
  

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  int count = 0;
  int count_2 = -40;
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
 // Read in controller input values
  double leftin = controller1.GetRawAxis(1); //Get Drive Left Joystick Y Axis Value
  double rightin = controller1.GetRawAxis(5); //Get Drive right Joystick Y Axis Value
  bool button_b = controller1.GetRawButton(2);
  bool button_a = controller1.GetRawButton(1); 
  bool button_y = controller1.GetRawButton(4);
  bool button_lb = controller1.GetRawButton(5);
  bool button_rb = controller1.GetRawButton(6);
  bool button_start = controller1.GetRawButton(8);
  bool button_back = controller1.GetRawButton(7);
  bool right_trigger = controller1.GetRawAxis(3);
  bool button_lb2 = controller2.GetRawButton(5);
  bool button_rb2 = controller2.GetRawButton(6);
  bool button_start2 = controller2.GetRawButton(8);
  bool button_back2 = controller2.GetRawButton(7);
  double right_trigger2 = controller2.GetRawAxis(3);
  double d_pad2 = controller2.GetPOV(0);
  double leftin2 = controller2.GetRawAxis(1);
  bool button_a2 = controller2.GetRawButton(1);
  bool button_b2 = controller2.GetRawButton(2);
  bool button_y2 = controller2.GetRawButton(4);
  double left_trigger2 = controller2.GetRawAxis(2);
  // Read in camera Stuff
  
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  
  table->PutNumber("ledMode", 0);
  table->PutNumber("camMode", 0);
  table->PutNumber("pipeline", 1); //Vision Target pipeline

  float camera_x = table->GetNumber("tx", 0);
  float camera_exist = table->GetNumber("tv", 0);
  float image_size = table->GetNumber("ta", 0);
  auto leftinstr = std::to_string(camera_x);
 // auto rightinstr = std::to_string(RightStick);

// Push string values to Dashboard
  //frc::SmartDashboard::PutString("DB/String 2",leftinstr);
  //frc::SmartDashboard::PutString("DB/String 1",rightinstr);

  // Drive Code Section
  bool distance_tf_b = false;
  double count_max = MyAuto.ReturnTableVal(0,5);
  int count_max_int = (int)count_max;
  double count_max_2 = MyAuto.ReturnTableVal_2(0,5);
  int count_max_int_2 = (int)count_max_2;
  
  if (button_b){
   distance_tf_b = MyDrive.Camera_Centering(leftin, camera_x);
  }
  else if (button_a and count < count_max_int){
    
    //Get setpoint values from tables
    
    double left_pos = MyAuto.ReturnTableVal(count,0);
    double left_speed = MyAuto.ReturnTableVal(count,1);
    double right_pos = MyAuto.ReturnTableVal(count,2);
    double right_speed = MyAuto.ReturnTableVal(count,3);
    double heading = MyAuto.ReturnTableVal(count,4);
    

    //Call PID Loop to follow path
    MyDrive.drive_PID(left_pos, right_pos, left_speed, right_speed,heading,count) ;
    count ++;
    count_2 = -40;
  }

else if (button_y and count_2 < count_max_int_2){
  
  if (count_2 < -20){
    MyDrive.encoder_drive(-150, count_2);
    
  }
  else if (count_2 < 0){
    MyDrive.gyro_drive(180); 
  }
  else if (count_2 < count_max_int_2){
    double left_pos = MyAuto.ReturnTableVal_2(count_2,0);
    double left_speed = MyAuto.ReturnTableVal_2(count_2,1);
    double right_pos = MyAuto.ReturnTableVal_2(count_2,2);
    double right_speed = MyAuto.ReturnTableVal_2(count_2,3);
    double heading = MyAuto.ReturnTableVal_2(count_2,4);
    

    //Call PID Loop to follow path
    MyDrive.drive_PID(left_pos, right_pos, left_speed, right_speed,heading,count_2) ;
  }
  count_2++;
}

  else {
    MyDrive.Joystick_drive(leftin,rightin);
  }

frc::SmartDashboard::PutString("DB/String 4", to_string(count_2));
    //LED section
  if (distance_tf_b){
    MyDrive.BlueLeds();
    
  }
  else if (camera_exist==1) {
    MyDrive.WhiteLeds();
    
  }
  else {
    MyDrive.OffLeds();
    
  }
  //DashBoard Code
  MyDrive.Dashboard();
  MyAppendage.Dashboard();
  MyLog.Dashboard();
  frc::SmartDashboard::PutString("Camera TV", to_string(camera_exist));
  if (camera_exist == 1){
    frc::SmartDashboard::PutBoolean("Target Secured", true);
  }
  else {
    frc::SmartDashboard::PutBoolean("Target Secured", false);
  }
  frc::SmartDashboard::PutString("Camera TX", to_string(camera_x));
  frc::SmartDashboard::PutString("Camera TA", to_string(image_size));
}


void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic() {

  // Read in controller input values
  double leftin = controller1.GetRawAxis(1); //Get Drive Left Joystick Y Axis Value
  double rightin = controller1.GetRawAxis(5); //Get Drive right Joystick Y Axis Value
  bool button_b = controller1.GetRawButton(2);
  bool button_a = controller1.GetRawButton(1); 
  bool button_y = controller1.GetRawButton(4);
  bool button_lb = controller1.GetRawButton(5);
  bool button_rb = controller1.GetRawButton(6);
  bool button_start = controller1.GetRawButton(8);
  bool button_back = controller1.GetRawButton(7);
  bool right_trigger = controller1.GetRawAxis(3);
  bool button_lb2 = controller2.GetRawButton(5);
  bool button_rb2 = controller2.GetRawButton(6);
  bool button_start2 = controller2.GetRawButton(8);
  bool button_back2 = controller2.GetRawButton(7);
  double right_trigger2 = controller2.GetRawAxis(3);
  double d_pad2 = controller2.GetPOV(0);
  double leftin2 = controller2.GetRawAxis(1);
  bool button_a2 = controller2.GetRawButton(1);
  bool button_b2 = controller2.GetRawButton(2);
  bool button_y2 = controller2.GetRawButton(4);
  double left_trigger2 = controller2.GetRawAxis(2);
  // Read in camera Stuff
  
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  
  table->PutNumber("ledMode", 0);
  table->PutNumber("camMode", 0);
  table->PutNumber("pipeline", 1); //Vision Target pipeline

  float camera_x = table->GetNumber("tx", 0);
  float camera_exist = table->GetNumber("tv", 0);
  float image_size = table->GetNumber("ta", 0);
  auto leftinstr = std::to_string(image_size);
  //auto rightinstr = std::to_string(RightStick);
  bool distance_tf = false;
  bool distance_platform = false;
  bool distance_tf_b = false;
// Push string values to Dashboard
  frc::SmartDashboard::PutString("DB/String 2",leftinstr);
  //frc::SmartDashboard::PutString("DB/String 1",rightinstr);
  // Drive Code Section
  if (button_b){
    distance_tf_b = MyDrive.Camera_Centering(leftin, camera_x);
  }
  else if (button_a){
    distance_tf = MyDrive.Camera_Centering_Distance(camera_x, image_size);
  }
  //else if (button_y){
  //  distance_platform = MyDrive.platform_adjust();
  //}
  else {
    MyDrive.Joystick_drive(leftin,rightin);

  MyLog.DrivetrainCurrentCompare(0, rightin);
	MyLog.DrivetrainCurrentCompare(1, rightin);
	MyLog.DrivetrainCurrentCompare(14, leftin);
	MyLog.DrivetrainCurrentCompare(15, leftin);
  }

  //Climber code section
  MyDrive.Climb_Extend(button_lb, button_rb, leftin);

  //Logging section
  MyLog.PDP(15, 0, true);

  // Appendage code
  if (button_lb2){
    MyAppendage.spatuclawExtend();
  }
  else if (button_rb2){
    MyAppendage.spatuclawRetract();
  }
  
  if (button_start2){
    MyAppendage.spatuclawOpen();
  }
  else if (button_back2){
    MyAppendage.spatuclawClose();
  }
  
  if (right_trigger2 > 0.5){
    MyAppendage.punchyOut();
  }
  else {
    MyAppendage.punchyIn();
  }

  if (d_pad2 > 45 and d_pad2 < 135){
    MyAppendage.spatuclawIn();
  }
  else if (d_pad2 > 225 and d_pad2 < 315){
    MyAppendage.spatuclawOut();
  }
  else {
    MyAppendage.spatuclawStop();
  }

    
  if (button_a2) {
    if (left_trigger2 > 0.5){
      MyAppendage.elevator_PID(250);
    }
    else {
      MyAppendage.elevator_PID(1);
    }
  }
  else if (button_b2) {
    if (left_trigger2 > 0.5){
      MyAppendage.elevator_PID(750);
    }
    else {
      MyAppendage.elevator_PID(500);
    }
  }
  else if (button_y2) {
    if (left_trigger2 > 0.5){
      MyAppendage.elevator_PID(1250);
    }
    else {
      MyAppendage.elevator_PID(1000);
    }
  }
  else {
    MyAppendage.elevator_joystick(leftin2);
  }
//LED section
if (distance_tf or distance_tf_b){
  MyDrive.BlueLeds();
  
}
else if (camera_exist==1) {
  MyDrive.WhiteLeds();
  
}
else if (distance_platform){
  MyDrive.YellowLeds();
}
else if (button_lb and button_rb){
  MyDrive.PartyLeds();
}

else {
  MyDrive.OffLeds();
  
}

//Dashboard code
  MyDrive.Dashboard();
  MyAppendage.Dashboard();
  MyLog.Dashboard();
  frc::SmartDashboard::PutString("Camera TV", to_string(camera_exist));
  if (camera_exist == 1){
    frc::SmartDashboard::PutBoolean("Target Secured", true);
  }
  else {
    frc::SmartDashboard::PutBoolean("Target Secured", false);
  }
  frc::SmartDashboard::PutString("Camera TX", to_string(camera_x));
  frc::SmartDashboard::PutString("Camera TA", to_string(image_size));



}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif