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
  bool button_lb = controller1.GetRawButton(5);
  bool button_rb = controller1.GetRawButton(6);
  bool button_start = controller1.GetRawButton(8);
  bool button_back = controller1.GetRawButton(7);
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
  frc::SmartDashboard::PutString("DB/String 2",leftinstr);
  //frc::SmartDashboard::PutString("DB/String 1",rightinstr);
  // Drive Code Section
  if (button_b){
    MyDrive.Camera_Centering(leftin, camera_x);
  }
  else if (button_a and count < 120){
    
    //Get setpoint values from tables
    
    double left_pos = MyAuto.ReturnTableVal(count,0);
    double left_speed = MyAuto.ReturnTableVal(count,1);

    left_pos = left_pos*120;
    left_speed = left_speed*120;
    //Call PID Loop to follow path
    MyDrive.drive_PID(left_pos, 0, left_speed, 0) ;
    count ++;
  }
  else {
    MyDrive.Joystick_drive(leftin,rightin);
  }
  
   
}

void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic() {

  // Read in controller input values
  double leftin = controller1.GetRawAxis(1); //Get Drive Left Joystick Y Axis Value
  double rightin = controller1.GetRawAxis(5); //Get Drive right Joystick Y Axis Value
  bool button_b = controller1.GetRawButton(2);
  bool button_a = controller1.GetRawButton(1); 
  bool button_lb = controller1.GetRawButton(5);
  bool button_rb = controller1.GetRawButton(6);
  bool button_start = controller1.GetRawButton(8);
  bool button_back = controller1.GetRawButton(7);
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
if (camera_exist==1){
  MyDrive.WhiteLeds();
}
else {
  MyDrive.OffLeds();
}
// Push string values to Dashboard
  frc::SmartDashboard::PutString("DB/String 2",leftinstr);
  //frc::SmartDashboard::PutString("DB/String 1",rightinstr);
  // Drive Code Section
  if (button_b){
    MyDrive.Camera_Centering(leftin, camera_x);
  }
  else if (button_a){
    MyDrive.Camera_Centering_Distance(camera_x, image_size);
  }
  else {
    MyDrive.Joystick_drive(leftin,rightin);
  }

  //Climber code section
  MyDrive.Climb_Extend(button_lb, button_rb, button_start, button_back);

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
  
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif