/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "frc\Joystick.h"
#include <frc\WPILib.h>
#include "networktables/NetworkTable.h"
#include "NetworkTables/NetworkTableInstance.h"
#include "Drive.h"
#include "Appendage.h"
#include "Log.h"
#include "Auto.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  
  // Subsystem Definition
  Drive MyDrive;
  Appendage MyAppendage;
  Log MyLog;
  Auto MyAuto;

 private:
  frc::Joystick controller1{ 0 };  // Xbox controller 1
  frc::Joystick controller2{ 1 }; // Xbox controller 2
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  int count;
  int count_2;
  int count_3;
  int count_4;
  bool cam_state;
  bool spatuclawState;
  
};
