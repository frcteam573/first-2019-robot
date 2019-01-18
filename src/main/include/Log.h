/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// It's desirable that everything possible under private except
// for methods that implement subsystem capabilities

#ifndef Log_H
#define Log_H

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc\WPILib.h"
#include "frc\VictorSP.h"
using namespace std;

class Log : public frc::Subsystem {

 private:
//frc::VictorSP * Leftdrive;

 public:
  Log();
  void InitDefaultCommand() override;
// User Wrtitten Functions Definitions here.

};
#endif 