/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// It's desirable that everything possible under private except
// for methods that implement subsystem capabilities

#ifndef Auto_H
#define Auto_H

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc\WPILib.h"

using namespace std;

class Auto : public frc::Subsystem {

 private:
//frc::VictorSP * Leftdrive;

 public:
  Auto();
  double ReturnTableVal(int count, int select);
  double ReturnTableVal_2(int count, int select);
// User Wrtitten Functions Definitions here.

};
#endif 