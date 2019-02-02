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
#include <fstream>
#include <ctime>
#include <time.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <frc/PowerDistributionPanel.h>
using namespace std;

class Log : public frc::Subsystem {

 private:
//frc::VictorSP * Leftdrive;

 public:
  Log();
	string dateAndTime();
	void Create();
	void PDP(int slot, double limit, bool override);
	void Write(string text);
	void PDPTotal();
	void Close();
	void DrivetrainCurrentCompare(int slot,double PWMin);
	void Dashboard();
// User Wrtitten Functions Definitions here.

};
#endif 