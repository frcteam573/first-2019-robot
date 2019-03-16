/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc\WPILib.h"
#include "frc\VictorSP.h"
#include "Log.h"
#include <fstream>
#include <ctime>
#include <time.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <frc/PowerDistributionPanel.h>

using namespace std;

ofstream outText;
frc::PowerDistributionPanel board;
int counter;

Log::Log() : Subsystem("Log") {
//Leftdrive = new frc::VictorSP(0);

}

string Log::dateAndTime() {

	/*time_t curTime = time(nullptr);
	string timeOut = asctime(localtime(&curTime));
	//return timeOut;
	return "test value";*/

	time_t t = time(0);
	struct tm * now = localtime(&t);

	int year = now->tm_year + 1900;
	string yearS = to_string(year);
	int month = now->tm_mon + 1;
	string monthS = to_string(month);
	int day = now->tm_mday;
	string dayS = to_string(day);
	int hour = now->tm_hour;
	string hourS = to_string(hour);
	int minute = now->tm_min;
	string minuteS = to_string(minute);
	int second = now->tm_sec;
	string secondS = to_string(second);
  
	string currentTimeDate = monthS + "." + dayS + "." + yearS + "___" + hourS + "." + minuteS + "." + secondS;

	return currentTimeDate;

}

void Log::Create() {

	//MUST PUT USB IN PORT ON INSIDE OF ROBORIO

	//late incorporate time and date and USB file path
	string filepath = "/u/" + dateAndTime() + ".csv";
	outText.open(filepath);
	//outText.open("/u/test.txt");

}

void Log::Write(string text) {

	outText << text;
	outText << endl;

}

void Log::PDP(int slot, double limit, bool override) {

	double val = board.GetCurrent(slot);

	if(val > limit || override){
	string output = dateAndTime() + "," + to_string(slot) + "," + to_string(limit) + "," + to_string(val) + "," + to_string(override);

	Write(output);
}


}

void Log::PDPTotal(){
	double val = board.GetTotalCurrent();
	bool light;


	if (val > 400){

		counter = counter + 1;
		if (counter > 50){
			light = true;
		}
		else{
			light = false;
		}
	}
	else {
		counter = 0;
		light = false;
	}

	frc::SmartDashboard::PutBoolean("Over 400 amps", light);
	//auto Gyrooutstr = std::to_string(counter);
	//frc::SmartDashboard::PutString("DB/String 5",Gyrooutstr);
}

void Log::DrivetrainCurrentCompare(int slot,double PWMin){

	double current = board.GetCurrent(slot);

	if (abs(PWMin) > .2){
		if (abs(current) < 1){
			string output = "!----PDP Slot" + to_string(slot) + "is not getting enough current when driven";
			Write(output);
			frc::SmartDashboard::PutString("PDP not driven:",to_string(slot));
		}
	}
}

void Log::Dashboard(){

	double val;

	val = board.GetCurrent(0);
	frc::SmartDashboard::PutString("Right Drive Front", to_string(val));

	val = board.GetCurrent(1);
	frc::SmartDashboard::PutString("Right Drive Back", to_string(val));

	// val = board.GetCurrent(2);
	//frc::SmartDashboard::PutString("Left Cim 3", to_string(val));

	 val = board.GetCurrent(13);
	frc::SmartDashboard::PutString("Elevator 1", to_string(val));

	 val = board.GetCurrent(14);
	frc::SmartDashboard::PutString("Left Drive Front", to_string(val));

	val = board.GetCurrent(15);
	frc::SmartDashboard::PutString("Left Drive Back", to_string(val));

	//val = board.GetCurrent(15);
	//frc::SmartDashboard::PutString("Right Cim 3", to_string(val));
	//frc::SmartDashboard::PutString("DB/String 4",to_string(board.GetCurrent(15)));

	val = board.GetCurrent(10);
	frc::SmartDashboard::PutString("Left Arm", to_string(val));

	val = board.GetCurrent(2);
	frc::SmartDashboard::PutString("Right Arm", to_string(val));

	val = board.GetCurrent(3);
	frc::SmartDashboard::PutString("Right Arm Drive", to_string(val));


	 val = board.GetCurrent(12);
	frc::SmartDashboard::PutString("Elevator 2", to_string(val));

	 val = board.GetCurrent(13);
	frc::SmartDashboard::PutString("Elevator 1", to_string(val));

	 val = board.GetCurrent(11);
	frc::SmartDashboard::PutString("Left Arm Drive", to_string(val));

	val = board.GetCurrent(4);
	frc::SmartDashboard::PutString("Intake Roller", to_string(val));

	PDPTotal();

	

}
void Log::Close() {

	outText.close();

}

