#include "main.h"
#include "drive.hpp"
#include "rotate.hpp"
#include "subsystems.hpp"
using namespace okapi;

MotorGroup chassis({-1, -2, 4, 5});

double headg;
int multiplier = 0;

// function to drive straight while holding a target heading
void driveIt(double targHeading, double rpm) {
	headg = gyroRotate.getHeading();
    pros::delay(5);
	//pros::lcd::set_text(1,std::to_string(headg));			//displays actual heading to lcd
	gyroRotate.rotateController.setTarget(targHeading);		//sets the heading of the gyro to the desired path angle

	//output to drivetrain, modeled as arcade to account for gyro correction
	//drive->getModel()->driveVectorVoltage(rpm, gyroRotate.rotateController.step(gyroRotate.getHeading()));
	drive->getModel()->driveVectorVoltage(rpm, gyroRotate.rotateController.step(gyroRotate.getHeading()));
}

// more advanced driveIt that slows down as it approaches the target
void chassisProfiling(double target, double error, long double aggr, double targHeading, int timeOut){
  double rpm = 0;	//output speed in millivolts
	long startTime = pros::millis();

	if (target>chassis.getPosition()) {
  while (chassis.getPosition()<(target-error) && (pros::millis()<=startTime+timeOut)) {						//while the current position is less than the target, drive
	//	std::cout << std::to_string(rpm) << ", " << chassis.getPosition() << std::endl;				//output rpm to console, used for troubleshooteing
    rpm = sqrtl(aggr*(-1*chassis.getPosition()+target));	//sqrt function to control velocity as it approaches the target position, aggr is how fast it slows down
    if (rpm>1) {
      rpm = 1;
    }
    driveIt(targHeading, rpm);
  }
} else {
	while(chassis.getPosition()>(target+error) && (pros::millis()<=startTime+timeOut)){						//while the current position is less than the target, drive
	//	std::cout << std::to_string(rpm) << ", " << chassis.getPosition() << std::endl;				//output rpm to console, used for troubleshooteing
    rpm = -sqrtl(-aggr*(-1*chassis.getPosition()+target));	//sqrt function to control velocity as it approaches the target position, aggr is how fast it slows down
    if(rpm<-1){
      rpm = -1;
    }
    driveIt(targHeading, rpm);
  }
}
  lDrive.moveVelocity(0);			//stops the drive after it reaches its target
  rDrive.moveVelocity(0);			//stops the drive after it reaches its target
  chassis.tarePosition();
}

//BEGIN Work In Progress S Curve Code
void driveItCurve(double targHeading, double rpm, double delay, double initHead, int multiplier){
	 headg = gyroRotate.getHeading();
	 pros::delay(5);
	// pros::lcd::set_text(1,std::to_string(headg));			//displays actual heading to lcd
	 if(multiplier==1){
	 if(chassis.getPosition()>delay){
	 		gyroRotate.rotateController.setTarget(targHeading);		//sets the heading of the gyro to the desired path angle
	 }
	 else {
		 gyroRotate.rotateController.setTarget(initHead);
	 }
 }else{
	 if(chassis.getPosition()<delay){
	 		gyroRotate.rotateController.setTarget(targHeading);		//sets the heading of the gyro to the desired path angle
	 }
	 else {
		 gyroRotate.rotateController.setTarget(initHead);
	 }
 }

	 //output to drivetrain, modeled as arcade to account for gyro correction
	 drive->getModel()->driveVectorVoltage(rpm, gyroRotate.rotateController.step(gyroRotate.getHeading()));
}
void curveProfiling(double target, double error, long double aggr, double targHeading, double delay, double initHead, bool forward, int timeOut){
 double rpm = 0;																					//output speed in millivolts
 int startTime= pros::millis();
 chassis.tarePosition();
 if (forward==true){
	 multiplier = 1;
 }
 else{
	 multiplier = -1;
 }
 if(multiplier == 1){
 while(chassis.getPosition()<(target-error)  && (pros::millis()<=startTime+timeOut)){						//while the current position is less than the target, drive
 //	std::cout << std::to_string(rpm) << ", " << chassis.getPosition() << std::endl;				//output rpm to console, used for troubleshooteing
	 rpm = multiplier*sqrtl(aggr*(-1*chassis.getPosition()+target));	//sqrt function to control velocity as it approaches the target position, aggr is how fast it slows down
	 if(rpm>1){
		 rpm = 1;
	 }
	 else if(rpm<-1){
		 rpm = -1;
	 }
	 driveItCurve(targHeading, rpm, delay, initHead, multiplier);
 }
}else{
	while(chassis.getPosition()>(target+error)  && (pros::millis()<=startTime+timeOut)){
					//while the current position is less than the target, drive
  //	std::cout << std::to_string(rpm) << ", " << chassis.getPosition() << std::endl;				//output rpm to console, used for troubleshooteing
 	 rpm = multiplier*sqrtl(aggr*(-1*chassis.getPosition()-target));	//sqrt function to control velocity as it approaches the target position, aggr is how fast it slows down
 	 if(rpm<-1){
 		 rpm = -1;
 	 }
	 //pros::lcd::set_text(3,std::to_string(rpm));
 	 driveItCurve(targHeading, rpm, delay, initHead, multiplier);
  }
 }
 lDrive.moveVelocity(0);			//stops the drive after it reaches its target
 rDrive.moveVelocity(0);			//stops the drive after it reaches its target
 chassis.tarePosition();
}

//Noah Stuff

void profile(double target, double error, long double aggr, double targHeading){
double rpm = 0;	//output speed in millivolts
	long startTime=pros::millis();

	if(target>chassis.getPosition()){
  while(chassis.getPosition()<(target-error)){						//while the current position is less than the target, drive
	//	std::cout << std::to_string(rpm) << ", " << chassis.getPosition() << std::endl;				//output rpm to console, used for troubleshooteing
    if(chassis.getPosition()<= (.5*target)){

	rpm = sqrt(10*aggr*(chassis.getPosition()+target));	
	
	} else if (chassis.getPosition()>=(.5*target)){
	rpm = sqrtl(aggr*(-1*chassis.getPosition()+target)); //sqrt function to control velocity as it approaches the target position, aggr is how fast it slows down
	}
	
	if(rpm>1){
      rpm = 1;
    }
    driveIt(targHeading, rpm);
  }
 }else{
	while(chassis.getPosition()>(target+error)){						//while the current position is less than the target, drive
	//	std::cout << std::to_string(rpm) << ", " << chassis.getPosition() << std::endl;				//output rpm to console, used for troubleshooteing
    // rpm = -sqrtl(-aggr*(-1*chassis.getPosition()+target));	//sqrt function to control velocity as it approaches the target position, aggr is how fast it slows down
    
	if(chassis.getPosition()<= (.5*target)){
	rpm = -10*aggr*sqrt((chassis.getPosition()-target));	

	} else if (chassis.getPosition()>=(.5*target)){
	rpm = -aggr*sqrtl(-chassis.getPosition()); //sqrt function to control velocity as it approaches the target position, aggr is how fast it slows down
	}
	
	if(rpm<-1){
      rpm = -1;
    }
    driveIt(targHeading, rpm);
  }
}
  lDrive.moveVelocity(0);			//stops the drive after it reaches its target
  rDrive.moveVelocity(0);
  chassis.tarePosition();			//stops the drive after it reaches its target
}