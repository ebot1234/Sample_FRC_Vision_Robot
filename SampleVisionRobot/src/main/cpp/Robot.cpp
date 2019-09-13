#include "Robot.h"
#include <iostream>
#include <stdlib.h>
#include <frc/Timer.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <cscore.h>
#include <CameraServer/CameraServer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "ctre/Phoenix.h"
#include <AHRS.h>
#include <pathfinder.h>
using namespace frc;
using namespace std;

Robot::Robot() {
  
}

//Network Table Varibles
nt::NetworkTableEntry Angle;
nt::NetworkTableEntry Distance;
//Motor Varibles
TalonSRX fl = {0};
TalonSRX fr = {1};
TalonSRX bl = {2};
TalonSRX br = {3};
//Driver Controller Varibles
frc::Joystick stick0{0};
frc::Joystick stick1{1};
//USB Camera Varibles
cs::UsbCamera camera1;
cs::VideoSink server;
//Gyro Varibles
AHRS ahrs;


void Robot::RobotInit() {
//Network Tables Setup
 auto inst = nt::NetworkTableInstance::GetDefault();
 auto table = inst.GetTable("Vision");
 Angle = table->GetEntry("angle");
 Distance = table->GetEntry("distance");
 //Motor Setup
 fl.SetInverted(false);
 fl.SetNeutralMode(Coast);
 fr.SetInverted(true);
 fr.SetNeutralMode(Coast);
 bl.SetInverted(false);
 bl.SetNeutralMode(Coast);
 br.SetInverted(true);
 br.SetNeutralMode(Coast);
 //Camera Setup
 camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
 server = frc::CameraServer::GetInstance()->GetServer();
 printf("Robot Ready\n");
}

//Absolute Value Function
double dabs(double input){
        if(input >= 0){
		    return input;
	    }
	    else if(input < 0){
		    return -input;
	    }
	    else{
		    std::cout << "Math is broken" << std::endl;
	    }
}

//Mapping function to scale values
double map(double inMax, double inMin, double outMax, double outMin, double input){
	double output;

	output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

	return output;
}
//Sets the speed of the drivetrain motors
void setSpeed(double left, double right){
    fl.Set(ControlMode::PercentOutput, left);
    bl.Set(ControlMode::PercentOutput, left);
    fr.Set(ControlMode::PercentOutput, right);
    br.Set(ControlMode::PercentOutput, right);
    printf("Motor Speeds: Left: %f, Right %f", left, right);
}
//PID Turning -- Positive number for clockwise, Negative for anti-clockwise --
void PIDTurn(int angle, double timeOut){
    ahrs.Reset();
    Timer t1;
    t1.Get();
    t1.Reset();
    t1.Start();

    Wait(0.25);

    double errorPrior = 0; //Error from previous cycles
    double intergal = 0; //Starts at 0 since that is what works
    double derivative = 0; //Not really used but looks nice
    double interationTime = 0.1; //Time it should take to loop
    int timeBuffer = 0;

    double kP = 4; //Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
    double kI = 0.25; //Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
    double kD = 0.1; //Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3

    double error = angle - ahrs.GetYaw();
    double output;

    while(timeBuffer < 5 && t1.Get() < timeOut){
        error = angle - ahrs.GetYaw(); //Error = final - current
        intergal = intergal + (error * interationTime);
        derivative = (error - errorPrior) / interationTime;

        output = (kP * error) + (kI * intergal) + (kD * derivative); //Sum all the parts together
        output = map(output, -angle, angle, -0.7, 0.7);

        if (angle < 0){
            setSpeed(output, -output);
        }
        else if (angle > 0){
            setSpeed(-output, output);
        }
        else {
            printf("Angle = 0");
        }

        if (fabs(error) < 3){
            timeBuffer++;
        }
        else {
            timeBuffer = 0;
        }

        errorPrior = error; //Updates the error counts
        printf("Current Angle %f", ahrs.GetYaw());
        printf("Setpoint %f", angle);

        Wait(interationTime); //Wait the loop time
    }

    setSpeed(0.0, 0.0);
    printf("PID Turn Complete");
}

void driveWhile(double distance, double speed){
    fl.SetSelectedSensorPosition(0,0,0);
    fr.SetSelectedSensorPosition(0,0,0);

    double wheel_radius = 2.5;
    double wheel_cir = 2 * 3.14159265 * wheel_radius;
    double PPR = 1440;
    double encIn = PPR / wheel_cir;
    double encTarget = distance * encIn;

    if (distance > 0){
        while(fr.GetSelectedSensorPosition(0) < encTarget){
            setSpeed(-speed, -speed);
        }
    }
    else if(distance < 0){
        while(fr.GetSelectedSensorPosition(0) > encTarget){
            setSpeed(speed, speed);
        }
    }

    setSpeed(0.0, 0.0);
}

void generatePath(){
    int point_length = 3;
    Waypoints points[point_length];
    
    //Define points
}


void Robot::Autonomous() {
    ahrs.Reset();

    //Drive to vision distance

}

void Robot::OperatorControl() {

}


void Robot::Test() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
