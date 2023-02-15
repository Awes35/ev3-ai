#pragma config(Sensor, S1,     bumperSens,     sensorEV3_Touch, modeEV3Bump)
#pragma config(Sensor, S2,     rightSens,      sensorEV3_Color)
#pragma config(Sensor, S3,     leftSens,       sensorEV3_Color)
#pragma config(Sensor, S4,     ultraSens,      sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          rightMotor,    tmotorEV3_Large, openLoop, driveRight, encoder)
#pragma config(Motor,  motorB,           ,             tmotorEV3_Large, openLoop)
#pragma config(Motor,  motorC,          clawMotor,     tmotorEV3_Large, openLoop)
#pragma config(Motor,  motorD,          leftMotor,     tmotorEV3_Large, openLoop, driveLeft, encoder)

#include <stdlib.h>

/*
Authors: Kollen Gruizenga & John Weisbrod
Function Name: lineFollow.c
Function Purpose: This function will allow the robot to follow a line, created by the intersection of a light and dark space.
					When running, the robot will seek to follow the black paper "line", keeping the black paper on the right the whole time.
                    If the robot finds that the sensors see the same color (based on light value), the robot will either
                        - Turn left if the robot is seeing the line (black)
                        - Turn right if the robot is away from line (white)
Assignment Name: Line Follower
*/

//constant program properties
const int FSPD = -30;
const int BSPD = 30;
const int GEN_THRES = 5;
const int WHITE_THRESHOLD = 20;
const int BLACK_THRESHOLD = 7;

//indicator bool's for arbitrater
bool line_follow_active = false;
bool bumper_active = false;

//move command values for switch case
const int MOVE_STOP = 1;
const int MOVE_FWD = 2;
const int MOVE_BCKWD = 3;
const int MOVE_LEFT_TURN = 4;
const int MOVE_RIGHT_TURN = 5;
const int MOVE_LEFT_ARC = 6;
const int MOVE_RIGHT_ARC = 7;
const int MOVE_HARD_LEFT = 8;

//init behavior commands
int line_follow_command = MOVE_STOP;
int bumper_command = MOVE_STOP;


void moveRobotForward(){
		setMotorSpeed(leftMotor, FSPD);		//Set the leftMotor (motor1) to half power (50)
		setMotorSpeed(rightMotor, FSPD);  //Set the rightMotor (motor6) to half power (50)
}

void moveRobotBackward(){
	setMotorSpeed(leftMotor, BSPD);		//Set the leftMotor (motor1) to half power (50)
	setMotorSpeed(rightMotor, BSPD);  //Set the rightMotor (motor6) to half power (50)
}

void slowFwdRight(){
	// This makes it go slow right (right motor turning at 50% of left one)
	//setMotorSyncTime(leftMotor, rightMotor, 50, 1000, -25);
	setMotorSpeed(leftMotor, 0);			//Set the leftMotor (motor1) to Off
	setMotorSpeed(rightMotor, FSPD);  	//Set the rightMotor (motor6) to full power forward (100)
	//sleep(1500);											//Wait for 1 second before continuing on in the program.
}

void slowFwdLeft(){
	// This makes it go slow left (left motor turning at 50% of right one)
	//setMotorSyncTime(leftMotor, rightMotor, -50, 1000, -25);
	setMotorSpeed(leftMotor, FSPD);		//Set the leftMotor (motor1) to full power forward (100)
	setMotorSpeed(rightMotor, 0);  		//Set the rightMotor (motor6) to full power reverse (-100)
	//sleep(1500);											//Wait for 1 second before continuing on in the program.
}

/*
Command that tells the robot what method of movement it should take
Parameters: operation (what to do)
Return: None
Important Note: setMotorSyncTime(motor1, motor2, turnRatio aka should one motor s
pin more than the other, howLongTheMotorsMoveFor, how much power goes to the wheel with a greater value in the ratio)
*/
void Move (int op){
	switch (op){
		case MOVE_STOP:
			stopAllMotors();
			break;
		case MOVE_FWD: // basic movement forward, straight ahead
            // (left and right, move equally, for 1000 milliseconds, at standard forward speed)
			setMotorSyncTime(leftMotor, rightMotor, 0, 1000, FSPD);
			break;
		case MOVE_BCKWD:
            // (left and right, move equally, for 1000 milliseconds, at standard backwards speed)
			setMotorSyncTime(leftMotor, rightMotor, 0, 1000, BSPD);
			break;
		case MOVE_LEFT_TURN:
            // (left and right, apply negative power to left and pos to right, move for 175 milliseconds)
			setMotorSyncEncoder(leftMotor, rightMotor, -100, 175, FSPD);
			break;
		case MOVE_RIGHT_TURN:
            // (left and right apply positive power to left, and negative to right, move for 175)
			setMotorSyncEncoder(leftMotor, rightMotor, 100, 175, FSPD);
			break;
		case MOVE_LEFT_ARC:
            // (left and right, apply zero power to left, and all power to right, move for 500 milliseconds)
			setMotorSyncTime(leftMotor, rightMotor, -30, 500, FSPD);
			break;
		case MOVE_RIGHT_ARC:
            //(left and right, apply zero power to right, and all power to left, move for 500 milliseconds)
			setMotorSyncTime(leftMotor, rightMotor, 30, 500, FSPD);
			break;
		case MOVE_HARD_LEFT:
            // left and right move equally, for 400 milliseconds backwards
			setMotorSyncTime(leftMotor, rightMotor, 0, 400, BSPD);
            // apply negative power to left and positive to right, move for 175 milliseconds left
			setMotorSyncEncoder(leftMotor, rightMotor, -100, 175, FSPD);
			break;
		default:
			stopAllMotors();
	}
	sleep (500);

}

//When robot bumper is pressed, move robot backwards away from object
task bumper_hit(){
	int touchSensVal;

	while (true){
		//get touch sensor status
		touchSensVal = SensorValue[bumperSens];
	//if bumper pressed
		if (touchSensVal > 0){
			//indicate command update scenario to arbitrater, with backwards command
			bumper_active = true;
			bumper_command = MOVE_BCKWD;
		}
	}
	bumper_active = false;
	releaseCPU();
}


// -- RECEIVING ERRORS; we talked to you in class about this one, issues with comparing variable values with RobotC "multithreading" .. (var's not updating)
//---------------------------------------------------------------------------
//task line_follow(){ //brighter = higher val
//	//var's for storing off last val's and current val's
//    int rightSensVal, leftSensVal;
//    int lastRightSensVal, lastLeftSensVal;
//
//    //load initial "last" values and move forward once to begin
//    lastRightSensVal = SensorValue[rightSens];
//    lastLeftSensVal = SensorValue[leftSens];
//    Move(MOVE_FWD);
//
//    while (true){
//    	//read current left/right sensor values
//        rightSensVal = SensorValue[rightSens];
//        leftSensVal = SensorValue[leftSens];
//
//        //if sensors show same brightness.. (strayed from straddling line)
//        if (abs(rightSensVal - leftSensVal) < LIGHT_THRESHOLD){
//        	//indicate command update scenario to arbitrater
//            line_follow_active = true;
//            //if right changed significantly.. move back towards Right
//            if ((abs(rightSensVal - lastRightSensVal) - LIGHT_THRESHOLD) > 0){
//                line_follow_command = MOVE_RIGHT_ARC;
//            }
//            //if left changed significantly.. move back towards Left
//            else if ((abs(leftSensVal - lastLeftSensVal) - LIGHT_THRESHOLD) > 0){
//                line_follow_command = MOVE_LEFT_ARC;
//            }
//        }
//        //save last value
//        lastRightSensVal = rightSensVal;
//        lastLeftSensVal = leftSensVal;
//
//        line_follow_active = false;
//        releaseCPU();
//    }
//
//}//end task


//function to follow line, keeping black color on the right
//following left side of black paper
task line_follow(){
    //When the robot follows the line, the line should be in between the sensors, meaning
        // right sensor should be seeing black, and left sensor should be seeing white.
    
    int leftSensVal, rightSensVal;
    
    while (true){
        //get sensor readings
        leftSensVal = SensorValue[leftSens];
        rightSensVal = SensorValue[rightSens];
        
        //if both sensors are on same color, difference of 5 tolerated
        // "leftSensVal == rightSensVal"
        if (abs(leftSensVal - rightSensVal) < GEN_THRESH){
            //if both sensors are on black (within black threshold)
            if (rightSensVal < BLACK_THRESHOLD){
                line_follow_command = MOVE_LEFT_ARC;
                line_follow_command = true;
            }
            //if both sensors are on white (within white threshold)
            else if (rightSensVal > WHITE_THRESHOLD){
                line_follow_command = MOVE_RIGHT_ARC;
                line_follow_command = true;
            }
        }
        
        line_follow_active = false;
        releaseCPU();
        
    } //end while
    
} //end task


task main(){
	//set motor power levels (to full)
	motor[rightMotor] = 127;
	motor[leftMotor] = 127;
	//begin tasks, create command var
    int motorCommand;
    startTask(line_follow);
    startTask(bumper_hit);

    //run/move robot based on tasks
    while(true){
    	//default move forward
        motorCommand = MOVE_FWD;
        
        //if task has a change and updated the move command needed
        if (line_follow_active){
            motorCommand = line_follow_command;
        }
        //if task has a change and updated the move command needed
        if (bumper_active){
            motorCommand = bumper_command;
        }

        //move robot whichever direction
        Move(motorCommand);
        releaseCPU();
    }

}//end task

