#pragma config(Sensor, S1,     bumperSens,     sensorEV3_Touch, modeEV3Bump)
#pragma config(Sensor, S2,     rightSens,      sensorEV3_Color)
#pragma config(Sensor, S3,     leftSens,       sensorEV3_Color)
#pragma config(Sensor, S4,     ultraSens,      sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          rightMotor,    tmotorEV3_Large, openLoop, driveRight, encoder)
#pragma config(Motor,  motorB,           clawMotorArm,             tmotorEV3_Large, openLoop)
#pragma config(Motor,  motorC,          clawMotorHand,     tmotorEV3_Large, openLoop)
#pragma config(Motor,  motorD,          leftMotor,     tmotorEV3_Large, openLoop, driveLeft, encoder)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//RECONFIG SENSORS/MOTORS !!!!

/*
Authors: Kollen Gruizenga & John Weisbrod
Function Name: ballFind.c
Function Purpose:
Assignment Name: Ball Finder
*/

//constant program properties -- Forward is NEGATIVE for our motors.
const int FSPD = -30;
const int BSPD = 30;
const int C_FSPD = 10;
const int C_BSPD = -10
const int LIGHT_THRESHOLD = 5;

//indicator bool's for arbitrater
bool bumper_active = false;
bool ball_present_active = false;
bool bank_active = false;
bool carrying_ball = false;

//move command values for switch case
const int MOVE_STOP = 1;
const int MOVE_FWD = 2;
const int MOVE_BCKWD = 3;
const int MOVE_LEFT_TURN = 4;
const int MOVE_RIGHT_TURN = 5;
const int MOVE_LEFT_ARC = 6;
const int MOVE_RIGHT_ARC = 7;
const int MOVE_HARD_LEFT = 8;
const int MOVE_SLIGHT_LEFT = 9;
const int MOVE_SLIGHT_RIGHT = 10;
const int MOVE_CLAW_ARM_DOWN = 11;
const int MOVE_CLAW_ARM_UP = 12;
const int MOVE_CLAW_HAND_CLOSED = 13;
const int MOVE_CLAW_HAND_OPEN = 14;
const int MOVE_BACK_RIGHT = 15;


//move process value/parameter
int move_proc_dist = 5;
int spin_deg = 0;

//init behavior commands
int bumper_command = MOVE_STOP;
int ball_present_cmd = MOVE_STOP;
int in_bank_command = MOVE_STOP;


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


void Move (int op){
	switch (op){
		case MOVE_STOP:
			stopAllMotors();
			break;
		case MOVE_FWD:
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
        case MOVE_SLIGHT_LEFT:
            setMotorSyncTime(leftMotor, rightMotor, 0, 400, BSPD);
            setMotorSyncEncoder(leftMotor, rightMotor, -20, 175, FSPD);
            break;
        case MOVE_SLIGHT_RIGHT:
            setMotorSyncTime(leftMotor, rightMotor, 0, 400, BSPD);
            setMotorSyncEncoder(leftMotor, rightMotor, 20, 175, FSPD);
            break;
        case MOVE_CLAW_ARM_DOWN:
            setMotorTarget(clawMotorArm, 120, C_BSPD);
            break;
        case MOVE_CLAW_ARM_UP:
            setMotorTarget(clawMotorArm, 120, C_FSPD);
            break;
        case MOVE_CLAW_HAND_CLOSED:
            setMotorTarget(clawMotorHand, 60, C_FSPD);
            break;
        case MOVE_CLAW_HAND_OPEN:
            setMotorTarget(clawMotorHand, 60, C_BSPD);
            break;
        case MOVE_BACK_RIGHT:
            //move back
            setMotorSyncTime(leftMotor, rightMotor, 0, 1000, BSPD);
            //turn right
            setMotorSyncEncoder(leftMotor, rightMotor, 100, 175, FSPD);
            break;
		default:
			stopAllMotors();
	}
//	sleep (500);

}


void grab_ball(){
    //move claw down to ground
    Move(MOVE_CLAW_ARM_DOWN);
    waitUntilMotorStop(clawMotorArm);
    //pinch claws together around ball
    Move(MOVE_CLAW_HAND_CLOSED);
    waitUntilMotorStop(clawMotorHand);
    //move claw up
    Move(MOVE_CLAW_ARM_UP);
    waitUntilMotorStop(clawMotorArm);
    
    //inform that we are carrying ball
    carrying_ball = true;
    
}


void drop_ball(){
    //bring claw down
    Move(MOVE_CLAW_ARM_DOWN)
    waitUntilMotorStop(clawMotorArm);
    //release claw pinch (releasing ball)
    Move(MOVE_CLAW_HAND_OPEN);
    waitUntilMotorStop(clawMotorHand);
    //bring claw back up
    Move(MOVE_CLAW_ARM_UP);
    waitUntilMotorStop(clawMotorArm);
    
    //inform we no longer carry a ball
    carrying_ball = false;
}


bool check_color(int val){
    int lightVal = SensorValue[rightSens];
    
////    if ((lightVal >= val - LIGHT_THRESHOLD)
//    if ((lightVal + LIGHT_THRESHOLD) >= val){
//        return true;
//    }
    //val-threshold < lightVal < val+threshold
    if (lightVal >= (val - LIGHT_THRESHOLD)) && (lightVal <= (val + LIGHT_THRESHOLD)) {
        return true;
    }
    return false;
}


task in_bank(){
    int colorVal;
    
    while(true){
        //get color val
        colorVal = SensorValue[rightSens];
        
        //if over bank
        if (colorVal <= BLACK+LIGHT_THRESHOLD){
            //if carrying ball, stay in bank to drop
            if (carrying_ball){
                in_bank_command = MOVE_STOP;
            }
            //if not carrying ball, exit bank (back up, turn right)
            else{
                in_bank_command = MOVE_BACK_RIGHT;
            }
            bank_active = true;
        }
    }
    
    bank_active = false;
    releaseCPU();
}


////When robot bumper is pressed (hit wall)
//task bumper_hit(){
//    int touchSensVal, colorVal;
//    int BLACK = 1;
//    while (true){
//        //get touch sensor status
//        touchSensVal = SensorValue[bumperSens];
//        colorVal = SensorValue[rightSens];
//
//        //if bumper pressed
//        if (touchSensVal > 0){
//            //check if we are carrying ball already to drop at corner black spot
//            if (carrying_ball == true){
//                //if in black spot to drop ball
//                if (colorVal <= BLACK+LIGHT_THRESHOLD){
//                    //drop ball in black corner
//                    drop_ball();
//                    //move backward once (not against wall), turn left to continue..
//                    bumper_command = MOVE_SLIGHT_LEFT;
//                }
//                //else reorient to the right to continue forward
//                else{
//                    bumper_command = MOVE_BACK_RIGHT;
//                }
//            }
//            //else, turn from wall and continue search
//            else{
//                //command to move back a bit then turn right small amount for new direction
//                bumper_command = MOVE_SLIGHT_RIGHT;
//            }
//
//            bumper_active = true;
//        }
//    }
//    bumper_active = false;
//    releaseCPU();
//}


//When robot bumper is pressed (hit wall)
task bumper_hit(){
    int touchSensVal;
    while (true){
        //get touch sensor status
        touchSensVal = SensorValue[bumperSens];
        
        //if bumper pressed
        if (touchSensVal > 0){
            //else, turn from wall and continue search.. move back, right slightly
            bumper_command = MOVE_SLIGHT_RIGHT;
            bumper_active = true;
        }
    }
    bumper_active = false;
    releaseCPU();
}


task ball_present(){
    //if bumper not pressed + distance sensor sees object within limit distance
    int touchSensVal, distSensVal;
    
    while (true){
        //get sensor status
        touchSensVal = SensorValue[bumperSens];
        distSensVal = SensorValue[ultraSens];
        
        //make sure we aren't already transporting ball
        if (carrying_ball == false){
            // sqrt(ball_x^2 + ball_y^2)
            //x = half the length of bumper, y=distance directly straight from sensor to bumper
            
            //if bumper not pressed & object in front of bumper
//            if ((touchSensVal != 1) && (distSensVal <= sqrt(double(x^2 + y^2)))){
            if ((touchSensVal != 1) && distSensVal <= 20){
                ball_present_cmd = MOVE_STOP;
                ball_present_active = true;
            }
        }
    }
    ball_present_active = false;
    releaseCPU();
}


task main(){
    //set motor power levels (to full)
    motor[rightMotor] = 127;
    motor[leftMotor] = 127;
    //begin tasks, create command var
    int motorCommand;
    int WHITE = 21;
    int BLACK = 1;
    
    startTask(ball_present);
    startTask(bumper_hit);

    //run/move robot based on tasks
    while(true){
        //default move forward
        motorCommand = MOVE_FWD;
        
        //if found ball to pick up (false if already carrying ball) -- PRIORITY 3
        if (ball_present_active){
            motorCommand = ball_present_cmd;
            //only grab ball if it is white -- FOR "BALL FINDER"
            if (check_color(WHITE)){
                grab_ball();
            }
        }
        
        //if in bank (black corner) -- PRIORITY 2
        if (bank_active){
            motorCommand = in_bank_command;
        }
        
//        //if have ball picked up and are at a wall -- PRIORITY 2
//        if (bumper_active && carrying_ball){
//            //while rightSens != BLACK
//            // on black: rightSens <= BLACK + THRESH
//            while(rightSens > BLACK+LIGHT_THRESHOLD){
//                //if hit wall (and not on black).. reorient and continue
//                if(bumper_active){
//                    Move(MOVE_BCKWD);
//                    Move(MOVE_RIGHT_TURN);
//                }
//                Move(MOVE_FWD);
//            }
//            //drop ball at corner
//            drop_ball();
//        }
        
        //if bumper pressed (ran into wall.. or other robot?) -- PRIORITY 1
        if (bumper_active){
            motorCommand = bumper_command;
        }
        
        //move robot whichever direction
        Move(motorCommand);
        
        //if robot in bank location
        if (bank_active){
            //score ball in bank
            drop_ball();
            //back up a bit, turn left for new direction
            Move(MOVE_SLIGHT_LEFT);
        }
        
        releaseCPU();
    }

}//end task
