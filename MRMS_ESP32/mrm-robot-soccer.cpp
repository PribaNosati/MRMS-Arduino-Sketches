#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b2.h>
//#include <mrm-ir-finder2.h>
#include <mrm-ir-finder3.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include "mrm-robot-soccer.h"

/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotSoccer::RobotSoccer(char name[]) : Robot(name) {
	motorGroup = new MotorGroupStar(this, mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 3);

	pidXY = new Mrm_pid(0.5, 200, 0); // PID controller, regulates motors' speeds for linear motion in the x-y plane
	pidRotation = new Mrm_pid(0.5, 100, 0); // PID controller, regulates rotation around z axis
	actionAdd(new ActionSoccerPlay(this));
	actionCatch = new ActionSoccerCatch(this);
	actionIdle = new ActionSoccerIdle(this);

	mrm_mot4x3_6can->directionChange(0); // Uncomment to change 1st wheel's rotation direction
	mrm_mot4x3_6can->directionChange(1); // Uncomment to change 2nd wheel's rotation direction
	mrm_mot4x3_6can->directionChange(2); // Uncomment to change 3rd wheel's rotation direction
	mrm_mot4x3_6can->directionChange(3); // Uncomment to change 4th wheel's rotation direction

	mrm_8x8a->actionSet(_actionLoop, 1); // Button 1 starts user defined loop() function
}

/** Rear distance to wall
@return - in mm
*/
uint16_t RobotSoccer::back() {
	return mrm_lid_can_b2->reading(2); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Ball's direction
@return - robot's front is 0�, positive angles clockwise, negative anti-clockwise. Back of the robot is 180�.
*/
int16_t RobotSoccer::ballAngle() {
	return mrm_ir_finder3->angle();
}

/** Read barrier
@return - true if interrupted
*/
bool RobotSoccer::barrier() {
	return analogRead(35) < 300; // Adjust this value
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotSoccer::bitmapsSet() {
	uint8_t red[8] = { 0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000 };
	uint8_t green[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
	mrm_8x8a->bitmapCustomStore(red, green, 7);
}

/** Reads push button switch
@number - 0 to 3, push button's ordinal number
@return - true if pressed
*/
bool RobotSoccer::button(uint8_t number) {
	return mrm_8x8a->switchRead(number);
}

/** Line sensor - brightness of the surface
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - brightness as an analog value.
*/
uint16_t RobotSoccer::brightness(uint8_t transistorNumber, uint8_t deviceNumber) {
	return mrm_ref_can->reading(transistorNumber, deviceNumber);
}

/** Go around the ball and approach it.
*/
void RobotSoccer::catchBall() {
	if (mrm_ir_finder3->distance() == 0) {

	}
	else
		actionSet(actionIdle);
}

/** Front distance to wall
@return - in mm
*/
uint16_t RobotSoccer::front() {
	return mrm_lid_can_b2->reading(0); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
@param speed - 0 to 100.
@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
Values between -180 and 180.
@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
numbers because a value 100 turns on all the motors at maximal speed.
@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/
void RobotSoccer::go(float speed, float angleDegrees, float rotation, uint8_t speedLimit) {
	motorGroup->go(speed, angleDegrees, rotation, speedLimit);
}

/** Test - go straight ahead.
*/
void RobotSoccer::goAhead() {
	const uint8_t speed = 40;
	motorGroup->go(speed);
	end();
}

/**Compass
@return - North is 0�, clockwise are positive angles, values 0 - 360.
*/
float RobotSoccer::heading() {
	return mrm_imu->heading();
}

/** No ball detected - return to Your goal.
*/
void RobotSoccer::idle() {
	if (setup())
		headingToMaintain = mrm_imu->heading();
	if (mrm_ir_finder3->distance() == 0 && false)
		actionSet(actionCatch);
	else {
		//print("\n\rError: %i = %i - 300\n\r", (int)(300 - robot->mrm_lid_can_b2->reading(3)), robot->mrm_lid_can_b2->reading(3));
		float errorL = 900 - mrm_lid_can_b2->reading(3);
		float errorR = mrm_lid_can_b2->reading(1) - 900;
		motorGroup->goToEliminateErrors(errorL > errorR ? errorL : errorR, 150 - mrm_lid_can_b2->reading(2), headingToMaintain - mrm_imu->heading(), pidXY, pidRotation, true);
		//delay(500);
	}
}

/** Left distance to wall
@return - in mm
*/
uint16_t RobotSoccer::left() {
	return mrm_lid_can_b2->reading(3); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Line sensor
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - true if white line found
*/
bool RobotSoccer::line(uint8_t transistorNumber, uint8_t deviceNumber) {
	return !mrm_ref_can->dark(transistorNumber, deviceNumber);
}

/** Custom test.
*/
void RobotSoccer::loop() {
	motorGroup->go(100, 0);
	//static int initialDirection;
	//if (setup()) {
	//	initialDirection = mrm_imu->heading();
	//}

	//int rotationalError = mrm_imu->heading() - initialDirection;

	//int positionError = 0;
	//if (mrm_lid_can_b2->reading(1) > 80)
	//	positionError = 91 - mrm_lid_can_b2->reading(1);
	//else if (mrm_lid_can_b2->reading(3) > 80)
	//	positionError = mrm_lid_can_b2->reading(1) - 91;

	//motorGroup->go(abs(positionError), positionError < 0 ? -90 : 90, rotationalError);
}

/** Starts robot
*/
void RobotSoccer::play() {
	if (motorGroup == NULL) {
		print("Define robot->motorGroupStar first.\n\r");
		return;
	}
	headingToMaintain = mrm_imu->heading();
	actionSet(actionIdle);
}

/**Pitch
@return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0�.
*/
float RobotSoccer::pitch() {
	return mrm_imu->pitch();
}

/** Right distance to wall
@return - in mm
*/
uint16_t RobotSoccer::right() {
	return mrm_lid_can_b2->reading(1); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Roll
@return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0�.
*/
float RobotSoccer::roll() {
	return mrm_imu->roll();
}

/** Display fixed sign stored in sensor
@image - sign's number
*/
void RobotSoccer::sign(uint8_t number) {
	mrm_8x8a->bitmapDisplay(number);
}
