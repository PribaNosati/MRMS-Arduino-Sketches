#include <Interfaces.h>
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
	motorGroup = new MotorGroupStar(mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 3);

	// LED signs to be assigned to different actions follow. It is easier to follow action flow by checking the display.

	// LED Calibrate
	Mrm_8x8a::LEDSignText* signCalibrate = new Mrm_8x8a::LEDSignText(); // Here, a text will be displayed instead of a 8x8 bitmap.
	strcpy(signCalibrate->text, "Calibr.");

	// LED approach opponent's goal
	Mrm_8x8a::LEDSignBitmap* signGoalApproach = new Mrm_8x8a::LEDSignBitmap();
	// Each "1" will turn on a single LED.
	signGoalApproach->green[0] = 0b01111110;
	signGoalApproach->green[1] = 0b01000010;
	signGoalApproach->green[2] = 0b00000000;
	signGoalApproach->green[3] = 0b00000000;
	signGoalApproach->green[4] = 0b00000000;
	signGoalApproach->green[5] = 0b00000000;
	signGoalApproach->green[6] = 0b00000000;
	signGoalApproach->green[7] = 0b00000000;

	// LED 8x8 idle
	Mrm_8x8a::LEDSignBitmap* signIdle = new Mrm_8x8a::LEDSignBitmap();
	// Each "1" will turn on a single LED.
	signIdle->green[0] = 0b00000000;
	signIdle->green[1] = 0b00000000;
	signIdle->green[2] = 0b00000000;
	signIdle->green[3] = 0b00000000;
	signIdle->green[4] = 0b00000000;
	signIdle->green[5] = 0b00000000;
	signIdle->green[6] = 0b01000010;
	signIdle->green[7] = 0b01111110;

	// LED 8x8 line avoid
	Mrm_8x8a::LEDSignBitmap* signLineAvoid = new Mrm_8x8a::LEDSignBitmap();
	signLineAvoid->green[0] = 0b11111111;
	signLineAvoid->green[1] = 0b10000001;
	signLineAvoid->green[2] = 0b10000001;
	signLineAvoid->green[3] = 0b10000001;
	signLineAvoid->green[4] = 0b10000001;
	signLineAvoid->green[5] = 0b10000001;
	signLineAvoid->green[6] = 0b10000001;
	signLineAvoid->green[7] = 0b11111111;

	// LED 8x8 catch
	Mrm_8x8a::LEDSignBitmap* signCatch = new Mrm_8x8a::LEDSignBitmap();
	signCatch->green[0] = 0b00011000;
	signCatch->green[1] = 0b00011000;
	signCatch->green[2] = 0b00000000;
	signCatch->green[3] = 0b00100100;
	signCatch->green[4] = 0b01011010;
	signCatch->green[5] = 0b01000010;
	signCatch->green[6] = 0b00100100;
	signCatch->green[7] = 0b00011000;

	// LED Bounce
	Mrm_8x8a::LEDSignText* signBounce = new Mrm_8x8a::LEDSignText(); // Here, a text will be displayed instead of a 8x8 bitmap.
	strcpy(signBounce->text, "Bounce");

	// Actions
	pidXY = new Mrm_pid(0.5, 100, 0); // PID controller, regulates motors' speeds for linear motion in the x-y plane: 4, 100, 0 - ok.
	pidRotation = new Mrm_pid(2.0, 100, 0); // PID controller, regulates rotation around z axis

	actions->insert({"soc", new ActionRobotSoccer(this, "Soccer play", 1, Board::BoardId::ID_ANY, NULL, &RobotSoccer::play)});
	actions->insert({"bou", new ActionRobotSoccer(this, "Soccer bounce", 1, Board::BoardId::ID_ANY, signBounce, &RobotSoccer::bounce)});
	actions->insert({"clb", new ActionRobotSoccer(this, "Soccer calibrate", 1, Board::BoardId::ID_ANY, signCalibrate, &RobotSoccer::calibrate)});
	actions->insert({"cat", new ActionRobotSoccer(this, "Soccer catch", 1, Board::BoardId::ID_ANY, signCatch, &RobotSoccer::catchBall)});
	actions->insert({"apr", new ActionRobotSoccer(this, "Soccer approach", 1, Board::BoardId::ID_ANY, signGoalApproach, &RobotSoccer::goalApproach)});
	actions->insert({"idl", new ActionRobotSoccer(this, "Soccer idle", 1, Board::BoardId::ID_ANY, signIdle, &RobotSoccer::idle)});
	actions->insert({"avo", new ActionRobotSoccer(this, "Soccer line avoid", 1, Board::BoardId::ID_ANY, signLineAvoid, &RobotSoccer::lineAvoid)});
	actions->insert({"bar", new ActionRobotSoccer(this, "Soccer barrier test", 1, Board::BoardId::ID_ANY, NULL, &RobotSoccer::barrierTest)});


	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus. For example, ActionWallsTest test is only manu-based, and it is all right.
	// This test is not supposed to be called in code.

	// mrm_mot4x3_6can->directionChange(mrm_mot4x3_6can->devices[0]); // Uncomment to change 1st wheel's rotation direction
	// mrm_mot4x3_6can->directionChange(mrm_mot4x3_6can->devices[1]); // Uncomment to change 2nd wheel's rotation direction
	// mrm_mot4x3_6can->directionChange(mrm_mot4x3_6can->devices[2]); // Uncomment to change 3rd wheel's rotation direction
	// mrm_mot4x3_6can->directionChange(mrm_mot4x3_6can->devices[3]); // Uncomment to change 4th wheel's rotation direction

	// Buttons
	mrm_8x8a->actionSet(actionFind("soc"), 0); // Button 1 starts the play
	mrm_8x8a->actionSet(actionFind("bou"), 1); // Button 2 starts user defined bounce() function
	mrm_8x8a->actionSet(actionFind("loo"), 2); // Button 3 starts user defined loop() function
	mrm_8x8a->actionSet(actionFind("men"), 3); // Button 4 stops the robot and prints main manu

	// Set number of phototransistors in each line sensor.
	mrm_ref_can->transistorCountSet(5, 0); // 5 instead of 6 since IR ball interferes with 6th transistor.
	mrm_ref_can->transistorCountSet(8, 1);
	mrm_ref_can->transistorCountSet(8, 2);
	mrm_ref_can->transistorCountSet(8, 3);
}

/** Rear distance to wall
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/
uint16_t RobotSoccer::back(uint8_t sampleCount, uint8_t sigmaCount) {
	return mrm_lid_can_b2->distance(2, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
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
	return analogRead(SOCCER_BARRIER_PIN) < BARRIER_MID_VALUE; // Adjust this value
}

/** Test barrier
*/
void RobotSoccer::barrierTest(){
	print("%i - %s ball\n\r", analogRead(SOCCER_BARRIER_PIN), barrier() ? "" : "no");
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotSoccer::bitmapsSet() {
	// uint8_t red[8] = { 0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000 };
	// uint8_t green[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
	// mrm_8x8a->bitmapCustomStore(red, green, 7);
}

/** Bouncing off the lines
*/
void RobotSoccer::bounce(){
	static int directionCurrent = 45;
	const int VARIATION = 45;
	const int SPEED = 127;
	const bool AVOID_LINE = false;
	if (setup()){
		headingToMaintain = heading();
	}

	if (AVOID_LINE && lineAny()){
		actionSet("avo");
		if (mrm_ref_can->any(false, 0)) // Front
			directionCurrent = headingRandom(-180, VARIATION);
		if (mrm_ref_can->any(false, 1)) // Right
			directionCurrent = headingRandom(-90, VARIATION);
		if (mrm_ref_can->any(false, 2)) // Back
			directionCurrent = headingRandom(0, VARIATION);
		if (mrm_ref_can->any(false, 3)) // Left
			directionCurrent = headingRandom(90, VARIATION);
	}
	else{
		uint16_t minDistance = left();
		if (right() < minDistance) 
			minDistance = right();
		if (front() < minDistance) 
			minDistance = front();
		if (back() < minDistance) 
			minDistance = back();
		go(map(minDistance, 0, 700, 40, SPEED), directionCurrent, pidRotation->calculate(heading() - headingToMaintain));
		if (left() < 200){
			directionCurrent = headingRandom(90, VARIATION);
			// print("Left %i\n\r", directionCurrent);
		}
		if (right() < 200){
			directionCurrent = headingRandom(-90, VARIATION);
			// print("Right %i\n\r", directionCurrent);
		}
		if (front() < 200){
			directionCurrent = headingRandom(180, VARIATION);
			// print("Front %i\n\r", directionCurrent);
		}
		if (back() < 200){
			directionCurrent = headingRandom(0, VARIATION);
			// print("Back %i\n\r", directionCurrent);
		}
		// for (uint8_t i = 0; i < 4; i++)

		static uint32_t ms = 0;
		if (millis() - ms > 1000){
			print("Target: %i, current: %i\n\r", (int)headingToMaintain, (int)heading());
			ms = millis();
		}
	}
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

/** Calibrate all line sensors
 */
void RobotSoccer::calibrate(){
	go(0, 0, 25, 100);
	mrm_ref_can->calibrate(0);
	mrm_ref_can->calibrate(1);
	mrm_ref_can->calibrate(2);
	mrm_ref_can->calibrate(3);
	go(0, 0, 0, 0);
	end();
}

/** Go around the ball and approach it.
*/
void RobotSoccer::catchBall() {
	if (lineAny())
		actionSet("avo");
	else if (barrier())
		actionSet("apr");
	else if (mrm_ir_finder3->distance() > 100) {
		float direction = -mrm_ir_finder3->angle() - 10;
		if (fabsf(direction) > 7)
			direction += (direction > 0 ? 60 : -60);
		go(40, direction, pidRotation->calculate(heading() - headingToMaintain), 100);
		print("Catch ball, angle: %i\n\r", (int)mrm_ir_finder3->angle());
	}
	else
		actionSet("idl");
}

/** Front distance to wall
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/
uint16_t RobotSoccer::front(uint8_t sampleCount, uint8_t sigmaCount) {
	return mrm_lid_can_b2->distance(0, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
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
	const uint8_t speed = 60;
	motorGroup->go(speed);
	end();
}

/** Approach oppoent's goal
*/
void RobotSoccer::goalApproach(){
	if (lineAny())
		actionSet("avo");
	else if (!barrier())
		actionSet("idl");
	else{
		float errorL = SOCCER_SIDE_DISTANCE_WHEN_CENTERED - left();
		float errorR = right() - SOCCER_SIDE_DISTANCE_WHEN_CENTERED;
		float errorX = fabsf(errorL) > fabsf(errorR) ? errorL : errorR;
		float errorY = front();

		float direction = atan2(errorX, errorY) / PI * 180;
		go(60, direction, pidRotation->calculate(heading() - headingToMaintain));
		print("Goal approach\n\r");
	}
}

/**Compass
@return - North is 0�, clockwise are positive angles, values 0 - 360.
*/
float RobotSoccer::heading() {
	return mrm_imu->heading();
}

float RobotSoccer::headingRandom(int heading, int variation){
	float newHeading = heading + (2 * (rand() % variation) - variation);
	if (newHeading > 180)
		newHeading -= 360;
	else if (newHeading < -180)
		newHeading += 360;
	return newHeading;
}

/** No ball detected - return to Your goal.
*/
void RobotSoccer::idle() {
	if (lineAny())
		actionSet("avo");
	else if (mrm_ir_finder3->distance() > 50)
		actionSet("cat");
	else {
		float errorL = SOCCER_SIDE_DISTANCE_WHEN_CENTERED - left();
		float errorR = right() - SOCCER_SIDE_DISTANCE_WHEN_CENTERED;
		float errorX = left() > right() ? errorL : errorR;
		float errorY = SOCCER_BACK_DISTANCE_WHEN_GOALIE - back();
		float speed = 0; // Default: if no room, stay put
		if (left() > 600 || right() > 600 || back() > 500)
			speed = pidXY->calculate(fabsf(errorX) + fabsf(errorY), false, SOCCER_SPEED_LIMIT);
		float angularSpeed = pidRotation->calculate(heading() - headingToMaintain, false, SOCCER_ANGULAR_SPEED_LIMIT);

		float direction = atan2(errorX, errorY) / PI * 180;
		go(speed, direction, angularSpeed);
		print("Idle: L:%i/%i R:%i/%i Hea:%i\n\r", left(), (int)errorL, right(), (int)errorR, (int)direction);
		//motorGroup->goToEliminateErrors(errorL > errorR ? errorL : errorR, 200 - back(), heading() - headingToMaintain, pidXY, pidRotation, true);
	}
}

/** Left distance to wall
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/
uint16_t RobotSoccer::left(uint8_t sampleCount, uint8_t sigmaCount) {
	return mrm_lid_can_b2->distance(3, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Line sensor
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - true if white line found
*/
bool RobotSoccer::line(uint8_t transistorNumber, uint8_t deviceNumber) {
	return !mrm_ref_can->dark(transistorNumber, deviceNumber);
}

bool RobotSoccer::lineAny(){
	const bool AVOID_LINE = false;
	const bool ENABLE_FRONT_SENSOR = false;
	if (AVOID_LINE)
		for (uint8_t i = (ENABLE_FRONT_SENSOR ? 0 : 1); i < 4; i++)
			if (mrm_ref_can->any(false))
				return true;
	return false;
}

void RobotSoccer::lineAvoid(){
	static TriState lineLeft;
	static TriState lineFront;
	static float escapeDirection;
	const uint16_t WALL_DISTANCE = 550;

	if (setup()){
		print("Line avoid enter\n\r");
		lineLeft = TriState::Unknown;
		lineFront = TriState::Unknown;
	}

	// Line front?
	if (mrm_ref_can->any(false, 0) && lineFront == TriState::Unknown && front() < WALL_DISTANCE)
			lineFront = TriState::Yes, print("Front");

	// Line right?
	if (mrm_ref_can->any(false, 1) && lineLeft == TriState::Unknown && right() < WALL_DISTANCE)
			lineLeft = TriState::Opposite, print("Right");

	// Line back?
	if (mrm_ref_can->any(false, 2) && lineFront == TriState::Unknown && back() < WALL_DISTANCE)
			lineFront = TriState::Opposite, print("Back");

	// Line left?
	if (mrm_ref_can->any(false, 3) && lineLeft == TriState::Unknown && left() < WALL_DISTANCE)
			lineLeft = TriState::Yes, print("Left");

	if (lineLeft != TriState::Unknown || lineFront != TriState::Unknown){ // A line, not a false alarm
		int8_t x = 0;
		int8_t y = 0;
		if (lineFront == TriState::Yes)
			y--;
		else if (lineFront == TriState::Opposite)
			y++;
		if (lineLeft == TriState::Yes)
			x++;
		else if (lineLeft == TriState::Opposite)
			x--;
		escapeDirection = atan2(x, y) / PI * 180;
		print("Line avoid, F:%i R:%i B:%i L:%i Esc dir: %i L:%i F:%i\n\r", mrm_ref_can->any(false, 0), mrm_ref_can->any(false, 1),
			mrm_ref_can->any(false, 2), mrm_ref_can->any(false, 3), (int)escapeDirection, lineLeft, lineFront);

		go(70, escapeDirection, pidRotation->calculate(heading() - headingToMaintain), 100);
		if (!lineAny()){
			print("Escaped\n\r");
			lineLeft = TriState::Unknown;
			lineFront = TriState::Unknown;
			actionSet("idl");
			//actionSet(actionBounce);
		}
	}
	else
		actionSet("idl"), print("False line");
		//actionSet(actionBounce), print("False line");
}

int8_t speedX;
int8_t speedY;

/** Custom test.
*/
void RobotSoccer::loop() {
	if (serialDataCount() > 10 ){
		print("Data: %s\n\r", uartRxCommandCumulative);
		serialDataClear();
	}
}

/** Generic actions, use them as templates
*/
void RobotSoccer::loop0() { speedY += 5, actionSet("lo4");} // Forward

void RobotSoccer::loop1() { speedY -= 5, actionSet("lo4");} // Backward

void RobotSoccer::loop2() { speedX -= 5, actionSet("lo4");} // Left

void RobotSoccer::loop3() { speedX += 5, actionSet("lo4");} // Right

void RobotSoccer::loop4() {
	const int MAX_SPEED = 30;
	//Limit speed
	if (speedX < -MAX_SPEED)
		speedX = -MAX_SPEED;
	if (speedX > MAX_SPEED)
		speedX = MAX_SPEED;
	if (speedY < -MAX_SPEED)
		speedY = -MAX_SPEED;
	if (speedY > MAX_SPEED)
		speedY = MAX_SPEED;
	float direction = atan2(speedX, speedY) / PI * 180; // Arcus tangens to get angle.
	float speed = sqrt(speedX * speedX + speedY * speedY); // Variable speed.
	// print("%i°, x: %i, y: %i, speed: %i\n\r", (int)direction, speedX, speedY, (int)speed);
	go(speed, direction, pidRotation->calculate(heading() - headingToMaintain), 100);
}

/** Starts robot
*/
void RobotSoccer::play() {
	if (motorGroup == NULL) {
		print("Define robot->motorGroupStar first.\n\r");
		return;
	}
	headingToMaintain = mrm_imu->heading();
	print("Yaw: %i\n\r", (int)headingToMaintain);
	actionSet("idl");
	//actionSet(actionBounce);
}

/** Right distance to wall
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/
uint16_t RobotSoccer::right(uint8_t sampleCount, uint8_t sigmaCount) {
	return mrm_lid_can_b2->distance(1, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}


/** Display fixed sign stored in sensor
@image - sign's number
*/
void RobotSoccer::sign(uint8_t number) {
	mrm_8x8a->bitmapDisplay(number);
}
