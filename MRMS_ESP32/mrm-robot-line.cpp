#include <mrm-8x8a.h>
#include <mrm-col-can.h>
#include <mrm-fet-can.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-lid-d.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-mot4x10.h>
#include "mrm-robot-line.h"
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>
#include <mrm-us-b.h>

/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotLine::RobotLine(char name[]) : Robot(name) {
	// MotorGroup class drives the motors.
	// 2nd, 4th, 6th, and 8th parameters are output connectors of the controller (number 0 - 3, meaning 1. - 4. connector). 
	// 2nd one must be connected to LB (Left-Back) motor, 4th to LF (Left-Front), 6th to RF (Right-Front), and 8th to RB (Right-Back). 
	// Therefore, You can connect motors freely, but have to adjust the parameters here. In this example output (connector) 0 is LB, etc.
	motorGroup = new MotorGroupDifferential(mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 3);
	motorGroup->delayMs = [this](uint16_t pauseMs){delayMs(pauseMs);}; //To avoid back-pointer

	distanceInterface = mrm_lid_can_b; // We will program using interface instead of implementation

	Mrm_8x8a::LEDSignText* signTest = new Mrm_8x8a::LEDSignText();

	// All the actions that sholuld be called from code will be defined here; the callable objects will be created.
	actions->insert({"eva", new ActionRobotLine(this, "Evacuation zone", 1, Board::BoardId::ID_ANY, NULL, &RobotLine::evacuationZone)});
	actions->insert({"lnf", new ActionRobotLine(this, "Line follow", 1, Board::BoardId::ID_ANY, NULL, &RobotLine::lineFollow)});
	actions->insert({"obs", new ActionRobotLine(this, "Obstacle avoid", 0, Board::BoardId::ID_ANY, NULL, &RobotLine::obstacleAvoid)});
	actions->insert({"rcj", new ActionRobotLine(this, "RCJ line", 1, Board::BoardId::ID_ANY, NULL, &RobotLine::rcjLine)});
	actions->insert({"str", new ActionRobotLine(this, "Stop motors", 1, Board::BoardId::ID_ANY, NULL, &RobotLine::stop)});
	actions->insert({"msh", new ActionRobotLine(this, "Motor short test", 1, Board::BoardId::ID_ANY, NULL, &RobotLine::motorShortTest)});
	actions->insert({"lo5", new ActionRobotLine(this, "loop5", 8, Board::BoardId::ID_ANY, signTest, &RobotLine::loop5)});
	actions->insert({"lo6", new ActionRobotLine(this, "loop6", 8, Board::BoardId::ID_ANY, signTest, &RobotLine::loop6)});
	actions->insert({"lo7", new ActionRobotLine(this, "loop7", 8, Board::BoardId::ID_ANY, signTest, &RobotLine::loop7)});
	actions->insert({"lo8", new ActionRobotLine(this, "loop8", 8, Board::BoardId::ID_ANY, signTest, &RobotLine::loop8)});
	actions->insert({"lo9", new ActionRobotLine(this, "loop9", 8, Board::BoardId::ID_ANY, signTest, &RobotLine::loop9)});

	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus.

#define CUSTOMIZE_SERVO 0
#if CUSTOMIZE_SERVO
	servoUpAdd = preferences->getInt("srvUpAdd"); // Servo up correction. Positive nubers lift servo.
	preferences->putInt("srvDown", 0); // Uncomment to correct (add to) servo-up positions. 
	if (servoUpAdd != 0)
		print("Servo-up add: %i\n\r", servoUpAdd);
#else
	servoUpAdd = 0;
#endif

	// Servo motors. Note that some pins are not appropriate for PWM (servo)
	mrm_servo->add(18, (char*)"ServoUp", 0, 300, 0.5, 2.5); // Data for mrm-rds5060-300
	mrm_servo->add(19, (char*)"ServoR", 0, 180, 0.5, 2.5); // Data for mrm-ps-1109hb
	mrm_servo->add(17, (char*)"ServoL", 0, 180, 0.5, 2.5); // Data for mrm-ps-1109hb

	// Set buttons' actions.
	// mrm_8x8a->actionSet(actionRCJLine, 0); // Button 1 starts RCJ Line.
	// mrm_8x8a->actionSet(actionEvacuationZone, 1); // Button 2 starts robot in evacution zone.
	mrm_8x8a->actionSet(actionFind("loo"), 2); // Button 3 starts user defined loop() function
	mrm_8x8a->actionSet(actionFind("sto"), 3); // Stop the robot

	// Put Your buttons' actions here.

	// Depending on your wiring, it may be necessary to spin some motors in the other direction. 
	mrm_mot4x3_6can->directionChange(mrm_mot4x3_6can->devices[0]); // Uncomment to change 1st wheel's rotation direction
	mrm_mot4x3_6can->directionChange(mrm_mot4x3_6can->devices[1]); // Uncomment to change 2nd wheel's rotation direction
	mrm_mot4x3_6can->directionChange(mrm_mot4x3_6can->devices[2]); // Uncomment to change 3rd wheel's rotation direction
	mrm_mot4x3_6can->directionChange(mrm_mot4x3_6can->devices[3]); // Uncomment to change 4th wheel's rotation direction

	// Digital switches connected to ESP32 pins
	// pinMode(25, INPUT_PULLDOWN);
	// pinMode(26, INPUT_PULLDOWN);
	pinMode(GRIPPER_SWITCH, INPUT_PULLDOWN); // Gripper's switch (27)
	// pinMode(32, INPUT_PULLDOWN);
	// pinMode(33, INPUT_PULLDOWN);

	// If uncommented, robot will immediately after power-on start executing loop()
	//actionSet(_actionLoop);

	armIdle();
}

/** Arm will go to ball-catch position.
*/
void RobotLine::armCatch() {
	mrm_servo->write(LIFT_SERVO_DOWN, 0); // Lower the arm. Parameter 0 defines servo, in this case lift-servo. LIFT_SERVO_DOWN is angle.
	mrm_servo->write(CATCH_SERVO_R_CATCH, 1); // Catch the ball. Parameter 1 - R catch-servo. CATCH_SERVO_CLOSE is angle.
	mrm_servo->write(CATCH_SERVO_L_CATCH, 2); // 
}

/** Arm will go to ball-catch ready position.
*/
void RobotLine::armCatchReady() {
	mrm_servo->write(LIFT_SERVO_DOWN + servoUpAdd, 0); // Lower the arm.
	mrm_servo->write(CATCH_SERVO_R_OPEN, 1); // 
	mrm_servo->write(CATCH_SERVO_L_OPEN, 2); // 
}

/** Arm will drop the ball
*/
void RobotLine::armDrop() {
	mrm_servo->write(LIFT_SERVO_UP + servoUpAdd, 0); // Lift the arm.
	mrm_servo->write(CATCH_SERVO_R_OPEN, 1); // 
	mrm_servo->write(CATCH_SERVO_L_OPEN, 2); // 
}

/** Arm in idle (closed - down) position
*/
void RobotLine::armIdle() {
	mrm_servo->write(LIFT_SERVO_IDLE + servoUpAdd, 0); // Lower the arm.
	mrm_servo->write(CATCH_SERVO_R_CLOSE, 1); // 
	mrm_servo->write(CATCH_SERVO_L_CLOSE, 2); // 
}

/** 
*/
void RobotLine::armUp() {
	mrm_servo->write(LIFT_SERVO_UP + servoUpAdd, 0); // Lift the arm.
	mrm_servo->write(CATCH_SERVO_L_CLOSE, 1); // 
	mrm_servo->write(CATCH_SERVO_R_CLOSE, 2); // 
}

/** Barrier interrupted?
* return interrupted or not
*/
bool RobotLine::barrier() {
	return analogRead(35) < barrierBrightest * 0.5; 
}

/** Stores bitmaps in mrm-led8x8a.
*/
void RobotLine::bitmapsSet() {
	mrm_8x8a->aliveWithOptionalScan(0, true); // Makes sure that mrm-8x8a is present and functioning. If not, issues a warning message.

	const std::vector<Mrm_8x8a::ledSign> selectedImages = {Mrm_8x8a::LED_CUSTOM, Mrm_8x8a::LED_EVACUATION_ZONE, Mrm_8x8a::LED_FULL_CROSSING_BOTH_MARKS,  
	Mrm_8x8a::LED_FULL_CROSSING_MARK_LEFT, Mrm_8x8a::LED_FULL_CROSSING_MARK_RIGHT, Mrm_8x8a::LED_FULL_CROSSING_NO_MARK,
	Mrm_8x8a::LED_HALF_CROSSING_MARK_LEFT, Mrm_8x8a::LED_HALF_CROSSING_MARK_RIGHT, Mrm_8x8a::LED_HALF_CROSSING_LEFT_NO_MARK,
	Mrm_8x8a:: LED_HALF_CROSSING_RIGHT_NO_MARK, Mrm_8x8a::LED_LINE_FULL, Mrm_8x8a::LED_LINE_FULL_BOTH_MARKS, Mrm_8x8a::LED_LINE_FULL_MARK_LEFT,
	Mrm_8x8a::LED_LINE_FULL_MARK_RIGHT, Mrm_8x8a::LED_LINE_INTERRUPTED,  Mrm_8x8a::LED_CURVE_LEFT, Mrm_8x8a::LED_CURVE_RIGHT, 
	Mrm_8x8a::LED_OBSTACLE, Mrm_8x8a::LED_OBSTACLE_AROUND_LEFT, Mrm_8x8a::LED_OBSTACLE_AROUND_RIGHT, Mrm_8x8a::LED_PAUSE, Mrm_8x8a::LED_PLAY,
	Mrm_8x8a::LED_T_CROSSING_BY_L,Mrm_8x8a::LED_T_CROSSING_BY_R, Mrm_8x8a::LED_WALL_AHEAD, Mrm_8x8a::LED_WALL_L, Mrm_8x8a::LED_WALL_R};
	mrm_8x8a->bitmapsSet(selectedImages);

}

/** Reads push button switch
@number - 0 to 3, push button's ordinal number
@return - true if pressed
*/
bool RobotLine::button(uint8_t number) {
	return mrm_8x8a->switchRead(number);
}

/** Line sensor - brightness of the surface
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@return - brightness as an analog value.
*/
uint16_t RobotLine::brightness(uint8_t transistorNumber) {
	return mrm_ref_can->reading(transistorNumber);
}

/** Go through a curve
*/
void RobotLine::curve() {
	bool left = line(LAST_TRANSISTOR); // Otherwise right
	display(left ? LED_CURVE_LEFT : LED_CURVE_RIGHT); 

	// If a marker found, rotate and do not continue executing the rest of the function.
	if (markers() && (line(3) || line(4) || (LAST_TRANSISTOR == 8 && line(5)))) // Center line needed, too.
		return;

	// Go a little ahead to check if it is T-crossing approached by side. Also, take a better position for turning.
	go(TOP_SPEED, TOP_SPEED);
	uint32_t startMs = millis();
	while (millis() - startMs < AHEAD_IN_CROSSING * 0.65) {
		if (line(left ? 0 : LAST_TRANSISTOR))// Opposite sensor - full crossing
			return; // Exit curve handling as crossing is found.
		noLoopWithoutThis();
	}

	// L curve, not T approached by side. Because, if T approached by side, center transistors would still sense black.
	if (!lineAny(2, 5)) {
		// Rotate
		stop();
		print("CURVE\n\r"); 
		delayMs(2000);
		go(left ? -TOP_SPEED : TOP_SPEED, left ? TOP_SPEED : -TOP_SPEED); // Start rotating.
		startMs = millis();
		while (millis() - startMs < 1300) { // Rotate until center line found, unless timeout expires.
			if (line(LAST_TRANSISTOR / 2)) { // Line in center?
				// delayMs(150); // Align better
				return;
			}
			noLoopWithoutThis();
		}
	}
	// T-crossing approached by side, go ahead.
	else {
		stop();
		print("HALF CROSSING\n\r");
		delayMs(2000);
	}
}

/** Dark surface?
* return dark or not
*/
bool RobotLine::dark() {
	return analogRead(36) < 2000; // 2000 is an example. Test to find Your best value
}

/** Display 8x8 image
@image - image's number
*/
void RobotLine::display(uint8_t image) {
	mrm_8x8a->bitmapCustomStoredDisplay(image);
}

/** Display 8x8 text
@text - text
*/
void RobotLine::display(char* text) {
	mrm_8x8a->text(text);
}


/** Enter evacuation-zone algorithm
*/
void RobotLine::evacuationZone() {
	// if (setup()) {
	// 	// Signal evacuation zone
	// 	display(LED_EVACUATION_ZONE);
	// 	stop();
	// 	armClose();// armCatchReady();
	// 	delayMs(300); // Wait for arm lowering.
	// 	barrierBrightest = analogRead(35); // Calibrate barrier according to actual external illumination
	// }

	// wallFollow();

	// // Ball found
	// if (barrier()) {
	// 	stop();
	// 	armCatch();
	// 	delayMs(1000);
	// 	armUp();
	// 	//go(50, 50);
	// 	delayMs(1000);
	// 	//stop();
	// 	armDrop();
	// 	delay(1000);
	// 	armClose();
	// 	end();
	// }
}

/** Front distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - distance in mm
*/
uint16_t RobotLine::front(uint8_t sampleCount, uint8_t sigmaCount) {
	return distanceInterface->distance(1, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Front side - left distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - distance in mm
*/
uint16_t RobotLine::frontLeft(uint8_t sampleCount, uint8_t sigmaCount) {
	return distanceInterface->distance(0, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Front side - left distance in cm, using ultrasonic sensor.
@return - distance in cm
*/
uint16_t RobotLine::frontLeftUS() {
	return mrm_us_b->reading(0); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Front side - right distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - distance in mm
*/
uint16_t RobotLine::frontRight(uint8_t sampleCount, uint8_t sigmaCount) {
	return distanceInterface->distance(2, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Start motors
@param leftSpeed, in range -127 to 127
@param right Speed, in range -127 to 127
@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/
void RobotLine::go(int16_t leftSpeed, int16_t rightSpeed) {
	motorGroup->go(leftSpeed, rightSpeed);
}

/** Test - go straight ahead using a defined speed.
*/
void RobotLine::goAhead() {
	const uint8_t speed = 40;
	go(speed, speed);
	end(); // This command will cancel actions and the robot will return in the default idle loop, after displaying menu.
}

/** Color sensor's hue
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - Hue
*/
uint8_t RobotLine::hue(uint8_t deviceNumber) {
	return mrm_col_can->hue(&mrm_col_can->devices[deviceNumber]);
}

/** Set color sensor's illumination intensity
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param current - 0 - 3
*/
void RobotLine::illumination(uint8_t current, uint8_t deviceNumber) {
	return mrm_col_can->illumination(&mrm_col_can->devices[deviceNumber], current);
}

/** Left side - rear sensor distance.
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/
uint16_t RobotLine::leftBack(uint8_t sampleCount, uint8_t sigmaCount) {
	return distanceInterface->distance(4, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Left side - front sensor distance.
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/
uint16_t RobotLine::leftFront(uint8_t sampleCount, uint8_t sigmaCount) {
	return distanceInterface->distance(0, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Line found?
@return - true if any sensor detects black.
@param firstTransistor - start checking from this transistor.
@param lastTransistor - do not check after this one.
*/
bool RobotLine::lineAny(uint8_t firstTransistor, uint8_t lastTransistor) {
	return mrm_ref_can->any(true, 0, firstTransistor, lastTransistor);
}

/** Line sensor
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@return - true if black line found
*/
bool RobotLine::line(uint8_t transistorNumber) {
	return mrm_ref_can->dark(transistorNumber);
}

/** Center of measurements, like center of the line
@return - -50 - 50. If 0, center of the robot is on the line. Depending on sensor, this value is roughly number of milimeter the line is left or right.
*/
float RobotLine::lineCenter() {
	return (mrm_ref_can->center() - 5000) / 80.0;
}

/** Follow a RCJ line.
*/
void RobotLine::lineFollow() {
	static uint32_t ms = 0;
	if (setup()){
		ms = millis();
		armUp();
	}

	if (lineAny())
		ms = millis();

	if (line(8))
		go(-90, 90);
	else if (line(0))
		go(90, -90);
	else if (line(7))
		go(0, 100);
	else if (line(1))
		go(100, 0);
	else if (line(6))
		go(30, 80);
	else if (line(2))
		go(80, 30);
	else if (line(5))
		go(40, 70);
	else if (line(3))
		go(70, 40);
	else if (line(4))
		go(70, 70);
	else
		if (millis() - ms > 100)
			stop();
		else
			go(70, 70);
	// static uint32_t lastLineFoundMs = millis(); // Used to measure gap in line.

	// // Obstacle?
	// #define OBSTACLE 0
	// #if OBSTACLE
	// if (frontLeft() < 50 && frontLeft() != 0) { // Front sensor (1).
	// 	//print("Obstacle: %i\n\r", frontLeft()); // For debugging.
	// 	stop(); // Stop.
	// 	delayMs(50); // Wait a little before another measurement to be sure that the lidar has enough time to send a fresh measurement.
	// 	if (frontLeft() < 50) { // Check one more time. If detected again, this will be an obstacle.
	// 		actionSet(actionObstacleAvoid); // Sets the new action.
	// 		actionObstacleAvoid->leftOfObstacle = (leftFront() > leftFront()); // Decides if go left or right, depending on space available.
	// 		return;
	// 	}
	// }
	// #endif

	// // Line found?
	// if (lineAny(true)) {
	// 	#define CROSSINGS 0
	// 	#if CROSSINGS
	// 	// Both edge sensors and middle sensor? Crossing. Check markers.
	// 	if (line(0) && line(LAST_TRANSISTOR / 2) && line(LAST_TRANSISTOR)) {
	// 		// Green markers?
	// 		if (!markers()) {// No mark detected. Go straight ahead.
	// 			display(LED_FULL_CROSSING_NO_MARK); // Show sign.
	// 			//delayMs(1000);
	// 			go(TOP_SPEED, TOP_SPEED);
	// 			delayMs(AHEAD_IN_CROSSING); // Make sure to cross the crossing.
	// 		}
	// 	}
	// 	// Edge sensor? If so, sharp bend.
	// 	else 
	// 	#endif
	// 	if (line(0) || line(LAST_TRANSISTOR))
	// 		curve();
	// 	else {
	// 		// Follow line
	// 		float lineCenterNow = lineCenter(); // mrm-ref-can returns center of line, between -50 and 50 (mm).
	// 		// Calculate slower motor's speed. The other one will run at top speed.
	// 		go(lineCenterNow < 0 ? TOP_SPEED : TOP_SPEED - lineCenterNow * 8, lineCenterNow < 0 ? TOP_SPEED + lineCenterNow * 8 : TOP_SPEED);
	// 		display(LED_LINE_FULL);
	// 	}
	// 	lastLineFoundMs = millis(); // Mark last time line detected.
	// }
	// // No line found for a long time -> evacuation area.
	// else if (millis() - lastLineFoundMs > BIGGEST_GAP_IN_LINE_MS)
	// 	actionSet(actionEvacuationZone);
	// // No line found for s short time -> gap in line, continue straight ahead.
	// else {
	// 	go(TOP_SPEED, TOP_SPEED);
	// 	display(LED_LINE_INTERRUPTED);
	// }
}

#include <SPI.h>

/** Custom test. The function will be called many times during the test, till You issue "x" menu command.
*/
void RobotLine::loop() {
	uint32_t ms = millis();
	int cnt = 0;
	int empty = 0;
	while (millis() < ms + 5000){
		uint8_t data[] = {0xFF, 0, 0, 0, 0, 0, 0, 0};
		mrm_can_bus->messageSend(0x200, 1, data);
		while (true){
			_msg = mrm_can_bus->messageReceive();
			if (_msg != nullptr &&_msg->data[0] == 0xFF)
				break;
			else
				empty++;
		}
		cnt++;
	}
	print("End %i, empty %i/n/r", cnt, empty);
	end();
}

/** Generic actions, use them as templates
*/
void RobotLine::loop0() { armIdle(); end();}
void RobotLine::loop1() { armCatchReady(); end(); }
void RobotLine::loop2() { armCatch(); end(); }
void RobotLine::loop3() { armUp(); end(); }
void RobotLine::loop4() { armDrop(); end(); }
void RobotLine::loop5() {		
	if (line(8))
		go(-90, 90);
	else if (line(0))
		go(90, -90);
	else if (line(5))
		go(10, 90);
	else if (line(3))
		go(90, 10);
	else
		go(60, 60); 
	}
void RobotLine::loop6() { 
	if (line(5))
			go(10, 80);
		else if (line(3))
			go(80, 10);
		else
			go(60, 60);
}
void RobotLine::loop7() { 
	if (mrm_lid_can_b2->distance(1) > 100 && mrm_lid_can_b2->distance(6) > 100)
		go(50, 50);
	else if (mrm_lid_can_b2->distance(0) > 100)
		go (50, -50);
	else if (mrm_lid_can_b2->distance(7) > 100)
		go (-50, 50);
	else
		go (-50, -50);
}
void RobotLine::loop8() { }
void RobotLine::loop9() { }

/** Generic menu
*/
void RobotLine::loopMenu() {
	menuLevel = 8;
	end();
}


/** Check markers and turn if any found
@return - true if marker found, false otherwise
*/
bool RobotLine::markers() {
	stop(); // Stop and wait to be sure the color sensor read new values.
	delayMs(200);
	//bool greenLeft = mrm_col_can->patternRecognizedBy6Colors(0) == 2; // This function returns laerned pattern's number, for sensor 0 (left). Learned pattern 2 is green.
	//bool greenRight = mrm_col_can->patternRecognizedBy6Colors(1) == 2; // This function returns laerned pattern's number, for sensor 0 (left). Learned pattern 2 is green.
	bool greenLeft = mrm_col_can->value(mrm_col_can->devices[0]) < 70 && mrm_col_can->hue(&mrm_col_can->devices[0]) > 60 && mrm_col_can->hue(&mrm_col_can->devices[0]) < 70;
	bool greenRight = mrm_col_can->value(mrm_col_can->devices[1]) < 70 && mrm_col_can->hue(&mrm_col_can->devices[1]) > 60 && mrm_col_can->hue(&mrm_col_can->devices[1]) < 70;

	bool fullLineL = !lineAny(4, 7);
	bool fullLineR = !lineAny(0, 3);

	if (!fullLineL)
		greenLeft = false;
	if (!fullLineR)
		greenRight = false;

	bool found = true;
	if (greenLeft && greenRight) { // Both markers detected. Turn backward.
		display(LED_FULL_CROSSING_BOTH_MARKS); // Show sign.
		print("MARK ");
		surfacePrint(false, 3000);
		print(" MARK\n\r");
		turn(180); // Turn by 180�.
	}
	else if (greenLeft && !greenRight) { // Only left marker. Turn left.
		display(LED_FULL_CROSSING_MARK_LEFT); // Show sign.
		print("MARK ");
		surfacePrint(true, 3000);
		turn(-90); // Turn by 90� left.
	}
	else if (!greenLeft && greenRight) { //Only right marker. Turn right.
		display(LED_FULL_CROSSING_MARK_RIGHT); // Show sign.
		surfacePrint(false, 3000);
		print(" MARK\n\r");
		turn(90); // Turn by 90� right.
	}
	else {
		found = false;
		surfacePrint(true, 3000);
	}
	return found;
}


/** Test motors
 */
void RobotLine::motorShortTest(){
	go(60, 60);
	delayMs(2000);
	go(-60, -60);
	delayMs(2000);
}

/** Avoid an obstacle on line.
*/
void RobotLine::obstacleAvoid() {
	stop();
	delayMs(2000);
	print("Prepreka");
	go(50, -50);
	uint32_t lastLineMs = millis();
	while (front() < 200 || frontLeft() < 200 || millis() - lastLineMs < 200){
		noLoopWithoutThis();
		if (lineAny())
			lastLineMs = millis();
	}
	stop();
	print("Okrenut");
	delayMs(2000);
	while(!lineAny()){
		if (front() < 200 || frontLeft() < 200)
			go(50, 50);
		else
			go(20, 80);
		noLoopWithoutThis();
	}
	stop();
	print("Naišao");
	delayMs(2000);
	go(70, 10);
	delayMs(500);
	print("Linija");
	// // Static variables - their value will be retained between this function's calls, just like with global variables, but they have local scope.
	// static uint8_t part = 0;
	// static uint32_t startMs = 0;

	// // This function will be executed many times during obstacle avoiding action, but only the first time the action (ActionObstacleAvoid) is executed will
	// // "setup()" be true, thus executing the next instruction (showing a sign).
	// if (setup())
	// 	display(LED_OBSTACLE); // Show a sign.

	// /* Obstacle evasive maneuver is an action big enough to justify its own ActionBase derived object, ActionObstacleAvoid, which is being executed right now. 
	// The maneuver itself consists of	different phases. Should all of them form ActionObstacleAvoid derived classes? Yes, that would be a good solution. However, 
	// here we will show a solution that is not object oriented: a "switch" statement. During many runs of this function, static variable "part" will be slowly 
	// changing its value from 0 up, marking different phases. The logic follows.
	// */
	// switch (part) {
	// case 0: // Turn in place in front of obstacle.
	// 	if (frontLeft() < 60 || lineAny()) // If obstacle is still in front or robot still on line - continue rotating.
	// 		// Read the member variable of the current action (of class ActionObstacleAvoid) to determine direction of rotation.
	// 		go(actionObstacleAvoid->leftOfObstacle ? -100 : 100, actionObstacleAvoid->leftOfObstacle ? 100 : -100);
	// 	else { // No more obastacle or line.
	// 		part = 1; // Advance to next phase.
	// 		startMs = millis(); // Mark phase's beginning.
	// 	}
	// 	break;
	// case 1: // Continue turning even more.
	// 	if (millis() - startMs > 50) { // A fixed number of milliseconds (50) is not the best solution, but it is here for demonstration purposes.
	// 		part = 2; // Rotation over, advance to next phase.
	// 		display(actionObstacleAvoid->leftOfObstacle ? LED_OBSTACLE_AROUND_LEFT : LED_OBSTACLE_AROUND_RIGHT); // Show sign.
	// 	}
	// 	break;
	// case 2: // Go around obstacle
	// 	// One of 2 middle sensors found a line?
	// 	if (line(4) || line(5)) { // Yes, the line found again.
	// 		go(actionObstacleAvoid->leftOfObstacle ? -100 : 100, actionObstacleAvoid->leftOfObstacle ? 100 : -100); // Start aligning with the line.
	// 		part = 3; // Advance to next phase.
	// 	}
	// 	// No line found, continue going around the obstacle.
	// 	else
	// 		// Here the fixed motors' speed are used - and that is not good. You should improve this action by feeding some sensors' output back into this motor-driving loop.
	// 		go(actionObstacleAvoid->leftOfObstacle ? 95 : 30, actionObstacleAvoid->leftOfObstacle ? 30 : 95);
	// 	break;
	// case 3: // Align with the found line. To do that, continue rotating till an near-edge sensor finds the line.
	// 	if (line(actionObstacleAvoid->leftOfObstacle ? 1 : 7))
	// 		part = 4; // Advance to next phase.
	// 	break;
	// case 4: // Follow line again.
	// default:
	// 	part = 0; // This is the last phase so reset "part" variable for the next obstacle.
	// 	actionSet(actionLineFollow); // As the robot is now on line, set the current action to line following.
	// 	actionPreprocessingEnd();
	// 	break;
	// }
}

/** Choose a pattern closest to the current 6 colors
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param includeValue - if true, HSV compared. If not, HS.
@raturn - patternNumber
*/
uint8_t RobotLine::patternColors(uint8_t deviceNumber) {
	return mrm_col_can->patternRecognizedBy6Colors(mrm_col_can->devices[deviceNumber]);
}


/** Starts the RCJ Line run after this action selected.
*/
void RobotLine::rcjLine() {
	//lineFollow();
	if (false)
		if (front() < 100)
			obstacleAvoid();
	mrm_8x8a->rotationSet(Mrm_8x8a::LED_8X8_BY_90_DEGREES); // Rotate the mrm-8x8a by 90� so that it can be read properly when standing behind the robot.
	bitmapsSet(); // Upload all the predefined bitmaps into the mrm-8x8a.
	display(LED_PLAY); // Show "play" sign.
	mrm_col_can->illumination(nullptr, 1); // Turn mrm-col-can's surface illumination on.
	//armClose(); // Arm will go to its idle (up) position.
	actionSet("lnf"); // The next action is line following.
}

/** Front distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - distance in mm
*/
uint16_t RobotLine::rightBack(uint8_t sampleCount, uint8_t sigmaCount) {
	return distanceInterface->distance(3); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Front distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - distance in mm
*/
uint16_t RobotLine::rightFront(uint8_t sampleCount, uint8_t sigmaCount) {
	return distanceInterface->distance(2); // Correct all sensors so that they return the same value for the same physical distance.
}


/** Color sensor's saturation
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - saturation.
*/
uint8_t RobotLine::saturation(uint8_t deviceNumber) {
	return mrm_col_can->saturation(&mrm_col_can->devices[deviceNumber]);
}

/** Move servo
@param degrees - Servo's target angle, 0 - 180°, or 0 - 360°, depending on model, counting clockwise
@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
*/
void RobotLine::servo(uint16_t degrees, uint8_t servoNumber){
	mrm_servo->write(degrees, servoNumber);
}

/** Display fixed sign stored in sensor
@image - sign's number
*/
void RobotLine::sign(uint8_t number) {
	mrm_8x8a->bitmapDisplay(number);
}

/** Stop the robot
*/
void RobotLine::stop() {
	motorGroup->stop();
	end();
}

/** Store 8x8 image to 8x8 LED's internal memory
@red - red pixels
@green - green pixels
@image - image's number
*/
void RobotLine::store(uint8_t red[], uint8_t green[], uint8_t image) {
	mrm_8x8a->bitmapCustomStore(red, green, image);
}


/** Prints line and color sensors. Used for debugging.
@param newLine - new line
@param delayMsAfterPrint - delay after print
*/
void RobotLine::surfacePrint(bool newLine, uint16_t delayMsAfterPrint) {
	print("%i/%i/%i ", mrm_col_can->hue(&mrm_col_can->devices[0]), mrm_col_can->saturation(&mrm_col_can->devices[0]), mrm_col_can->value(mrm_col_can->devices[0]));
	for (int8_t i = LAST_TRANSISTOR; i >= 0; i--)
		print("%i", line(i));
	print(" %i/%i/%i ", mrm_col_can->hue(&mrm_col_can->devices[1]), mrm_col_can->saturation(&mrm_col_can->devices[1]), mrm_col_can->value(mrm_col_can->devices[1]));
	if (newLine)
		print("\n\r");
	if (delayMsAfterPrint != 0)
		delayMs(delayMsAfterPrint);
}

/** Turns the robot clockwise using compass.
@param byDegreesClockwise - turn by defined number of degrees.
*/
void RobotLine::turn(int16_t byDegreesClockwise) {
	go(TOP_SPEED, TOP_SPEED); // Go ahead at top speed.
	delayMs(200); // Wait till the robot is in the position in which the line will hit the middle of robot.
	int16_t endAngle = mrm_imu->heading() + byDegreesClockwise; // Determine end angle.
	// If the result is outside the 0 -360�, correct.
	if (endAngle > 360)
		endAngle -= 360;
	else if (endAngle < 0)
		endAngle += 360;
	int8_t speed = byDegreesClockwise > 0 ? TOP_SPEED : -TOP_SPEED;
	go(speed, -speed); // Start turning
	// This is an example of a local loop and this is not a good solution, but is here as an example.
	while (abs(mrm_imu->heading() - endAngle) > 5) // As long as the error is 5� or more, continue rotating.
		noLoopWithoutThis(); // If a local loop is necessary, You will have to be invoking this function all the time.
	stop(); // Rotation over, stop.
}

/** Color sensor's value
@param deviceNumber - device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - value
*/
uint8_t RobotLine::value(uint8_t deviceNumber) {
	return mrm_col_can->value(mrm_col_can->devices[deviceNumber]);
}

#if LIDAR_COUNT == 6
/** Follows a wall.
*/
void RobotLine::wallFollow() {
	const uint16_t FOLLOW_WALL_MM = 90;
	const float STRENGTH_ROTATION = 3; // The bigger this factor, the more will the robot try to decrease rotational error.
	const float STRENGTH_LATERAL = 3; // The bigger this factor, the more will the robot try to decrease lateral error (distance to the wall).
	const uint16_t WALL_AHEAD_MM = 160; // Turn if distance is smaller.

	bool lastFollowedLeft = false; // Used to determine where to turn if wall ahead.

	// Follow wall
	if (wallLeft() || wallRight()){
		lastFollowedLeft = (wallLeft() && !wallRight()) || (wallLeft() && wallRight() && leftFront() < rightFront()); // Choose closer wall
		display(lastFollowedLeft ? LED_WALL_L : LED_WALL_R);

		// Rotational and lateral errors
		float errorTooMuchCW = lastFollowedLeft ? leftFront() - leftBack() : rightBack() - rightFront();
		float errorTooMuchLeft = lastFollowedLeft ? FOLLOW_WALL_MM - (leftFront() + leftBack()) / 2.0 : (rightFront() + rightBack()) / 2.0 - FOLLOW_WALL_MM;

		// Combined error
		float increaseLeftBy = -errorTooMuchCW * STRENGTH_ROTATION + errorTooMuchLeft * STRENGTH_LATERAL;

		// Drive one side at top speed.
		go(increaseLeftBy < 0 ? TOP_SPEED + increaseLeftBy : TOP_SPEED , increaseLeftBy < 0 ? TOP_SPEED : TOP_SPEED - increaseLeftBy);

		static uint32_t lastMs = 0;
		if (millis() - lastMs > 100) {
			print("Too L: %3i + Too CW: %3i = incr: %3i\n\r", (int)errorTooMuchLeft, (int)errorTooMuchCW, (int)increaseLeftBy);
			lastMs = millis();
		}
	}
	// No wall in sight, just go straight ahead.
	else
		go(TOP_SPEED, TOP_SPEED);


	// Wall ahead
	if (frontLeft() < WALL_AHEAD_MM && frontRight() < WALL_AHEAD_MM) {
		print("First: %i %i\n\r", frontLeft(), frontRight());
		stop();
		delayMs(50);
		if (frontLeft() < WALL_AHEAD_MM && frontRight() < WALL_AHEAD_MM) { // Measure one more time to be sure.
			print("Second: %i %i\n\r", frontLeft(), frontLeft());
			display(LED_WALL_AHEAD);

			// Turn away from the side wall.
			turn(lastFollowedLeft ? 90 : -90);
		}
	}
}
#else
/** Follows a wall.
*/
void RobotLine::wallFollow() {
	const uint16_t MAXIMUM_WALL_MM = 300; // If distance bigger than this value, do not follow wall.
	const uint16_t FOLLOW_WALL_MM = 90;
	const float FOLLOW_WALL_STRENGTH = 5; // The bigger this factor, the more will the robot turn.
	const uint16_t WALL_AHEAD_MM = 140;

	bool lastFollowedLeft; // Used to determine where to turn if wall ahead.
	uint16_t left = frontLeft();
	uint16_t right = frontLeft();

	// No wall in sight, just go straight ahead.
	if (left > MAXIMUM_WALL_MM && right > MAXIMUM_WALL_MM)
		go(TOP_SPEED, TOP_SPEED);
	// Follow wall.
	else {
		lastFollowedLeft = left < right; // Choose closer wall
		display(lastFollowedLeft ? LED_WALL_L : LED_WALL_R);
		int16_t error = ((lastFollowedLeft ? left : right) - FOLLOW_WALL_MM) * FOLLOW_WALL_STRENGTH; // Negative error - robot too much to the left, it should go right
		if (!lastFollowedLeft)
			error = -error;
		// Drive one side at top speed.
		go(error < 0 ? TOP_SPEED : TOP_SPEED - error, error < 0 ? TOP_SPEED + error : TOP_SPEED);
	}

	// Wall ahead
	if (frontLeft() < WALL_AHEAD_MM) {
		print("First: %i\n\r", frontLeft());
		stop();
		delayMs(100);
		if (frontLeft() < WALL_AHEAD_MM) { // Measure one more time to be sure.
			print("Second: %i\n\r", frontLeft());
			display(LED_WALL_AHEAD);
			delayMs(6000);
			// Turn away from the side wall.
			turn(lastFollowedLeft ? 90 : -90);
			print("Third: %i\n\r", frontLeft());
			delayMs(100);
			delayMs(3000);
			print("Fourth: %i\n\r", frontLeft());
		}
	}
}
#endif

/** Wall left?
@return - true if wall
*/
bool RobotLine::wallLeft() {
	return leftFront() < MAXIMUM_WALL_MM && leftBack() < MAXIMUM_WALL_MM && abs(leftFront() - leftBack()) < WALL_DIFFERENCE_ALLOWED_MM;
}

/** Wall right?
@return - true if wall
*/
bool RobotLine::wallRight() {
	return rightFront() < MAXIMUM_WALL_MM && rightBack() < MAXIMUM_WALL_MM && abs(rightFront() - rightBack()) < WALL_DIFFERENCE_ALLOWED_MM;
}