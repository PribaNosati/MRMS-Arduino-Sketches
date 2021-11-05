#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

// Change these values to get optimal robot's behaviour.

// CATCH_SERVO L and R drive jaws that catch ball.
#define CATCH_SERVO_L_CATCH 30 // Ball caught, left servo. Smaller number - more tightly closed.
#define CATCH_SERVO_L_CLOSE 0 // Closed (idle) position, no ball, left servo. Smaller number - more tightly closed.
#define CATCH_SERVO_L_OPEN 90 // Open position, ready to catch a ball, left servo.
#define CATCH_SERVO_R_CATCH 70 // Ball caught, right servo. Bigger number - more tightly closed.
#define CATCH_SERVO_R_CLOSE 90 // Closed (idle) position, no ball, right servo. Bigger number - more tightly closed.
#define CATCH_SERVO_R_OPEN 0 // Open position, ready to catch a ball, right servo. Bigger number - more tightly closed.

// LIFT_SERVO lifts catch the mechanism.
#define LIFT_SERVO_DOWN 130 // Lowest position, catching a ball. Increase number to lift higher.
#define LIFT_SERVO_UP 230 // Top (idle) position. Increase number to lift higher.

#define GRIPPER_SWITCH 27 // Gripper's switch for ball detection

#define LIDAR_COUNT 6 // 3 or 6, depending on model. If only 3 lidars built in, 6-mode cannot be used.

#define WALL_DIFFERENCE_ALLOWED_MM 50 // If difference bigger, no wall.
#define MAXIMUM_WALL_MM 300 // If distance bigger than this value, do not follow wall.

// mrm-8x8a display bitmaps.
enum ledSign {LED_CUSTOM, LED_EVACUATION_ZONE, LED_FULL_CROSSING_BOTH_MARKS, LED_FULL_CROSSING_MARK_LEFT, LED_FULL_CROSSING_MARK_RIGHT, LED_FULL_CROSSING_NO_MARK,
	LED_HALF_CROSSING_MARK_LEFT, LED_HALF_CROSSING_MARK_RIGHT, LED_HALF_CROSSING_LEFT_NO_MARK, LED_HALF_CROSSING_RIGHT_NO_MARK,
	LED_LINE_FULL, LED_LINE_FULL_BOTH_MARKS, LED_LINE_FULL_MARK_LEFT, LED_LINE_FULL_MARK_RIGHT, LED_LINE_INTERRUPTED, LED_CURVE_LEFT, 
	LED_CURVE_RIGHT, LED_OBSTACLE, LED_OBSTACLE_AROUND_LEFT, LED_OBSTACLE_AROUND_RIGHT, LED_PAUSE, LED_PLAY, LED_T_CROSSING_BY_L, 
	LED_T_CROSSING_BY_R, LED_WALL_AHEAD, LED_WALL_L, LED_WALL_R};

/* All the Action-classes have to be forward declared here (before RobotLine) as RobotLine declaration uses them. The other option would be
not to declare them here, but in that case Action-objects in RobotLine will have to be declared as ActionBase class, forcing downcast later in code, if
derived functions are used.*/
class ActionEvacuationZone;
class ActionObstacleAvoid;
class ActionLineFollow;
class ActionRCJLine;
class ActionMotorShortTest;

class ActionLoop0;
class ActionLoop1;
class ActionLoop2;
class ActionLoop3;
class ActionLoop4;
class ActionLoop5;
class ActionLoop6;
class ActionLoop7;
class ActionLoop8;
class ActionLoop9;
class ActionLoopMenu;

/** Robot for RCJ Rescue Line, a class derived from the base Robot class.
*/
class RobotLine : public Robot {
	uint16_t BIGGEST_GAP_IN_LINE_MS = 2500;
	// Changing this parameter will cause major behaviour change. Limit value: 127.
	const uint8_t TOP_SPEED = 90; // 7.4 V 80	11.1 V 60
	const uint16_t AHEAD_IN_CROSSING = 210; // 7.4V : 300
	const uint8_t LAST_TRANSISTOR = 7; // mrm-ref-can: 8, mrm-ref-can8: 7

	// Actions' declarations
	ActionEvacuationZone* actionEvacuationZone;
	ActionObstacleAvoid* actionObstacleAvoid;
	ActionLineFollow* actionLineFollow;
	ActionRCJLine* actionRCJLine;
	ActionBase* actionWallFollow;
	ActionStop* actionStop;
	ActionMotorShortTest* actionMotorShortTest;

	// Generic actions
	ActionLoop0* actionLoop0;
	ActionLoop1* actionLoop1;
	ActionLoop2* actionLoop2;
	ActionLoop3* actionLoop3;
	ActionLoop4* actionLoop4;
	ActionLoop5* actionLoop5;
	ActionLoop6* actionLoop6;
	ActionLoop7* actionLoop7;
	ActionLoop8* actionLoop8;
	ActionLoop9* actionLoop9;
	ActionLoopMenu* actionLoopMenu;

	uint16_t barrierBrightest = 2000; // Default value, it should be calibrated.

	MotorGroupDifferential* motorGroup = NULL; // Class that conveys commands to motors.

public:
	/** Constructor
	@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
	*/
	RobotLine(char name[] = (char*)"RCJ Line"); // Maximum 15 characters

	/** Arm will go to ball-catch position.
	*/
	void armCatch();

	/** Arm will go to ball-catch ready position.
	*/
	void armCatchReady();

	void armClose();

	/** Arm will drop the ball.
	*/
	void armDrop();

	/** Arm will lift the caught ball in the position where will be ready to drop it.
	*/
	void armUp();

	/** Barrier interrupted?
	* return interrupted or not
	*/
	bool barrier();

	/** Stores bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	/** Color sensor's blue
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - intensity of blue
	 */
	uint16_t blue(uint8_t deviceNumber = 0){return mrm_col_can->colorGreen(deviceNumber);}

	/** Line sensor - brightness of the surface
	@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
	@return - brightness as an analog value.
	*/
	uint16_t brightness(uint8_t transistorNumber);

	/** Reads push button switch
	@number - 0 to 3, push button's ordinal number
	@return - true if pressed
	*/
	bool button(uint8_t number);

	/** Go through a curve
	*/
	void curve();

	/** Dark surface?
	* return dark or not
	*/
	bool dark();

	/** Display 8x8 image
	@image - image's number
	*/
	void display(uint8_t image);

	/** Display 8x8 text
	@image - image's number
	*/
	void display(char* text);

	/** Enter evacuation-zone algorithm.
	*/
	void evacuationZone();

	/** Front distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - distance in mm
	*/
	uint16_t front(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Front side - left distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - distance in mm
	*/
	uint16_t frontLeft(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Front side - right distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - distance in mm
	*/
	uint16_t frontRight(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Start motors
	@param leftSpeed, in range -127 to 127
	@param right Speed, in range -127 to 127
	@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
	*/
	void go(int16_t leftSpeed, int16_t rightSpeed);

	/** Left side - rear sensor distance.
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - in mm
	*/
	uint16_t leftBack(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Left side - front sensor distance.
	 @param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - in mm
	*/
	uint16_t leftFront(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Line found?
	@return - true if any sensor detects black.
	@param firstTransistor - start checking from this transistor.
	@param lastTransistor - do not check after this one.
	*/
	bool lineAny(uint8_t fistTransistor = 0, uint8_t lastTransistor = 0xFF);

	/** Line sensor
	@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
	@return - true if black line found
	*/
	bool line(uint8_t transistorNumber);

	/** Center of measurements, like center of the line
	@return - -50 - 50. If 0, center of the robot is on the line. Depending on sensor, this value is roughly number of milimeter the line is left or right.
	*/
	float lineCenter();

	/** Generic actions, use them as templates
	*/
	void loop0();
	void loop1();
	void loop2();
	void loop3();
	void loop4();
	void loop5();
	void loop6();
	void loop7();
	void loop8();
	void loop9();

	/** Generic menu
	*/
	void loopMenu();

	/** Test - go straight ahead using a defined speed.
	*/
	void goAhead();

	/** Color sensor's green
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - intensity of green
	 */
	uint16_t green(uint8_t deviceNumber = 0){return mrm_col_can->colorGreen(deviceNumber);}

	/**Compass
	@return - North is 0�, clockwise are positive angles, values 0 - 360.
	*/
	float heading();

	/** Color sensor's hue
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - Hue
	*/
	uint8_t hue(uint8_t deviceNumber);

	/** Set color sensor's illumination intensity
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	@param current - 0 - 3
	*/
	void illumination(uint8_t current, uint8_t deviceNumber = 0);

	/** Follow a RCJ line.
	*/
	void lineFollow();

	/** Custom test
	*/
	void loop();

	/** Check markers and turn if any found
	@return - true if marker found, false otherwise
	*/
	bool markers();

	/** Test motors
	*/
	void motorShortTest();

	/** Avoid an obstacle on line.
	*/
	void obstacleAvoid();

	/** Color sensor's orange
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - intensity of orange
	 */
	uint16_t orange(uint8_t deviceNumber = 0){return mrm_col_can->colorOrange(deviceNumber);}

	/** Choose a pattern closest to the current 6 colors
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param includeValue - if true, HSV compared. If not, HS.
	@raturn - patternNumber
	*/
	uint8_t patternColors(uint8_t deviceNumber = 0);

	/**Pitch
	@return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0�.
	*/
	float pitch();

	/** Starts the RCJ Line run after this action selected.
	*/
	void rcjLine();

	/** Color sensor's red
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - intensity of red
	 */
	uint16_t red(uint8_t deviceNumber = 0){return mrm_col_can->colorRed(deviceNumber);}

	/** Front side - right distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - distance in mm
	*/
	uint16_t rightBack(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Front side - right distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - distance in mm
	*/
	uint16_t rightFront(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Roll
	@return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0�.
	*/
	float roll();

	/** Color sensor's saturation
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - saturation.
	*/
	uint8_t saturation(uint8_t deviceNumber);

	/** Move servo
	@param degrees - Servo's target angle, 0 - 180°, or 0 - 360°, depending on model, counting clockwise
	@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
	*/
	void servo(uint16_t degrees = 90, uint8_t servoNumber = 0);

	/** Display fixed sign stored in sensor
	@image - sign's number
	*/
	void sign(uint8_t number);

	/** Stop the robot
	*/
	void stop();

	/** Store 8x8 image to 8x8 LED's internal memory
	@red - red pixels
	@green - green pixels
	@image - image's number
	*/
	void store(uint8_t red[], uint8_t green[], uint8_t image);

	/** Prints line and color sensors. Used for debugging.
	@param newLine - new line
	@param delayMsAfterPrint - delay after print
	*/
	void surfacePrint(bool newLine = false, uint16_t delayMsAfterPrint = 0);

	/** Turns the robot clockwise using compass.
	@param byDegreesClockwise - turn by defined number of degrees.
	*/
	void turn(int16_t byDegreesClockwise);

	/** Color sensor's value
	@param deviceNumber - device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - value
	*/
	uint8_t value(uint8_t deviceNumber);

	/** Color sensor's violet
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - intensity of violet
	 */
	uint16_t violet(uint8_t deviceNumber = 0){return mrm_col_can->colorViolet(deviceNumber);}

	/** Follows a wall.
	*/
	void wallFollow();

	/** Wall left?
	@return - true if wall
	*/
	bool wallLeft();

	/** Wall right?
	@return - true if wall
	*/
	bool wallRight();

	/** Color sensor's yellow
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - intensity of yellow
	 */
	uint16_t yellow(uint8_t deviceNumber = 0){return mrm_col_can->colorYellow(deviceNumber);}
};

/** Actions serve a few purposes.
- They encapsulate in classes actions robot has to perform. So, we have classes for robot's parts, but here also for non-material terms.
- No global variables are used. When an information should be shared between one (but called repeatedly) or more functions, it will be stored inside the action object. For example, all the
start conditions will be in the object itself.
- You can use inheritance to indicate relationships between actions, which indeed exist. For example, a movement can be movement straight ahead or turning.
- You can use in a consistent way actions defined for the base robot, without its code being exposed here.
- The actions are included in menus just by including a parameter in the constructor call.
- Buttons can be used to start actions, as well as menu commands. Menus are displayed both in the connected PC and a Bluetooth device, like a mobile phone, and any of the 2 can be used to
issue commands.
*/

/** Actions specific for a RobotLine robot will be defined here. They are all derived from ActionBase class (inheriting its methods and properties).
They also all feature perform() function, the one that is called to do what they are supposed to. Their constructors always call base class' constructors [ActionBase(...)] with 3 or more
parameters specified here.
First parameter is robot and is always the same.
The second one is a 3-letter shortcut that is displayed in command menu. For example "lin" will be displayed for starting the Rescue Line run. When action is not supposed to be started from menu,
it can be an empty string.
The third parameter is a name of the action, again displayed in menu. For "lin", the name is "RCJ Line", causing menu entry "line - RCJ Line" to be displayed. Again, use empty string
for no-menu actions.
The fourth pareameter is menu level. When omitted, the action will not be a part of the menu. Use 1 otherwise. Higher level numbers will display the action in submenues, not used here.
*/

/** Follow a line.
*/
class ActionLineFollow : public ActionBase {
	void perform(){ ((RobotLine*)_robot)->lineFollow(); }
public:
	ActionLineFollow(RobotLine* robot) : ActionBase(robot, "lnf", "Line follow", 1) {}
};

/** Avoid an obstacle.
*/
class ActionObstacleAvoid : public ActionBase {
	void perform() { ((RobotLine*)_robot)->obstacleAvoid(); }
public:
	bool leftOfObstacle; // Obstacle is on the robot's left side

	ActionObstacleAvoid(RobotLine* robot) : ActionBase(robot, "obs", "Obstacle avoid", 0) {}
};

/** Evacuation zone algorithm.
*/
class ActionEvacuationZone : public ActionBase {
	void perform() { ((RobotLine*)_robot)->evacuationZone(); }
public:

	ActionEvacuationZone(RobotLine* robot) : ActionBase(robot, "eva", "Evacuation zone", 1) {}
};

/** Start RCJ Line run.
*/
class ActionRCJLine : public ActionBase {
	void perform() { ((RobotLine*)_robot)->rcjLine(); }
public:
	ActionRCJLine(Robot* robot) : ActionBase(robot, "lin", "RCJ Line", 1) {}
};

/** Follow a wall.
*/
class ActionWallFollow : public ActionBase {
	void perform() { ((RobotLine*)_robot)->wallFollow(); }
public:
	ActionWallFollow(RobotLine* robot) : ActionBase(robot, "wal", "Wall follow") {}
};


// ****************** Generic actions

class ActionLoop0 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop0(); }
public:
	ActionLoop0(RobotLine* robot) : ActionBase(robot, "lo0", "Loop 0", 8) {}
};

class ActionLoop1 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop1(); }
public:
	ActionLoop1(RobotLine* robot) : ActionBase(robot, "lo1", "Loop 1", 8) {}
};

class ActionLoop2 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop2(); }
public:
	ActionLoop2(RobotLine* robot) : ActionBase(robot, "lo2", "Loop 2", 8) {}
};

class ActionLoop3 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop3(); }
public:
	ActionLoop3(RobotLine* robot) : ActionBase(robot, "lo3", "Loop 3", 8) {}
};

class ActionLoop4 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop4(); }
public:
	ActionLoop4(RobotLine* robot) : ActionBase(robot, "lo4", "Loop 4", 8) {}
};

class ActionLoop5 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop5(); }
public:
	ActionLoop5(RobotLine* robot) : ActionBase(robot, "lo5", "Loop 5", 8) {}
};

class ActionLoop6 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop6(); }
public:
	ActionLoop6(RobotLine* robot) : ActionBase(robot, "lo6", "Loop 6", 8) {}
};

class ActionLoop7 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop7(); }
public:
	ActionLoop7(RobotLine* robot) : ActionBase(robot, "lo7", "Loop 7", 8) {}
};

class ActionLoop8 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop8(); }
public:
	ActionLoop8(RobotLine* robot) : ActionBase(robot, "lo8", "Loop 8", 8) {}
};

class ActionLoop9 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loop9(); }
public:
	ActionLoop9(RobotLine* robot) : ActionBase(robot, "lo9", "Loop 9", 8) {}
};

/** Menu for generic actions
*/
class ActionLoopMenu : public ActionBase {
	void perform() { ((RobotLine*)_robot)->loopMenu(); }
public:
	ActionLoopMenu(Robot* robot) : ActionBase(robot, "lme", "Loop (menu)", 1) {}
};

class ActionMotorShortTest : public ActionBase {
	void perform() { ((RobotLine*)_robot)->motorShortTest(); }
public:
	ActionMotorShortTest(Robot* robot) : ActionBase(robot, "msh", "Motor short test") {}
};
