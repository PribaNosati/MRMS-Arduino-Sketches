#pragma once
#include <mrm-action.h>
#include <mrm-pid.h>
#include <mrm-robot.h>

// Constants
#define BARRIER_MID_VALUE 600 // 0 - 4095
#define SOCCER_ANGULAR_SPEED_LIMIT 40 // 0 - 100
#define SOCCER_BARRIER_PIN 35 // Not to be changed unless pin reassigned
#define SOCCER_BACK_DISTANCE_WHEN_GOALIE 200 // Goalie's optimum distance when in front of won goal
#define SOCCER_SIDE_DISTANCE_WHEN_CENTERED 700 // Distance left and right when in the centre of the field
#define SOCCER_SPEED_LIMIT 60 // 0 - 127

/** Robot for RCJ Soccer
*/
class RobotSoccer : public Robot {
	enum TriState{Yes, Opposite, Unknown};

	// Actions' declarations
	ActionBase* actionBounce;
	ActionBase* actionCalibrate;
	ActionBase* actionCatch;
	ActionBase* actionGoalApproach;
	ActionBase* actionIdle;
	ActionBase* actionLineAvoid;
	ActionBase* actionPlay;

	float headingToMaintain; // Heading towards opponent's goal.
	MotorGroupStar* motorGroup;  // Class that conveys commands to motors.
	Mrm_pid* pidXY; // PID controller, regulates motors' speeds for linear motion in the x-y plane.
	Mrm_pid* pidRotation; // PID controller, regulates rotation around z axis.
public:
	/** Constructor
	@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
	*/
	RobotSoccer(char name[] = (char*)"RCJ Soccer");

	/** Rear distance to wall
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
	@return - in mm
	*/
	uint16_t back(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Ball's direction
	@return - robot's front is 0�, positive angles clockwise, negative anti-clockwise. Back of the robot is 180�.
	*/
	int16_t ballAngle();

	/** Read barrier
	@return - true if interrupted
	*/
	bool barrier();

	/** Test barrier
	*/
	void barrierTest();

	/** Store bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	/** Bouncing off the lines
	*/
	void bounce();

	/** Line sensor - brightness of the surface
	@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - brightness as an analog value.
	*/
	uint16_t brightness(uint8_t transistorNumber, uint8_t deviceNumber);

	/** Reads push button switch
	@number - 0 to 3, push button's ordinal number
	@return - true if pressed
	*/
	bool button(uint8_t number);

	/** Calibrate all line sensors
	 */
	void calibrate();

	/** Go around the ball and approach it.
	*/
	void catchBall();

	/** Front distance to wall
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
	@return - in mm
	*/
	uint16_t front(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
	@param speed - 0 to 100.
	@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
	Values between -180 and 180.
	@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
	numbers because a value 100 turns on all the motors at maximal speed.
	@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
	*/
	void go(float speed, float angleDegrees, float rotation, uint8_t speedLimit = 127);

	/** Test - go straight ahead.
	*/
	void goAhead();

	/** Approach oppoent's goal
	*/
	void goalApproach();

	/**Compass
	@return - North is 0�, clockwise are positive angles, values 0 - 360.
	*/
	float heading();

	float headingRandom(int newHeading, int variation);

	/** No ball detected - return to Your goal.
	*/
	void idle();

	/** Front distance to wall
	@return - in mm
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
	*/
	uint16_t left(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Line sensor
	@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - true if white line found
	*/
	bool line(uint8_t transistorNumber, uint8_t deviceNumber);

	bool lineAny();

	void lineAvoid();

	/** Custom test
	*/
	void loop();

	/** Starts robot.
	*/
	void play();

	/**Pitch
	@return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0�.
	*/
	float pitch();

	/** Right distance to wall
	@param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
	@return - in mm
	*/
	uint16_t right(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Roll
	@return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0�.
	*/
	float roll();

	/** Display fixed sign stored in sensor
	@image - sign's number
	*/
	void sign(uint8_t number);
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

/** Actions specific for a RobotSoccer robot will be defined here. They are all derived from ActionBase class (inheriting its methods and properties).
They also all feature perform() function, the one that is called to do what they are supposed to. Their constructors always call base class' constructors [ActionBase(...)] with 3 or more
parameters specified here.
First parameter is robot and is always the same.
The second one is a 3-letter shortcut that is displayed in command menu. For example "soc" will be displayed for starting the RCJ Soccer run. When action is not supposed to be started from menu,
it can be an empty string.
The third parameter is a name of the action, again displayed in menu. For "soc", the name is "Soccer play", causing menu entry "soc - Soccer play" to be displayed. Again, use empty string
for no-menu actions.
The fourth pareameter is menu level. When omitted, the action will not be a part of the menu. Use 1 otherwise. Higher level numbers will display the action in submenues, not used here.
*/

/** Bounce
*/
class ActionSoccerBounce : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->bounce(); }
public:
	ActionSoccerBounce(RobotSoccer* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "bou", "Soccer bounce", 1, ID_ANY, ledSign) {}
};

/** Test barrier.
*/
class ActionSoccerBarrierTest : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->barrierTest(); }
public:
	ActionSoccerBarrierTest(RobotSoccer* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "bar", "Soccer barr. test", 1, ID_ANY, ledSign) {}
};

/** Calibrating all the line sensors
*/
class ActionSoccerCalibrate : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->calibrate(); }
public:
	ActionSoccerCalibrate(RobotSoccer* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "clb", "Soccer calibrate", 1, ID_ANY, ledSign) {}
};


/** Go around the ball and approach it.
*/
class ActionSoccerCatch : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->catchBall(); }
public:
	ActionSoccerCatch(RobotSoccer* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "cat", "Soccer catch", 0, ID_ANY, ledSign) {}
};

/** Go to goal
*/
class ActionSoccerGoalApproach : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->goalApproach(); }
public:
	ActionSoccerGoalApproach(RobotSoccer* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "apr", "Soccer approach", 0, ID_ANY, ledSign) {}
};

/** Starts robot.
*/
class ActionSoccerPlay : public ActionBase {
	void perform(){ ((RobotSoccer*)_robot)->play(); }
public:
	ActionSoccerPlay(RobotSoccer* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "soc", "Soccer play") {}
};

/** Idle position, before own goal.
*/
class ActionSoccerIdle : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->idle(); }
public:
	ActionSoccerIdle(RobotSoccer* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "idl", "Soccer idle", 0, ID_ANY, ledSign) {}
};

class ActionSoccerLineAvoid : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->lineAvoid(); }
public:
	ActionSoccerLineAvoid(RobotSoccer* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "avo", "Soccer line avoid", 0, ID_ANY, ledSign) {}
};
