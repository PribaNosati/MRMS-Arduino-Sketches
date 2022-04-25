#include <mrm-8x8a.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-col-can.h>
#include <mrm-imu.h>
#include <mrm-ir-finder3.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-mot4x10.h>
#include "mrm-robot-min.h"
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>

class ActionMotorShortTest;

/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotMin::RobotMin(char name[]) : Robot(name) {
	// MotorGroup class drives the motors.
	// 2nd, 4th, 6th, and 8th parameters are output connectors of the controller (0 - 3, meaning 1 - 4. connector). 2nd one must be connected to LB (Left-Back) motor,
	// 4th to LF (Left-Front), 6th to RF (Right-Front), and 8th to RB (Right-Back). Therefore, You can connect motors freely, but have to
	// adjust the parameters here. In this example output (connector) 3 is LB, etc.
	//motorGroup = new MotorGroupDifferential(this, mrm_mot4x3_6can, 3, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 0);
	motorGroup = new MotorGroupDifferential(this, mrm_bldc4x2_5, 3, mrm_bldc4x2_5, 1, mrm_bldc4x2_5, 2, mrm_bldc4x2_5, 0);

	// Depending on your wiring, it may be necessary to spin some motors in the other direction. In this example, no change needed,
	// but uncommenting the following line will change the direction of the motor 2.
	//mrm_mot4x3_6can->directionChange(2);

	// All the actions will be defined here; the objects will be created. 


	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus. For example, ActionWallsTest test is only manu-based, and it is all right.
	// This test is not supposed to be called in code.


	// Set buttons' actions
	//mrm_8x8a->actionSet(_actionCANBusStress, 1); // Starts stress test.
	mrm_8x8a->actionSet(_actionLoop, 2); // Free-defined action.
	mrm_8x8a->actionSet(_actionMenuMain, 3); // Stop and display menu
	// Put Your buttons' actions here.

	// Upload custom bitmaps into mrm-8x8a.
	bitmapsSet();
}

/** Custom test. The function will be called many times during the test, till You issue "x" menu-command.
*/
void RobotMin::loop() {

}
