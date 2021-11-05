#include <Arduino.h>
#include <mrm-8x8a.h>
#include <mrm-board.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-can-bus.h>
#include <mrm-col-b.h>
#include <mrm-col-can.h>
#include <mrm-common.h>
#include <mrm-fet-can.h>
#include <mrm-imu.h>
//#include <mrm-ir-finder2.h>
#include <mrm-ir-finder3.h>
//#include <mrm-ir-finder-can.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-node.h>
#include <mrm-pid.h>
#include <mrm-ref-can.h>
#include <mrm-robot.h>
#include "mrm-robot-line.h"
#include "mrm-robot-maze.h"
#include "mrm-robot-min.h"
#include "mrm-robot-soccer.h"
#include <mrm-servo.h>
#include <mrm-switch.h>
#include <mrm-therm-b-can.h>
//#include <mrm-us.h>
#include <mrm-us-b.h>
#include <mrm-us1.h>

Robot *robot;

void setup() {
	robot = new RobotLine((char*)"Minnie"); // RobotLine, RobotMaze, RobotMin, RobotSoccer, or Your custom robot. "My robot" is Bluetooth name.
	robot->print("Start.\n\r");
}

void loop() {
	robot->refresh();
}
