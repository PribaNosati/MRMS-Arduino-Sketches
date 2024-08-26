#include <Arduino.h>
#include <mrm-col-can.h>
#include "mrm-robot-line.h"
#include "mrm-robot-maze.h"
#include "mrm-robot-min.h"
#include "mrm-robot-soccer.h"

Robot *robot;

void setup() {
	robot = new RobotMaze((char*)"Pluto"); // RobotLine, RobotMaze, RobotMin, RobotSoccer, or Your custom robot. "My robot" is Bluetooth name.
	print("Start.\n\r");
}

void loop() {
	robot->refresh();
}
 