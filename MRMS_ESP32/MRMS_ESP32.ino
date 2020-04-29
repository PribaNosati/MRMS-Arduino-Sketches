#include <Arduino.h>
#include <BluetoothSerial.h>
#include <mrm-8x8a.h>
#include <mrm-board.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-can-esp32.h>
#include <mrm-col-can.h>
#include <mrm-common.h>
#include <mrm-imu.h>
#include <mrm-pid.h>
#include <mrm-ir-finder2.h>
#include <mrm-ir-finder-can.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-node.h>
#include <mrm-ref-can.h>
#include <mrm-robot.h>
#include <mrm-robot-line.h>
#include <mrm-robot-soccer.h>
#include <mrm-servo.h>
#include <mrm-switch.h>
#include <mrm-therm-b-can.h>
#include <mrm-us.h>
#include <Wire.h>

Robot *robot;

void setup() {
	robot = new RobotLine(); // RobotLine, RobotSoccer, or Your custom robot
	robot->run();
}

void loop() {}

