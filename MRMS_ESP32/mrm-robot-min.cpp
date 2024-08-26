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
#include <EEPROM.h>

class ActionMotorShortTest;

#define AUTOSTART 0
#define DEVICE_COUNT_COLOR 0
#define DEVICE_COUNT_LED 1
#define DEVICE_COUNT_IR_FINDER 1
#define DEVICE_COUNT_LIDARS 4
#define DEVICE_COUNT_MOTORS 4
#define DEVICE_COUNT_REFLECTIVE 4
#define START_MOTORS 0
/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotMin::RobotMin(char name[]) : Robot(name) {
	// MotorGroup class drives the motors.
	// 2nd, 4th, 6th, and 8th parameters are output connectors of the controller (0 - 3, meaning 1 - 4. connector). 2nd one must be connected to LB (Left-Back) motor,
	// 4th to LF (Left-Front), 6th to RF (Right-Front), and 8th to RB (Right-Back). Therefore, You can connect motors freely, but have to
	// adjust the parameters here. In this example output (connector) 3 is LB, etc.
	motorGroup = new MotorGroupDifferential(this, mrm_mot4x3_6can, 3, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 0);
	// motorGroup = new MotorGroupDifferential(this, mrm_bldc4x2_5, 3, mrm_bldc4x2_5, 1, mrm_bldc4x2_5, 2, mrm_bldc4x2_5, 0);

	// Depending on your wiring, it may be necessary to spin some motors in the other direction. In this example, no change needed,
	// but uncommenting the following line will change the direction of the motor 2.
	//mrm_mot4x3_6can->directionChange(2);

	// All the actions will be defined here; the objects will be created. 


	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus. For example, ActionWallsTest test is only manu-based, and it is all right.
	// This test is not supposed to be called in code.

	// Set buttons' actions
	mrm_8x8a->actionSet(_actionLoop0, 1); // Starts stress test.
	mrm_8x8a->actionSet(_actionLoop, 2); // Free-defined action.
	mrm_8x8a->actionSet(_actionMenuMain, 3); // Stop and display menu
	// Put Your buttons' actions here.

	// Upload custom bitmaps into mrm-8x8a.
	bitmapsSet();

	delayMs(500); // For CAN Bus boot

#if AUTOSTART
	_actionCurrent = _actionLoop0; // Comment the line if no default action
#endif
	pinMode(26, OUTPUT);
	digitalWrite(26, LOW);
}

/** Custom test. The function will be called many times during the test, till You issue "x" menu-command.
*/
void RobotMin::loop() {
	#define LIST_ALL 0
	#define TEST1 0
	#define TEST2 0
	#define TEST3 0
	#define TEST4 0
	#define TEST5 1

	#if LIST_ALL
	uint8_t cnt = 0;
    uint8_t i = 0;
    do{
        deviceInfo(i, boardInfo, SENSOR_BOARD);
        if (strcmp(boardInfo->name, "") != 0){
            print("Sensor: %s, device nr: %i readings: %i \n\r", boardInfo->name, boardInfo->deviceNumber, boardInfo->readingsCount);
            cnt = boardInfo->readingsCount;
            for (uint8_t j = 0; j < cnt; j++)
                print("%i ", (((SensorBoard *)boardInfo->board)->reading(j, boardInfo->deviceNumber)));
            print("\n\r");
        }
        else
            cnt = 0;
        i++;
    }while(cnt != 0);
    end();
	#endif
	#if TEST1
	// if (setup()){
	// 	mrm_bldc4x2_5->speedSet(0, 127);
	// 	delayMs(1000);
	// 	mrm_bldc4x2_5->stop();
	// 	end();
	// }

	// static int16_t speed = 120;
	// 	mrm_bldc4x2_5->speedSet(0, speed);
	// 	print("%i\n\r", speed);

	// 	delayMs(2000);
	// speed = -speed;

	// if (setup()){
	// 	// // mrm_bldc4x2_5->start();
	// 	mrm_bldc4x2_5->speedSet(0, 20);
	// }
	// print("Enc:%i\n\r", mrm_bldc4x2_5->reading(0));
	#endif

	#if TEST2
	#define OUTPUT_MOTORS 1
	#define TOP_SPEED_TEST 100 //30
	// if (setup()) { // This part will execute only in the first run.
	// 	pinMode(27, OUTPUT);
	// 	pinMode(26, INPUT);
	// 	delay(500);
	// }
	static bool ok = true;
	if (setup())
		ok = true;

	// mrm-mot4x3.6_can
	static int16_t speed = 10;
	static bool up = true;
	#if OUTPUT_MOTORS
	motorGroup->go(speed, speed);
	#endif
	if (up)
		speed++;
	else
		speed--;
	// Change motor's direction
	if ((up && speed > TOP_SPEED_TEST) || (!up && speed < -TOP_SPEED_TEST))
		up = !up;

	// mrm-mot4x10
	// for (uint8_t i = 0; i < 4; i++)
	// 	mrm_mot4x10->speedSet(i, speed);

	// mrm-fets
	// if (speed == 120 && up) {
	// 	digitalWrite(27, HIGH);
	// 	delay(150);
	// 	digitalWrite(27, LOW);
	// }

	
	#define MRM_LID_CAN_B 1
	#define MRM_LID_COUNT 3
	#if MRM_LID_CAN_B
	// mrm-lid-can-b
	static uint32_t lastLidarMs = 0;
	static uint16_t lidarLast[3];
	static uint16_t lidarCount[3];
	if (millis() - lastLidarMs > 150){
		lastLidarMs = millis();
		for (uint8_t i = 0; i < MRM_LID_COUNT; i++){
			if (mrm_lid_can_b->alive(i)){
				if (mrm_lid_can_b->distance(i) == lidarLast[i] && mrm_lid_can_b->distance(i) != 2000){
					lidarCount[i]++;
					if (lidarCount[i] > 3000){
						print("Lidar stopped.");
						mrm_8x8a->bitmapDisplay('L');
						ok = false;
					}
				}
				else
					lidarCount[i] = 0;
				lidarLast[i] = mrm_lid_can_b->distance();
			}
		}
	}
	#endif


	#define MRM_LID_CAN_B2 0
	#if MRM_LID_CAN_B2
	// mrm-lid-can-b2
	static uint32_t lastLidar4Ms = 0;
	static uint16_t lidar4Last[3];
	static uint16_t lidar4Count[3];
	if (millis() - lastLidar4Ms > 150){
		lastLidar4Ms = millis();
		for (uint8_t i = 0; i < 3; i++){
			if (mrm_lid_can_b2->reading(i) == lidar4Last[i] && mrm_lid_can_b2->reading(i) != 2000){
				lidar4Count[i]++;
				if (lidar4Count[i] > 300){
					print("Lidar stopped.");
					mrm_8x8a->bitmapDisplay('L');
					ok = false;
				}
			}
			else
				lidar4Count[i] = 0;
			lidar4Last[i] = mrm_lid_can_b2->reading(i);
		}
	}
	#endif

	// mrm-ref-can
	#define MRM_REF_TEST 1
	#if MRM_REF_TEST
	static uint32_t lastRefMs = 0;
	static uint16_t refLast[9];
	static uint16_t refCount[9];
	if (millis() - lastRefMs > 150){
		lastRefMs = millis();
		for (uint8_t i = 0; i < 9; i++){
			if (mrm_ref_can->reading(i) == refLast[i]){
				refCount[i]++;
				if (refCount[i] > 300){
					print("Ref stopped.");
					mrm_8x8a->bitmapDisplay('R');
					ok = false;
				}
			}
			else
				refCount[i] = 0;
			refLast[i] = mrm_ref_can->reading(i);
		}
	}
	#endif

	// Display
	static uint8_t timeCnt = 0;
	static uint32_t lastDisplayMs = 0;
	if (millis() - lastDisplayMs > 500){
		#if MRM_LID_CAN_B
		print("Lid:");
		for (uint8_t i = 0; i < MRM_LID_COUNT; i++)
			if (mrm_lid_can_b->alive(i))
				print("%i ", mrm_lid_can_b->distance(i));
		#endif
		#if MRM_LID_CAN_B2
		print("Lid:");
		for (uint8_t i = 0; i < 3; i++)
			print("%i ", mrm_lid_can_b2->distance(i));
		#endif
		#if MRM_REF_TEST
		print(", ref:");
		for (uint8_t i = 0; i < 9; i++)
			print("%i ", mrm_ref_can->reading(i));
		#endif
		#define MRM_THERM_TEST 0
		#if MRM_THERM_TEST
		print(", therm:");
		for (uint8_t i = 0; i < 2; i++)
			print("%i ", mrm_therm_b_can->reading(i));
		#endif
		#define MRM_ENCODERS_TEST 0
		#if MRM_ENCODERS_TEST
		print(", enc:");
		for (uint8_t i = 0; i < 4; i++)
			print("%i ", mrm_bldc4x2_5->reading(i));
		#endif
		#if OUTPUT_MOTORS
		print(", mot:%i", speed);
		#endif
		print("\n\r");

		// mrm-led8x8
		static uint8_t currentChar = 33;
		if (ok){
			mrm_8x8a->bitmapDisplay(currentChar);
			if (++currentChar > 90)
				currentChar = 33;
		}

		lastDisplayMs = millis();

		if (timeCnt++ > 40){
			timeCnt = 0;
			print("Time: %i min\n\r", millis() / 1000 / 60);
		}
	}

	//mrm-col-can, mrm-ir-finder3, mrm-lid-can-b2, mrm-ref-can, mrm-therm-b-can, mrm-barr2*, mrm-enc
	// print("lid:%imm ir:%i refArr:%i th:%ideg barr:%i col:%i, enc:%i, col:%i\n\r", mrm_lid_can_b2->reading(0), mrm_ir_finder3->reading(0), mrm_ref_can->reading(0), mrm_therm_b_can->reading(0),
	// 	analogRead(36), mrm_col_can->reading(0), digitalRead(26), mrm_col_can->reading(0));

	// print("lid4m:%imm refArr:%i th:%ideg \n\r", mrm_lid_can_b2->reading(0), mrm_ref_can->reading(0), mrm_therm_b_can->reading(0));
	// actionSet(_actionLoop);
	#endif

	#if TEST3
	print("%i %i\n\r", mrm_therm_b_can->reading(0),  mrm_therm_b_can->reading(1));
	if ( mrm_therm_b_can->reading(0) > 50 ||  mrm_therm_b_can->reading(1) > 50)
		end();
	delayMs(100);
	#endif

	#if TEST4
		EEPROM.begin(12);
		uint8_t address = 0;
		EEPROM.write(address, 0xFF);
		address++;
		EEPROM.write(address, 17);
		EEPROM.commit(); // Warning: only 100000 times writeable

		address = 0;
		uint8_t content = EEPROM.read(address);
		address++;
		uint8_t content1 = EEPROM.read(address);
		print("Content: %i %i\n\r", (int)content, (int)content1);
		if (content == 0xFF){
			mrm_8x8a->text("Last failed");
			print("Last failed");
			EEPROM.begin(12);
			EEPROM.write(0, 0);
			EEPROM.commit(); // Warning: only 100000 times writeable
			for(;;);
		}
		else
			print("Nothing");
		end();
	#endif

	#if TEST5
		static const uint8_t PINS_COUNT = 7;
		static uint8_t pins[] = {12, 13, 14, 16, 25, 32, 33};
		if (setup()){
			print("Test master\n\r");
			for (uint8_t i = 0; i < PINS_COUNT; i++)
				pinMode(pins[i], OUTPUT);
			pinMode(27, INPUT_PULLDOWN);
		}
		print("On\n\r");
		for (uint8_t i = 0; i < PINS_COUNT; i++){
			digitalWrite(pins[i], HIGH);
			delayMs(100);
		}
		delayMs(50000);
		if (digitalRead(27)){
			mrm_8x8a->text((char*)"Break");
			print("End.");
			end();
		}
		else{
			print("Off\n\r");
			for (uint8_t i = 0; i < PINS_COUNT; i++){
				digitalWrite(pins[i], LOW);
				delayMs(100);
			}
			delayMs(3000);
		}
	#endif
}

#define EEPROM_SIZE 12

void RobotMin::loop0(){
	static uint32_t i = 0;
	bool ok = true;
	if (setup()){
		pinMode(26, OUTPUT);
		print("Started - devices test.\n\r");
		uint8_t address = 0;
		uint8_t count;
		if (EEPROM.read(address) == 0xFF){
			mrm_8x8a->text("Last failed");
			print("Last failed");
			end();
			ok = false;
		}
	}
	if (ok){
		uint8_t count = devicesScan(true);
		actionSet(_actionLoop0);
		if (count == DEVICE_COUNT_LED + DEVICE_COUNT_LIDARS + DEVICE_COUNT_MOTORS + 
			DEVICE_COUNT_REFLECTIVE + DEVICE_COUNT_COLOR + DEVICE_COUNT_IR_FINDER){
			print("Pass %i OK\n\r", ++i);
			#if START_MOTORS
			int8_t leftSpeed = millis() % 255 - 128;
			int8_t rightSpeed = millis() % 255 - 128;
			motorGroup->go(leftSpeed, rightSpeed);
			#endif
		}
		else{
			bool ref = mrm_ref_can->alive(0);
			bool lid0 = mrm_lid_can_b->alive(0);
			bool lid1 = mrm_lid_can_b->alive(1);
			bool lid2 = mrm_lid_can_b->alive(2);
			bool mot0 = mrm_mot4x3_6can->alive(0);
			char buffer[40];
			sprintf(buffer, "%i dev, %s%s%s%s%s", count, ref ? "" : "R", lid0 ? "" : "L0", 
				lid1 ? "" : "L1", lid2 ? "" : "L2", mot0 ? "" : "M0");
			mrm_8x8a->text(buffer);
			print("%i devices, stop\n\r", count);
			digitalWrite(26, HIGH);
#if AUTOSTART
			EEPROM.begin(EEPROM_SIZE);
			uint8_t address = 0;
			EEPROM.write(address, 0xFF);
			address++;
			EEPROM.write(address, count);
			EEPROM.commit(); // Warning: only 100000 times writeable
			for(;;);
#else
			end();
#endif
		}
	}
}

void RobotMin::loop1(){
			EEPROM.begin(12);
			EEPROM.write(0, 0);
			EEPROM.commit(); // Warning: only 100000 times writeable
}

void RobotMin::loop2(){
	while (millis() < 3000) //Wait for all the devices to complete start-up
		delayMs(50);
		// print("AA1\n\r");
	devicesStop();

	delayMs(5); // Read all the messages sent after stop.

	// Set not alive
print("Board: %s\n\r", board[14]->name());
			board[14]->aliveSet(false); // Mark as not alive. It will be marked as alive when returned message arrives.

	// Send alive ping
	for (uint8_t k = 0; k < 2; k++)
				board[14]->devicesScan(verbose);
				// print("SC1 %s ", board[i]->name()),count += board[i]->devicesScan(verbose), print("SC2");


	// Count alive
	uint8_t count = 0;
			count += board[14]->aliveCount(); 


		print("%i devices.\n\r", count);

	if (count == 0){
		print("No ref\n\r");
		_devicesScanBeforeMenuAndSwitches = false;
		end();
	}
}