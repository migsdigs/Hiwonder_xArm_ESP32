#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(&servoBus, 1);
LX16AServo servo2(&servoBus, 2);
LX16AServo servo3(&servoBus, 3);
int16_t startingAngles []= {-9000,0,9000};
bool calibrationDone = false;
#define HOME_SWITCH_PIN 32
int divisor = 3;
void setup() {
	while(!Serial);
	delay(1000);
	Serial.begin(115200);
	Serial.println("Starting");
	servoBus.begin(&Serial1, 1, // on TX pin 1
			2); // use pin 2 as the TX flag for buffer

	servoBus.retry = 2; // enforce synchronous real time
	//servoBus.debug(true);
	Serial.println("Beginning Coordinated Servo Example");
	pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);
	servo.disable();
	servo2.disable();
	servo3.disable();
}
// 40ms trajectory planning loop seems the most stable
void interp(int i){
	long start = millis();
	servoBus.move_sync_start();
	int32_t pos = servo.pos_read();
	int32_t pos2 = servo2.pos_read();
	int32_t pos3 = servo3.pos_read();

	int32_t angle = (i * 9 * divisor)-4500;
	int timingOffset = millis()-start;
	servo2.move_time_and_wait_for_sync(angle+4500, 10 * divisor+timingOffset);
	servo3.move_time_and_wait_for_sync(-4500-(angle), 10 * divisor+timingOffset);
	servo.move_time_and_wait_for_sync(angle*2, 10 * divisor+timingOffset);



	//if(abs(pos2-pos)>100){
//		Serial.printf("\n\nPosition at %d, %d  and %d-> %s\n", pos, pos2,pos3,
//				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");

//	Serial.printf("Move to %d -> %s\n", angle,
//			servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
	//}
	long took = millis() - start;

	long time = (10 * divisor) - took;
	if (time > 0)
		delay(time);
	else {
		Serial.println("Real Time broken, took: " + String(took));
	}
}
void loop() {
	bool HOME = !digitalRead(HOME_SWITCH_PIN);
	if(!calibrationDone){

		Serial.println("Homing pin "+String(HOME));

		if(HOME){
			Serial.println("Calibrating 1");
			servo.calibrate(startingAngles[0],-9000,9000);
			Serial.println("Calibrating 2");
			servo2.calibrate(startingAngles[1],-2512,10000);
			Serial.println("Calibrating 3");
			servo3.calibrate(startingAngles[2],-9000,9000);

			Serial.println("Limits read");
		}
		Serial.println("Read 1");
		int32_t pos = servo.pos_read();
		Serial.println("Read 2");
		int32_t pos2 = servo2.pos_read();
		Serial.println("Read 3");
		int32_t pos3 = servo3.pos_read();
		Serial.printf("\n\nPosition at %d, %d and %d-> %s\n", pos, pos2, pos3,
				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
		if(pos==startingAngles[0]&&
				pos2==startingAngles[1]&&
				pos3==startingAngles[2]&&
				HOME){
			calibrationDone=true;
			Serial.println("Calibration Done!");
			servo.move_time(0, 2000);
			delay(2000);
			servo2.move_time_and_wait_for_sync(0, 2000);
			servo3.move_time_and_wait_for_sync(0, 2000);
			servo.move_time_and_wait_for_sync(-9000, 2000);
			servoBus.move_sync_start();
			delay(2000);
		}
	}

	if(!calibrationDone){
		delay(50);
		return;
	}
	if(HOME){
		int32_t pos = servo.pos_read();
		int32_t 		pos2 = servo2.pos_read();
		int32_t 		pos3 = servo3.pos_read();
		Serial.printf("\n\nMAX at %d, %d and %d", servo.getMaxCentDegrees(),  servo2.getMaxCentDegrees(),  servo3.getMaxCentDegrees());
		Serial.printf("\nPos at %d, %d and %d", pos, pos2, pos3);
		Serial.printf("\nmin at %d, %d and %d", servo.getMinCentDegrees(), servo2.getMinCentDegrees(), servo3.getMinCentDegrees());
//
		delay(50);
		return;
	}

	for (int i = 0; i < 1000 / divisor; i++) {
		interp( i);
	}
	Serial.println("Interpolated Set pos done, not long set");

	for (int i = 1000 / divisor; i >0; i--) {
		interp( i);
	}
	Serial.println("Loop resetting");
}


