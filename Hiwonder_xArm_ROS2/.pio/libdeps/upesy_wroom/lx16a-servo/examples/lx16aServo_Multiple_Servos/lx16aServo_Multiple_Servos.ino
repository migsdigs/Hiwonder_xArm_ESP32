#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(&servoBus, 1);
LX16AServo servo2(&servoBus, 2);
LX16AServo servo3(&servoBus, 3);
void setup() {
	servoBus.begin(&Serial1, 1, // on TX pin 1
			2); // use pin 2 as the TX flag for buffer
	Serial.begin(115200);
	servoBus.retry = 1; // enforce synchronous real time
	servoBus.debug(true);
	Serial.println("Beginning Coordinated Servo Example");

}
// 40ms trajectory planning loop seems the most stable

void loop() {
	int divisor = 3;
	for (int i = 0; i < 1000 / divisor; i++) {
		long start = millis();
		int16_t pos = 0;
		pos = servo.pos_read();
		int16_t pos2 = servo2.pos_read();
		int16_t pos3 = servo3.pos_read();

		uint16_t angle = i * 24 * divisor;

		servo2.move_time_and_wait_for_sync(angle, 10 * divisor);
		servo3.move_time_and_wait_for_sync(angle, 10 * divisor);
		servo.move_time_and_wait_for_sync(angle, 10 * divisor);

		servoBus.move_sync_start();

		//if(abs(pos2-pos)>100){
		Serial.printf("\n\nPosition at %d and %d-> %s\n", pos, pos2,
				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
		Serial.printf("Move to %d -> %s\n", angle,
				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
		//}
		long took = millis() - start;

		long time = (10 * divisor) - took;
		if (time > 0)
			delay(time);
		else {
			Serial.println("Real Time broken, took: " + String(took));
		}
	}
	Serial.println("Interpolated Set pos done, not long set");

	servoBus.retry = 5; // These commands must arrive
	servo.move_time(0, 10000);
	servo2.move_time(0, 10000);
	servo3.move_time(0, 10000);
	servoBus.retry = 0; // Back to low latency mode
	for (int i = 0; i < 1000 / divisor; i++) {
		long start = millis();
		int16_t pos = 0;
		pos = servo.pos_read();
		int16_t pos2 = servo2.pos_read();
		int16_t pos3 = servo3.pos_read();

		Serial.printf("\n\nPosition at %d and %d\n", pos, pos2);

		Serial.println("Voltage = " + String(servo.vin()));
		Serial.println("Temp = " + String(servo.temp()));

		long took = millis() - start;

		long time = (10 * divisor) - took;
		if (time > 0)
			delay(time);
		else {
			Serial.println("Real Time broken, took: " + String(took));
		}
	}
	Serial.println("Loop resetting");
}
