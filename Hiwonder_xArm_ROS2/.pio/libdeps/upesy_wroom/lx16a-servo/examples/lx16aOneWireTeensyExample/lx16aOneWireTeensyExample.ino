#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(&servoBus, 5);

void setup() {
	Serial.begin(115200);
	Serial.println("Beginning Servo Example");
	servoBus.beginOnePinMode(&Serial2, 8);
	servoBus.debug(true);
	servoBus.retry=0;
}

void loop() {
	int divisor =4;

	for (int x = -9000; x < 9000; x+=1000) {
		long start = millis();
		int angle = x;
		int16_t pos = 0;
		pos = servo.pos_read();
//		Serial.printf("\n\nPosition at %d -> %s\n", pos,
//				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");

		//do {
			servo.move_time(angle, 10*divisor);
		//} while (!servo.isCommandOk());
//		Serial.printf("Move to %d -> %s\n", angle,
//				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
//		Serial.println("Voltage = " + String(servo.vin()));
//		Serial.println("Temp = " + String(servo.temp()));
//		Serial.println("ID  = " + String(servo.id_read()));
//		Serial.println("Motor Mode  = " + String(servo.readIsMotorMode()));
		long took = millis()-start;
		long time = (10*divisor)-took;
		if(time>0)
			delay(time);
		else{
			Serial.println("Real Time broken, took: "+String(took));
		}
	}

	servo.move_time(-9000, 3000);
	delay(4000);
}
