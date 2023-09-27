#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(&servoBus, 1);
void setup() {
	servoBus.begin(&Serial1,
			1,// on TX pin 1
			2);// use pin 2 as the TX flag for buffer
	Serial.begin(115200);
}

void loop() {
	for (int i = -10; i < 10; i++) {
		int16_t pos = 0;
		pos = servo.pos_read();
		Serial.printf("\n\nPosition at %d -> %s\n", pos,
				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");

		int16_t angle = i * 100;


		servo.motor_mode(angle);

		Serial.printf("Move to %d -> %s\n", angle,
				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
		Serial.println("Voltage = " + String(servo.vin()));
		Serial.println("Temp = " + String(servo.temp()));
		Serial.println("ID  = " + String(servo.id_read()));
		delay(500);
	}
	servo.move_time(0, 500);
	delay(5000);
}
