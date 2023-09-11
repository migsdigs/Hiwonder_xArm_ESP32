// Single Pin mode
#include <Arduino.h>
#include <lx16a-servo.h>
#include <WiFi.h>

// Servos and Servo Bus Instantiation
LX16ABus servoBus;
LX16AServo servo1(&servoBus, 1); // 0-16800 (0-168 degrees)
LX16AServo servo2(&servoBus, 2); // 0-24000 (0-240 degrees)
LX16AServo servo3(&servoBus, 3); // 0-24000 (0-240 degrees)
LX16AServo servo4(&servoBus, 4); // 0-24000 (0-240 degrees)
LX16AServo servo5(&servoBus, 5); // 0-24000 (0-240 degrees)
LX16AServo servo6(&servoBus, 6); // 0-24000 (0-240 degrees)

bool done_flag = true;

// Wifi const
const char* ssid = "Rogachino";
const char* password = "12345678";

bool LED_status = true;

void setup() {
	Serial.begin(115200);

	// LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LED_status);

	// WiFi setup
	delay(1000);
	WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

	while(WiFi.status() != WL_CONNECTED){
		LED_status = !LED_status;
		digitalWrite(LED_BUILTIN, LED_status);
        Serial.print(".");
        delay(100);
    }

	Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());

	// Servo Setup
	Serial.println("Beginning Servo Example");
	servoBus.beginOnePinMode(&Serial2,33);
	servoBus.debug(true);
	servoBus.retry=1;

	// servo6.move_time(0, 3000);
	// delay(6000);
	// servo6.move_time(24000, 3000);

	// delay(6000);

	// servo1.move_time(0, 3000);
	// delay(4000);
	// servo1.move_time(16000, 3000);
	// delay(4000);
	// servo1.move_time(8000, 3000);

	delay(6000);
	int id = 0;
	int temp = 0;
	int volt = 0;

	id = servo6.id_read(6);
	temp = servo6.temp();
	volt = servo6.vin();

	Serial.println("ID is: ");
	Serial.println(String(id));
	Serial.println("Servo temperature is: ");
	Serial.println(String(temp));
	Serial.println("Servo V_in is: ");
	Serial.println(String(volt));

	// Doesnt work ATM
	// servo.readLimits();
}

void loop() {
	// int divisor =4;

// 	for (int x = -9000; x < 9000; x+=1000) {
// 		long start = millis();
// 		int angle = x;
// 		int16_t pos = 0;
// 		pos = servo.pos_read();
// 		Serial.printf("\n\nPosition at %d -> %s\n", pos, servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");

// 	}
	if (done_flag == false) {
		Serial.println("Starting moving procedure.");
		// servo6.move_time(16000, 3000);
		// Serial.println("moving to 16000");
		// delay(4000);

		// servo6.move_time(4500, 3000);
		// Serial.println("moving to 4500");
		// delay(4000);

		// servo6.move_time(0, 3000);
		// Serial.println("moving to 0");
		// delay(4000);

		// servo.move_time(-4500, 3000);
		// Serial.println("moving to -4500");
		// delay(2000);

		// servo.move_time(-9000, 3000);
		// Serial.println("moving to -9000");
		// delay(2000);

		done_flag = true;

	}
	
	delay(4000);
}

// #include <Arduino.h>
// #include <lx16a-servo.h>
// LX16ABus servoBus;
// LX16AServo servo(&servoBus, 1);
// LX16AServo servo2(&servoBus, 2);
// LX16AServo servo3(&servoBus, 3);
// LX16AServo servo4(&servoBus, 4);
// LX16AServo servo5(&servoBus, 5);
// LX16AServo servo6(&servoBus, 6);

// void setup() {
// 	servoBus.begin(&Serial2, 26, // on TX pin 1
// 			25); // use pin 2 as the TX flag for buffer
// 	Serial.begin(115200);
// 	servoBus.retry = 1; // enforce synchronous real time
// 	servoBus.debug(true);
// 	Serial.println("Beginning Coordinated Servo Example");

// }
// // 40ms trajectory planning loop seems the most stable

// void loop() {
// 	int divisor = 3;
// 	for (int i = 0; i < 1000 / divisor; i++) {
// 		long start = millis();
// 		int16_t pos = 0;
// 		pos = servo.pos_read();
// 		int16_t pos2 = servo2.pos_read();
// 		int16_t pos3 = servo3.pos_read();

// 		uint16_t angle = i * 24 * divisor;

// 		servo2.move_time_and_wait_for_sync(angle, 10 * divisor);
// 		servo3.move_time_and_wait_for_sync(angle, 10 * divisor);
// 		servo.move_time_and_wait_for_sync(angle, 10 * divisor);

// 		servoBus.move_sync_start();

// 		//if(abs(pos2-pos)>100){
// 		Serial.printf("\n\nPosition at %d and %d-> %s\n", pos, pos2,
// 				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
// 		Serial.printf("Move to %d -> %s\n", angle,
// 				servo.isCommandOk() ? "OK" : "\n\nERR!!\n\n");
// 		//}
// 		long took = millis() - start;

// 		long time = (10 * divisor) - took;
// 		if (time > 0)
// 			delay(time);
// 		else {
// 			Serial.println("Real Time broken, took: " + String(took));
// 		}
// 	}
// 	Serial.println("Interpolated Set pos done, not long set");

// 	servoBus.retry = 5; // These commands must arrive
// 	servo.move_time(0, 10000);
// 	servo2.move_time(0, 10000);
// 	servo3.move_time(0, 10000);
// 	servoBus.retry = 0; // Back to low latency mode
// 	for (int i = 0; i < 1000 / divisor; i++) {
// 		long start = millis();
// 		int16_t pos = 0;
// 		pos = servo.pos_read();
// 		int16_t pos2 = servo2.pos_read();
// 		int16_t pos3 = servo3.pos_read();

// 		Serial.printf("\n\nPosition at %d and %d\n", pos, pos2);

// 		Serial.println("Voltage = " + String(servo.vin()));
// 		Serial.println("Temp = " + String(servo.temp()));

// 		long took = millis() - start;

// 		long time = (10 * divisor) - took;
// 		if (time > 0)
// 			delay(time);
// 		else {
// 			Serial.println("Real Time broken, took: " + String(took));
// 		}
// 	}
// 	Serial.println("Loop resetting");
// }




// // Playing with PWM pin
// #include <Arduino.h>
// #include <esp32-hal-ledc.h>
// // #include <lx16a-servo.h>

// // const variables
// const int pwmChannel = 0;
// const int pwmFreq = 1000;
// const int pwmResolution = 8;

// const int pwmPin = 23;

// void setup() {
//   Serial.begin(115200);
//   // Some LED stuff to verify code works
//   Serial.println("Setting up LED");
//   pinMode(LED_BUILTIN,OUTPUT);
//   digitalWrite(LED_BUILTIN, HIGH);

//   Serial.println("Setting up PWM");
//   ledcSetup(pwmChannel, pwmFreq, pwmResolution);
//   ledcAttachPin(pwmPin, pwmChannel);

//   Serial.println("Finished configuration.");

// }

// void loop() {
//   int dutyCycle = 0;

//   digitalWrite(LED_BUILTIN, LOW);
//   Serial.println("LED OFF\n");
//   delay(2000);

//   digitalWrite(LED_BUILTIN, HIGH);
//   Serial.println("LED ON\n");
//   delay(2000);

//   // for (size_t i = 0; i < 256; i++)
//   // {
//   //   /* code */
//   //   dutyCycle = i;
//   //   ledcWrite(pwmChannel, dutyCycle);
//   //   Serial.println(i);
//   //   delay(50);
    
//   // }
  

// }
// 	// Set any motor plugged in to ID 3
// 	// this INO acts as an auto-provisioner for any motor plugged in
// 	// servo.id_write(id);
// 	// Serial.println("Attempting set to ID "+String (id));
// 	// int read=servo.id_read();
// 	// if(read!=id || read==0){
// 	// 	Serial.println("\r\nERROR ID set failed");
// 	// 	delay(500);
// 	// }else{
// 	// 	Serial.println("\r\nCurrent ID is now "+String(read));
// 	// }
// 	// delay(200);


// // Shit that works
// #include <Arduino.h>
// #include <WiFi.h>
// #include <ESP32Servo.h>

// // const declarations
// const int lowestPin = 1;
// const int highestPin = 38;
// const char* ssid = "mignet";
// const char* password = "0824664494";

// // put function declarations here:



// // variable declaration
// Servo myservo;  // create servo object to control a servo
// int pos = 0;    // variable to store the servo position
// int servoPin = 5;

// void setup(){
//   // Servo Setup
//   // Allow allocation of all timers
// //   ESP32PWM::allocateTimer(0);
// //   ESP32PWM::allocateTimer(1);
// //   ESP32PWM::allocateTimer(2);
// //   ESP32PWM::allocateTimer(3);
// //   myservo.setPeriodHertz(50);    // standard 50 hz servo
// //   myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
//   // using default min/max of 1000us and 2000us
//   // different servos may require different min/max settings
//   // for an accurate 0 to 180 sweep
  

//   // put your setup code here, to run once:
//   pinMode(LED_BUILTIN,OUTPUT);
//   digitalWrite(LED_BUILTIN, HIGH);

//   Serial.begin(115200);
//   delay(1000);

//   // Wifi Setup
//   int count = 0;

//   WiFi.mode(WIFI_STA); //Optional
//   WiFi.begin(ssid, password);
//   Serial.println("\nConnecting");

//   while(WiFi.status() != WL_CONNECTED){
//       Serial.print(".\n");
//       delay(1000);
//       count++;
//       if (count > 10){
//         break;
//       }
//   }

//   if (WiFi.status() == WL_CONNECTED){
//     Serial.println("\nConnected to the WiFi network");
//     Serial.print("Local ESP32 IP: ");
//     Serial.println(WiFi.localIP());
//   }
//   else {
//     Serial.println("\nFailed to connect.");
//   }

    
// }

// void loop(){
//   // put your main code here, to run repeatedly:
//   digitalWrite(LED_BUILTIN, LOW);
//   Serial.println("LED OFF\n");
//   delay(500);

//   digitalWrite(LED_BUILTIN, HIGH);
//   Serial.println("LED ON\n");
//   delay(500);

//   for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
// 		// in steps of 1 degree
// 		myservo.write(pos);    // tell servo to go to position in variable 'pos'
// 		delay(15);             // waits 15ms for the servo to reach the position
// 	}
// 	for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
// 		myservo.write(pos);    // tell servo to go to position in variable 'pos'
// 		delay(15);             // waits 15ms for the servo to reach the position
// 	}

// }


