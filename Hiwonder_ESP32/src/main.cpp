// Single Pin mode
#include <Arduino.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(&servoBus, 1);

void setup() {
	Serial.begin(115200);
	Serial.println("Beginning Servo Example");
	servoBus.beginOnePinMode(&Serial2,33);
	servoBus.debug(true);
	servoBus.retry=0;

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
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


