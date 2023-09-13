// Single Pin mode
#include <Arduino.h>
#include <lx16a-servo.h>

#include <WiFi.h>

// ros includes
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>

// Servos and Servo Bus Instantiation
LX16ABus servoBus;
LX16AServo servo1(&servoBus, 1); // 0-16800 (0-168 degrees)
LX16AServo servo2(&servoBus, 2); // 0-24000 (0-240 degrees)
LX16AServo servo3(&servoBus, 3); // 0-24000 (0-240 degrees)
LX16AServo servo4(&servoBus, 4); // 0-24000 (0-240 degrees)
LX16AServo servo5(&servoBus, 5); // 0-24000 (0-240 degrees)
LX16AServo servo6(&servoBus, 6); // 0-24000 (0-240 degrees)

// Wifi const
const char* ssid = "Rogachino";
const char* password = "12345678";

bool LED_status = true;



char hello[13] = "hello world!";
std_msgs::String str_msg;
std_msgs::Int16MultiArray servo_position_array;


// ROS publishers
ros::Publisher chatter("chatter", &str_msg); 							// to check if serial is working
ros::Publisher servo_pos_pub("servo_positions", &servo_position_array);	// servo positions publisher



void servo_callback(const std_msgs::Int16MultiArray& cmd_msg) {
	str_msg.data = hello;
	chatter.publish( &str_msg );

	int servo_num = cmd_msg.data[0];
	int servo_move_pos = cmd_msg.data[1];

	if (servo_num == 1) {
		servo1.move_time(servo_move_pos, 1500);
	}
	else if (servo_num == 2) {
		servo2.move_time(servo_move_pos, 1500);
	}
	else if (servo_num == 3) {
		servo3.move_time(servo_move_pos, 1500);
	}
	else if (servo_num == 4) {
		servo4.move_time(servo_move_pos, 1500);
	}
	else if (servo_num == 5) {
		servo5.move_time(servo_move_pos, 1500);
	}
	else if(servo_num == 6) {
		servo6.move_time(servo_move_pos, 1500);
	}
	else {
		digitalWrite(LED_BUILTIN, LOW);
		delay(3000);
		digitalWrite(LED_BUILTIN,HIGH);
	}

	// servo6.move_time(servo_move_pos, 3000);
	delay(2000);
	// servo6.move_time(24000, 3000);
	Serial.println("Executing move command on servo.");
}

// ROS node handler
ros::NodeHandle nh;



// ROS Subscribers
// ros::Subscriber<std_msgs::UInt16> sub_servo("servo", servo_callback);
ros::Subscriber<std_msgs::Int16MultiArray> sub_servo("servo", servo_callback);


void setup() {
	Serial.begin(115200);

	// ROS chatter publisher
	nh.initNode();

	// Publishers (advertise)
	nh.advertise(chatter);
	nh.advertise(servo_pos_pub);

	// Subscribers
	nh.subscribe(sub_servo);



	// LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LED_status);

	// // WiFi setup (make this into a header file maybe?)
	// delay(1000);
	// WiFi.mode(WIFI_STA); //Optional
    // WiFi.begin(ssid, password);
    // Serial.println("\nConnecting");

	// while(WiFi.status() != WL_CONNECTED){
	// 	LED_status = !LED_status;
	// 	digitalWrite(LED_BUILTIN, LED_status);
    //     Serial.print(".");
    //     delay(100);
    // }

	// Serial.println("\nConnected to the WiFi network");
    // Serial.print("Local ESP32 IP: ");
    // Serial.println(WiFi.localIP());

	// Servo Setup
	Serial.println("Beginning Servo Example");
	servoBus.beginOnePinMode(&Serial2,33);
	servoBus.debug(true);
	servoBus.retry=1;

	// Reset the servo positions
	servo1.move_time(0,3000);
	servo2.move_time(12000,3000);
	servo3.move_time(12000,3000);
	servo4.move_time(12000,3000);
	servo5.move_time(12000,3000);
	servo6.move_time(12000,3000);
	

	// int pos = 123;
	// pos = servo6.pos_read();
	// Serial.println("The servo 6 position is: ");
	// Serial.print(pos);

	// delay(6000);
	// int id = 0;
	// int temp = 0;
	// int volt = 0;

	// id = servo6.id_read(6);
	// temp = servo6.temp();
	// volt = servo6.vin();

	// Serial.println("ID is: ");
	// Serial.println(String(id));
	// Serial.println("Servo temperature is: ");
	// Serial.println(String(temp));
	// Serial.println("Servo V_in is: ");
	// Serial.println(String(volt));

	// Doesnt work ATM
	// servo.readLimits();
}

void loop() {
	
	nh.spinOnce();
	delay(1);
	

}

