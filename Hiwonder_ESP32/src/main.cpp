// Single Pin mode
#include <Arduino.h>
#include <lx16a-servo.h>

#include <WiFi.h>
#include <iostream>
#include <string>

// ros includes
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>


// Servos and Servo Bus Instantiation
LX16ABus servoBus;
LX16AServo servo1(&servoBus, 1); // 0-16800 (0-168 degrees)
LX16AServo servo2(&servoBus, 2); // 0-24000 (0-240 degrees)
LX16AServo servo3(&servoBus, 3); // 0-24000 (0-240 degrees)
LX16AServo servo4(&servoBus, 4); // 0-24000 (0-240 degrees)
LX16AServo servo5(&servoBus, 5); // 0-24000 (0-240 degrees)
LX16AServo servo6(&servoBus, 6); // 0-24000 (0-240 degrees)

// Servo position variable
// int pos1 = 0;
// int pos2 = 12000;
// int pos3 = 12000;
// int pos4 = 12000;
// int pos5 = 12000;
// int pos6 = 12000;

// Wifi const
const char* ssid = "Rogachino";
const char* password = "12345678";

// const char* servo_names[6] = {"Servo 1", "Servo 2", "Servo 3", "Servo 4", " Servo 5", "Servo 6"};

bool LED_status = true;


char hello[13] = "hello world!";


// ROS node handler
ros::NodeHandle nh;

std_msgs::String str_msg;
std_msgs::UInt16 servo1_pos_msg;
sensor_msgs::JointState servo_position_array_msg;
// std_msgs::Float32MultiArray float32_array_msg;

// servo_position_array_msg Jointstate message components 
char *name[] = {"Servo_1", "Servo_2", "Servo_3", "Servo_4", "Servo_5", "Servo_6"};

float pos[] = {0, 0, 0, 0, 0, 0};


uint16_t pos1_test = 123; // just for testing

// ROS publishers
ros::Publisher chatter("chatter", &str_msg); 							// to check if serial is working
ros::Publisher servo_pos_pub("servo_positions", &servo_position_array_msg);	// servo positions publisher
// ros::Publisher servo1_pos_pub("servo1_position", &servo1_pos_msg);

// Callbacks
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
	// nh.advertise(servo1_pos_pub);

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
	servo1.move_time(4000,3000);
	servo2.move_time(11000,3000);
	servo3.move_time(13000,3000);
	servo4.move_time(11000,3000);
	servo5.move_time(13000,3000);
	servo6.move_time(11000,3000);
	
	

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

}

void loop() {	


	// int16_t servo_pos_array[6] = {pos1, pos2, pos3, pos4, pos5, pos6};
	

	pos[0] = servo1.vin();
	pos[1] = servo2.vin();
	pos[2] = servo3.vin();
	pos[3] = servo4.vin();
	pos[4] = servo5.vin();
	pos[5] = servo6.vin();

	servo_position_array_msg.name = name;
	servo_position_array_msg.position = pos;
	servo_position_array_msg.name_length = 6;
	servo_position_array_msg.position_length = 6;
	servo_pos_pub.publish(&servo_position_array_msg);

	// servo1_pos_msg.data = servo1.temp();
	// servo1_pos_pub.publish(&servo1_pos_msg); 

	nh.spinOnce();
	delay(10);
	

}

