#include <Arduino.h>
#include <lx16a-servo.h>
#include <WiFi.h>
#include <iostream>
#include <string>

// ROS Includes
#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS msg includes
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence.h>


#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// =====================================================================================================
// DEFINES
// =====================================================================================================
#define JOINT_DOUBLE_LEN 20

// // Servos and Servo Bus Instantiation
LX16ABus servoBus;
LX16AServo servo1(&servoBus, 1); // 0-16800 (0-168 degrees)
LX16AServo servo2(&servoBus, 2); // 0-24000 (0-240 degrees)
LX16AServo servo3(&servoBus, 3); // 0-24000 (0-240 degrees)
LX16AServo servo4(&servoBus, 4); // 0-24000 (0-240 degrees)
LX16AServo servo5(&servoBus, 5); // 0-24000 (0-240 degrees)
LX16AServo servo6(&servoBus, 6); // 0-24000 (0-240 degrees)

// // Create string sequence
rosidl_runtime_c__String__Sequence name_sequence;

// Array to populate JointState message for Servo positions
double pos[6] = {0, 0, 0, 0, 0, 0};
double effort_array[] = {0, 0, 0, 0, 0, 0};
double vel[] = {0, 0, 0, 0, 0, 0};

rcl_node_t node_hiwonder;	// Node object
rcl_timer_t timer;			// Timer object

// Publisher and Subscriber objects
rcl_publisher_t 	publisher_servo_pos;
rcl_subscription_t 	subscriber_servo_cmd;

// ROS check definitions
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;


// ROS Error Handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Error loop (will flash LED if error)
void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(50);
  }
}


// ROS messages
sensor_msgs__msg__JointState servo_position_array_msg;
std_msgs__msg__Int32 msg;


// =====================================================================================================
// CALLBACKS
// =====================================================================================================

// Function Prototypes


// Callbacks
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_BUILTIN, (msg->data == 0) ? LOW : HIGH);  
}

// void servo_callback() {
// 	str_msg.data = hello;
// 	chatter.publish( &str_msg );

// 	int servo_num = cmd_msg.data[0];
// 	int servo_move_pos = cmd_msg.data[1];

// 	if (servo_num == 1) {
// 		servo1.move_time(servo_move_pos, 1500);
// 	}
// 	else if (servo_num == 2) {
// 		servo2.move_time(servo_move_pos, 1500);
// 	}
// 	else if (servo_num == 3) {
// 		servo3.move_time(servo_move_pos, 1500);
// 	}
// 	else if (servo_num == 4) {
// 		servo4.move_time(servo_move_pos, 1500);
// 	}
// 	else if (servo_num == 5) {
// 		servo5.move_time(servo_move_pos, 1500);
// 	}
// 	else if(servo_num == 6) {
// 		servo6.move_time(servo_move_pos, 1500);
// 	}
// 	else {
// 		digitalWrite(LED_BUILTIN, LOW);
// 		delay(3000);
// 		digitalWrite(LED_BUILTIN,HIGH);
// 	}

// 	// servo6.move_time(servo_move_pos, 3000);
// 	delay(2000);
// 	// servo6.move_time(24000, 3000);
// 	Serial.println("Executing move command on servo.");
// }


void setup() {
	Serial.begin(115200);
	set_microros_serial_transports(Serial);
	delay(2000);

	// LED on Init
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(1000);
	digitalWrite(LED_BUILTIN, LOW);
	delay(1000);
	digitalWrite(LED_BUILTIN, HIGH);

	// =====================================================================================================
    // SERVO SETUP
    // =====================================================================================================

	Serial.println("Beginning Servo Example");
	servoBus.beginOnePinMode(&Serial2,33);
	servoBus.debug(true);
	servoBus.retry=1;

	// Reset the servo positions
	servo1.move_time(2000,2000);
	servo2.move_time(12000,2000);
	servo3.move_time(12000,2000);
	servo4.move_time(12000,2000);
	servo5.move_time(12000,2000);
	servo6.move_time(12000,2000);
	delay(2000);

	// Read in servo positions
	pos[0] = servo1.pos_read();
	pos[1] = servo2.pos_read();
	pos[2] = servo3.pos_read();
	pos[3] = servo4.pos_read();
	pos[4] = servo5.pos_read();
	pos[5] = servo6.pos_read();
	delay(2000);


	// =====================================================================================================
    // ROS SETUP
    // =====================================================================================================

	allocator = rcl_get_default_allocator();									// Initialize micro-ROS allocator
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));					// Initialize support options
	
	// create node
  	RCCHECK(rclc_node_init_default(&node_hiwonder, "Hiwonder_xArm", "", &support));


	// =====================================================================================================
    // PUBLISHERS
    // =====================================================================================================

	// Servo position publisher
	RCCHECK(rclc_publisher_init_default(&publisher_servo_pos, 
	&node_hiwonder, 
	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
	"servo_pos_publisher"));



	// =====================================================================================================
    // SUBSCRIBERS
    // =====================================================================================================

	// RCCHECK(rclc_subscription_init_default(&subscriber_servo_cmd, 
	// &node_hiwonder, 
	// ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), 
	// "servo_cmd_subscriber"));
	RCCHECK(rclc_subscription_init_default(
    &subscriber_servo_cmd,
    &node_hiwonder,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

	// Subscriber Inits

	// Timer Inits
	// const 

	// // create executor (for timers)
	// RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	// RCCHECK(rclc_executor_add_timer(&executor, &timer));


	// =====================================================================================================
    // MESSAGE CONSTRUCTION
    // =====================================================================================================

	// (1) servo_position_array_msg
	// Assigning dynamic memory to the name string sequence (this is not working)
  	// servo_position_array_msg.name.capacity = 6;
	// servo_position_array_msg.name.size = 6;
	// servo_position_array_msg.name.data = (rosidl_runtime_c__String*) malloc(servo_position_array_msg.position.capacity * sizeof(rosidl_runtime_c__String));

	bool success = rosidl_runtime_c__String__Sequence__init(&name_sequence, 6);
	servo_position_array_msg.name = name_sequence;
	success = rosidl_runtime_c__String__assignn(&servo_position_array_msg.name.data[0], "Servo1", 6);
	success = rosidl_runtime_c__String__assignn(&servo_position_array_msg.name.data[1], "Servo2", 6);
	success = rosidl_runtime_c__String__assignn(&servo_position_array_msg.name.data[2], "Servo3", 6);
	success = rosidl_runtime_c__String__assignn(&servo_position_array_msg.name.data[3], "Servo4", 6);
	success = rosidl_runtime_c__String__assignn(&servo_position_array_msg.name.data[4], "Servo5", 6);
	success = rosidl_runtime_c__String__assignn(&servo_position_array_msg.name.data[5], "Servo6", 6);


	// Assigning dynamic memory to POSITION rosidl_runtime_c__double__Sequence 
	servo_position_array_msg.position.capacity = 6;
	servo_position_array_msg.position.size = 6;
	servo_position_array_msg.position.data = (double*) malloc(servo_position_array_msg.position.capacity * sizeof(double));
	servo_position_array_msg.position.data = pos; 

	// Assigning dynamic memory to EFFORT rosidl_runtime_c__double__Sequence 
	servo_position_array_msg.effort.capacity = 6;
	servo_position_array_msg.effort.size = 6;
	servo_position_array_msg.effort.data = (double*) malloc(servo_position_array_msg.position.capacity * sizeof(double));
	servo_position_array_msg.effort.data = effort_array;

	// Assigning dynamic memory to EFFORT rosidl_runtime_c__double__Sequence 
	servo_position_array_msg.velocity.capacity = 6;
	servo_position_array_msg.velocity.size = 6;
	servo_position_array_msg.velocity.data = (double*) malloc(servo_position_array_msg.position.capacity * sizeof(double));
	servo_position_array_msg.velocity.data = effort_array;


	// header
	// Assigning dynamic memory to the frame_id char sequence
	servo_position_array_msg.header.frame_id.capacity = 100;
	servo_position_array_msg.header.frame_id.data = (char*) malloc(servo_position_array_msg.header.frame_id.capacity * sizeof(char));
	servo_position_array_msg.header.frame_id.size = 0;

	// Assigning value to the frame_id char sequence
	strcpy(servo_position_array_msg.header.frame_id.data, "Hiwonder Arm");
	servo_position_array_msg.header.frame_id.size = strlen(servo_position_array_msg.header.frame_id.data);

	// Assigning time stamp
	servo_position_array_msg.header.stamp.sec = (uint16_t)(rmw_uros_epoch_millis()/1000);
	servo_position_array_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();
	
	// create executor
  	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_servo_cmd, &msg, &subscription_callback, ON_NEW_DATA));

	// msg.data = 15;
}

	
void loop() {	

	pos[0] = servo1.pos_read();
	pos[1] = servo2.pos_read();
	pos[2] = servo3.pos_read();
	pos[3] = servo4.pos_read();
	pos[4] = servo5.pos_read();
	pos[5] = servo6.pos_read();

	// pos[0]++;
	// pos[1]++;
	// pos[2]++;
	// pos[3]++;
	// pos[4]++;
	// pos[5]++;

	// Update servo_position_array_msg
	servo_position_array_msg.position.data = pos;
	servo_position_array_msg.header.stamp.sec = (uint16_t)(rmw_uros_epoch_millis()/1000); 	// timestamp
	servo_position_array_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();		// timestamp

	// servo_position_array_msg.name.size = 6;
	// servo_position_array_msg.name.data = (char *) name;
	// servo_position_array_msg.position.data = pos;

	delay(100);
	RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  	RCSOFTCHECK(rcl_publish(&publisher_servo_pos, &servo_position_array_msg, NULL));

	// delay(1000);
  	// RCSOFTCHECK(rcl_publish(&publisher_servo_pos, &msg, NULL));
}







// // Micro-ROS test
// #include <Arduino.h>
// #include <micro_ros_platformio.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// // Error handle loop
// void error_loop() {
//   while(1) {
//     digitalWrite(2, !digitalRead(2));
//     delay(100);
//   }
// }

// // void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
// //   RCLC_UNUSED(last_call_time);
// //   if (timer != NULL) {
// //     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
// //     msg.data++;
// //   }
// // }

// void setup() {
//   // Configure serial transport
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);
//   delay(2000);

//   allocator = rcl_get_default_allocator();

//   //create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

//   // create publisher
//   RCCHECK(rclc_publisher_init_default(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//     "micro_ros_platformio_node_publisher"));

//   // create timer,
// //   const unsigned int timer_timeout = 1000;
// //   RCCHECK(rclc_timer_init_default(
// //     &timer,
// //     &support,
// //     RCL_MS_TO_NS(timer_timeout),
// //     timer_callback));

//   // create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
// //   RCCHECK(rclc_executor_add_timer(&executor, &timer));

//   msg.data = 15;
// }

// void loop() {
//   delay(1000);
//   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

// //   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// }