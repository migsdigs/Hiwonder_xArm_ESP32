#include <Arduino.h>
#include <lx16a-servo.h>
#include <WiFi.h>
#include <iostream>
#include <string>

// ROS Includes
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS msg includes
#include <std_msgs/msg/int64_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
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
// GLOBALS
// =====================================================================================================

// Servos and Servo Bus Instantiation
LX16ABus servoBus;
LX16AServo servo1(&servoBus, 1); // 0-16800 (0-168 degrees)
LX16AServo servo2(&servoBus, 2); // 0-24000 (0-240 degrees)
LX16AServo servo3(&servoBus, 3); // 0-24000 (0-240 degrees)
LX16AServo servo4(&servoBus, 4); // 0-24000 (0-240 degrees)
LX16AServo servo5(&servoBus, 5); // 0-24000 (0-240 degrees)
LX16AServo servo6(&servoBus, 6); // 0-24000 (0-240 degrees)

// Create string sequence
rosidl_runtime_c__String__Sequence name__string_sequence;

// // Array to hold latest read servo parameters
int16_t vin[6] = {0, 0, 0, 0, 0, 0};				// servo voltages
int16_t temp[6] = {0, 0 ,0 ,0 ,0 ,0}; 			// servo temperatures

int16_t v_in_cached[6] = {0,0,0,0,0,0};
int16_t temp_cached[6] = {0,0,0,0,0,0};

// Array to populate JointState message for Servo positions
double pos[6] = {0, 0, 0, 0, 0, 0};				// servo positions
double effort_array[] = {0, 0, 0, 0, 0, 0};		// effort array (just needed to populate message component)
double vel[] = {0, 0, 0, 0, 0, 0};				// velocity array "  "

// Timer settings
static const uint16_t timer_divider = 80;
static const uint64_t timer_max_count = 20000;

static const uint16_t timer_divider_2 = 80;
static const uint64_t timer_max_count_2 = 5000000;

static hw_timer_t *timer = NULL;		// timer for publishing servos pos.
static hw_timer_t *timer2 = NULL;		// timer for publishing servos volt & temps

bool publish_servo_pos_flag = false;
bool publish_temp_and_volt_flag = true;


// =====================================================================================================
// ROS ERROR HANDLING
// =====================================================================================================
#define ROS_LED_PIN 4
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

bool micro_ros_init_successful;
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


bool error_loop(){
	return false;
}


rcl_node_t node_hiwonder;	// Node object

// Publisher and Subscriber objects
rcl_publisher_t 	publisher;
rcl_publisher_t 	publisher_servo_pos;
rcl_publisher_t		publisher_servo_vin;
rcl_publisher_t 	publisher_servo_temp;

rcl_subscription_t 	subscriber;
rcl_subscription_t 	subscriber_one_servo_cmd;
rcl_subscription_t 	subscriber_multi_servo_cmd;

// ROS check definitions
rclc_support_t 	support;
rcl_allocator_t allocator;

rcl_service_t 	service;
rcl_wait_set_t 	wait_set;


// Executors
rclc_executor_t executor;
rclc_executor_t executor_subscriber;
rclc_executor_t executor_servo_pos_publish;
rclc_executor_t executor_servo_vin_publish;
rclc_executor_t executor_servo_temp_publish;
rclc_executor_t executor_single_servo_cmd_sub;
rclc_executor_t executor_multi_servo_move_cmd_sub;


// ROS messages
sensor_msgs__msg__JointState servo_position_array_msg;
std_msgs__msg__Int32 msg;

std_msgs__msg__Int64MultiArray single_servo_cmd_msg_in;
std_msgs__msg__Int16MultiArray multi_servo_cmd_msg_in;

std_msgs__msg__Int16MultiArray servo_voltages_msg;
std_msgs__msg__Int16MultiArray servo_temps_msg;


// =====================================================================================================
// INTERUPT SERVICE ROUTINES
// =====================================================================================================

// Timer for changing the servo pos. publish flag
void IRAM_ATTR onTimer() {
	// Toggle LED
	int pin_state = digitalRead(LED_BUILTIN);
	digitalWrite(LED_BUILTIN, !pin_state); 

	publish_servo_pos_flag = true;
}

// Timer for changing the servo temp and volt publish flag
void IRAM_ATTR onTimer2() {
	// Toggle LED
	int pin_state = digitalRead(22);
	digitalWrite(22, !pin_state);

	publish_temp_and_volt_flag = true;
}

// =====================================================================================================
// CALLBACKS
// =====================================================================================================

// // Subscriber callback for commanding single servo
// void subscription_callback_single_servo(const void * msgin) {  
// 	// digitalWrite(LED_BUILTIN, LOW);
// 	// delay(200);
// 	// digitalWrite(LED_BUILTIN,HIGH);
// 	// delay(200);
// 	// digitalWrite(LED_BUILTIN, LOW);
// 	// delay(200);
// 	// digitalWrite(LED_BUILTIN,HIGH);

// 	const std_msgs__msg__Int64MultiArray * one_servo = (const std_msgs__msg__Int64MultiArray *)msgin;

// 	// TODO: Add condition that servo pos is positive (i.e. dont do anything if it negative)
// 	if (one_servo->data.data[0] == 1) {
// 		servo1.move_time(one_servo->data.data[1], one_servo->data.data[2]);
// 	}
// 	else if (one_servo->data.data[0] == 2) {
// 		servo2.move_time(one_servo->data.data[1], one_servo->data.data[2]);
// 	}
// 	else if (one_servo->data.data[0] == 3) {
// 		servo3.move_time(one_servo->data.data[1], one_servo->data.data[2]);
// 	}
// 	else if (one_servo->data.data[0] == 4) {
// 		servo4.move_time(one_servo->data.data[1], one_servo->data.data[2]);
// 	}
// 	else if (one_servo->data.data[0] == 5) {
// 		servo5.move_time(one_servo->data.data[1], one_servo->data.data[2]);
// 	}
// 	else if(one_servo->data.data[0] == 6) {
// 		servo6.move_time(one_servo->data.data[1], one_servo->data.data[2]);
// 	}
// 	else {
// 		digitalWrite(LED_BUILTIN, LOW);
// 		delay(200);
// 		digitalWrite(LED_BUILTIN,HIGH);
// 		delay(200);
// 		digitalWrite(LED_BUILTIN, LOW);
// 		delay(200);
// 		digitalWrite(LED_BUILTIN,HIGH);
// 	}
// }

// Subscriber callback for commanding multi servo
void subscription_callback_multi_servo(const void * msgin) {
	// Move all the servos (dont move if argument is negative)
	const std_msgs__msg__Int16MultiArray * multi_servo = (const std_msgs__msg__Int16MultiArray *)msgin;

	if (multi_servo->data.data[0] >= 0) {
		servo1.move_time(multi_servo->data.data[0], multi_servo->data.data[6]);
	}

	if (multi_servo->data.data[1] >= 0) {
		servo2.move_time(multi_servo->data.data[1], multi_servo->data.data[7]);
	}

	if (multi_servo->data.data[2] >= 0) {
		servo3.move_time(multi_servo->data.data[2], multi_servo->data.data[8]);
	}
	
	if (multi_servo->data.data[3] >= 0) {
		servo4.move_time(multi_servo->data.data[3], multi_servo->data.data[9]);
	}

	if (multi_servo->data.data[4] >= 0) {
		servo5.move_time(multi_servo->data.data[4], multi_servo->data.data[10]);
	}

	if (multi_servo->data.data[5] >= 0) {
		servo6.move_time(multi_servo->data.data[5], multi_servo->data.data[11]);
	}

	// digitalWrite(LED_BUILTIN, LOW);
	// delay(200);
	// digitalWrite(LED_BUILTIN,HIGH);
	// delay(200);
	// digitalWrite(LED_BUILTIN, LOW);
	// delay(200);
	// digitalWrite(LED_BUILTIN,HIGH);
}



bool create_entities(){
	
	// =====================================================================================================
	// ROS SETUP
	// =====================================================================================================

	allocator = rcl_get_default_allocator();			// Initialize micro-ROS allocator
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));					// Initialize support options
  	RCCHECK(rclc_node_init_default(&node_hiwonder, "Hiwonder_xArm_node", "", &support)); // create node

	// =====================================================================================================
    // PUBLISHERS
    // =====================================================================================================

	// Servo position publisher (publishes at a rate of X Hz)
	RCCHECK(rclc_publisher_init_default(
		&publisher_servo_pos, 
		&node_hiwonder, 
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
		"servo_pos_publisher"));

	// Servo voltage publisher (publishes every 5 seconds)
	RCCHECK(rclc_publisher_init_default(
		&publisher_servo_vin, 
		&node_hiwonder, 
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), 
		"servo_volt_publisher"));

	// Servo temperature publisher (publishes every 5 seconds)
	RCCHECK(rclc_publisher_init_default(
		&publisher_servo_temp, 
		&node_hiwonder, 
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), 
		"servo_temp_publisher"));

	// =====================================================================================================
    // SUBSCRIBERS
    // =====================================================================================================

	// Subscriber for moving multiple servos
	RCCHECK(rclc_subscription_init_default(
		&subscriber_multi_servo_cmd, 
		&node_hiwonder, 
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), 
		"multi_servo_cmd_sub"));
	
	// =====================================================================================================
    // EXECUTORS
    // =====================================================================================================

	// init executors
	RCCHECK(rclc_executor_init(&executor_servo_pos_publish, &support.context, 1, &allocator));			// executor for publishing servo pos.
	RCCHECK(rclc_executor_init(&executor_servo_vin_publish, &support.context, 1, &allocator));			// executor for publishing servo pos.
	RCCHECK(rclc_executor_init(&executor_servo_temp_publish, &support.context, 1, &allocator));			// executor for publishing servo pos.
	// RCCHECK(rclc_executor_init(&executor_single_servo_cmd_sub, &support.context, 1, &allocator));		// executor for one servo cmd subscriber
	RCCHECK(rclc_executor_init(&executor_multi_servo_move_cmd_sub, &support.context, 1, &allocator));	// executor for multi servo cmd subscriber

	// executor additions
	// RCCHECK(rclc_executor_add_subscription(&executor_single_servo_cmd_sub, &subscriber_one_servo_cmd, &single_servo_cmd_msg_in, &subscription_callback_single_servo, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor_multi_servo_move_cmd_sub, &subscriber_multi_servo_cmd, &multi_servo_cmd_msg_in, &subscription_callback_multi_servo, ON_NEW_DATA));

	return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher_servo_pos, &node_hiwonder);
  rcl_publisher_fini(&publisher_servo_vin, &node_hiwonder);
  rcl_publisher_fini(&publisher_servo_temp, &node_hiwonder);

  rcl_subscription_fini(&subscriber_multi_servo_cmd, &node_hiwonder);

  rclc_executor_fini(&executor_servo_pos_publish);
  rclc_executor_fini(&executor_servo_vin_publish);
  rclc_executor_fini(&executor_servo_temp_publish);
  rclc_executor_fini(&executor_multi_servo_move_cmd_sub);

  rcl_node_fini(&node_hiwonder);
  rclc_support_fini(&support);
}

// =====================================================================================================
// SETUP FUNCTION
// =====================================================================================================

void setup() {
	Serial.begin(115200);
	set_microros_serial_transports(Serial);
	delay(2000);


	// LED on Init
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(250);
	digitalWrite(LED_BUILTIN, LOW);
	delay(250);
	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(LED_BUILTIN, LOW);
	delay(250);
	digitalWrite(LED_BUILTIN, HIGH);

	pinMode(22, OUTPUT);
	digitalWrite(22, HIGH);
	delay(250);
	digitalWrite(22, LOW);
	delay(250);
	digitalWrite(22, HIGH);
	delay(250);
	digitalWrite(22, LOW);
	delay(250);
	digitalWrite(22, HIGH);

	pinMode(4, OUTPUT);

	// =====================================================================================================
    // SERVO SETUP
    // =====================================================================================================

	Serial.println("Beginning Servo Example");
	servoBus.beginOnePinMode(&Serial2,33);
	servoBus.debug(true);
	servoBus.retry=3;

	// Reset the servo positions
	servo1.move_time(16000,1000);
	servo2.move_time(12000,1250);
	servo3.move_time(12000,1500);
	servo4.move_time(12000,2000);
	servo5.move_time(12000,2000);
	servo6.move_time(12000,2000);
	delay(2000);

	servo1.move_time(2000,500);
	delay(500);
	servo1.move_time(16000,500);
	delay(500);
	servo1.move_time(2000,500);
	delay(500);

	// Read in servo positions
	pos[0] = servo1.pos_read();
	pos[1] = servo2.pos_read();
	pos[2] = servo3.pos_read();
	pos[3] = servo4.pos_read();
	pos[4] = servo5.pos_read();
	pos[5] = servo6.pos_read();
	delay(1000);

	// Read in servo voltages
	v_in_cached[0] = servo1.vin();
	v_in_cached[1] = servo2.vin();
	v_in_cached[2] = servo3.vin();
	v_in_cached[3] = servo4.vin();
	v_in_cached[4] = servo5.vin();
	v_in_cached[5] = servo6.vin();
	delay(1000);

	// Read in servo temperatures
	temp_cached[0] = servo1.temp();
	temp_cached[1] = servo2.temp();
	temp_cached[2] = servo3.temp();
	temp_cached[3] = servo4.temp();
	temp_cached[4] = servo5.temp();
	temp_cached[5] = servo6.temp();
	delay(1000);

	delay(1000);

	// Declare ros-agent state as waiting;
	state = WAITING_AGENT;


	// =====================================================================================================
    // TIMER SETUP
    // =====================================================================================================

	// Timer 1. (publish position at faster rate)
	// Create and start timer (num, divider, countUp)
	timer = timerBegin(0, timer_divider, true);

	// Provide ISR to timer (timer, function, edge)
	timerAttachInterrupt(timer, &onTimer, true);

	// At what count should ISR trigger (timer, count, autoreload)
	timerAlarmWrite(timer, timer_max_count, true);

	// Allow ISR to trigger
	timerAlarmEnable(timer);


	// Timer 2. (publish v_in and temp every +- 5 seconds)
	// Create and start timer (num, divider, countUp)
	timer2 = timerBegin(1, timer_divider_2, true);

	// Provide ISR to timer (timer, function, edge)
	timerAttachInterrupt(timer2, &onTimer2, true);

	// At what count should ISR trigger (timer, count, autoreload)
	timerAlarmWrite(timer2, timer_max_count_2, true);

	// Allow ISR to trigger
	timerAlarmEnable(timer2);
	

	// // =====================================================================================================
    // // SERVICES
    // // =====================================================================================================


	// =====================================================================================================
    // TIMERS
    // =====================================================================================================


	// =====================================================================================================
    // MESSAGE CONSTRUCTION
    // =====================================================================================================

	// (1) servo_position_array_msg
	// Assigning dynamic memory to the name string sequence 

	bool success = rosidl_runtime_c__String__Sequence__init(&name__string_sequence, 6);
	servo_position_array_msg.name = name__string_sequence;
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
	servo_position_array_msg.velocity.data = vel;

	// header
	// Assigning dynamic memory to the frame_id char sequence
	servo_position_array_msg.header.frame_id.capacity = 100;
	servo_position_array_msg.header.frame_id.data = (char*) malloc(servo_position_array_msg.header.frame_id.capacity * sizeof(char));
	servo_position_array_msg.header.frame_id.size = 0;

	// Assigning value to the frame_id char sequence
	strcpy(servo_position_array_msg.header.frame_id.data, "Hiwonder xArm");
	servo_position_array_msg.header.frame_id.size = strlen(servo_position_array_msg.header.frame_id.data);

	// Assigning time stamp
	servo_position_array_msg.header.stamp.sec = (uint16_t)(rmw_uros_epoch_millis()/1000);
	servo_position_array_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();
	

	// // (2) one_servo_cmd_msg
	// single_servo_cmd_msg_in.data.capacity = 3; 
  	// single_servo_cmd_msg_in.data.size = 3;
  	// single_servo_cmd_msg_in.data.data = (int64_t*) malloc(3 * sizeof(int64_t));

	// single_servo_cmd_msg_in.layout.dim.capacity = 3;
	// single_servo_cmd_msg_in.layout.dim.size = 3;
	// single_servo_cmd_msg_in.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(3 * sizeof(std_msgs__msg__MultiArrayDimension));

	// for(size_t i = 0; i < single_servo_cmd_msg_in.layout.dim.capacity; i++){
	// 	single_servo_cmd_msg_in.layout.dim.data[i].label.capacity = 10;
	// 	single_servo_cmd_msg_in.layout.dim.data[i].label.size = 10;
	// 	single_servo_cmd_msg_in.layout.dim.data[i].label.data = (char*) malloc(single_servo_cmd_msg_in.layout.dim.data[i].label.capacity * sizeof(char));
	// }


	// (3) multi_servo_cmd_msg
	multi_servo_cmd_msg_in.data.capacity = 12;
	multi_servo_cmd_msg_in.data.size = 0;
  	multi_servo_cmd_msg_in.data.data = (int16_t*) malloc(single_servo_cmd_msg_in.data.capacity * sizeof(int16_t));

	multi_servo_cmd_msg_in.layout.dim.capacity = 3;
	multi_servo_cmd_msg_in.layout.dim.size = 0;
	multi_servo_cmd_msg_in.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(multi_servo_cmd_msg_in.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

	for(size_t i = 0; i < multi_servo_cmd_msg_in.layout.dim.capacity; i++){
		multi_servo_cmd_msg_in.layout.dim.data[i].label.capacity = 10;
		multi_servo_cmd_msg_in.layout.dim.data[i].label.size = 0;
		multi_servo_cmd_msg_in.layout.dim.data[i].label.data = (char*) malloc(multi_servo_cmd_msg_in.layout.dim.data[i].label.capacity * sizeof(char));
	}


	// (4) servo_voltages_msg
	servo_voltages_msg.data.capacity = 6;
	servo_voltages_msg.data.size = 6;
	servo_voltages_msg.data.data = (int16_t*) malloc(servo_voltages_msg.data.capacity * sizeof(int16_t));

	servo_voltages_msg.layout.dim.capacity = 3;
	servo_voltages_msg.layout.dim.size = 0;
	servo_voltages_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(servo_voltages_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

	for(size_t i = 0; i < servo_voltages_msg.layout.dim.capacity; i++){
		servo_voltages_msg.layout.dim.data[i].label.capacity = 10;
		servo_voltages_msg.layout.dim.data[i].label.size = 0;
		servo_voltages_msg.layout.dim.data[i].label.data = (char*) malloc(servo_voltages_msg.layout.dim.data[i].label.capacity * sizeof(char));
	}

	// (5) servo_temps_msg
	servo_temps_msg.data.capacity = 6;
	servo_temps_msg.data.size = 6;
	servo_temps_msg.data.data = (int16_t*) malloc(servo_temps_msg.data.capacity * sizeof(int16_t));

	servo_temps_msg.layout.dim.capacity = 3;
	servo_temps_msg.layout.dim.size = 0;
	servo_temps_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(servo_temps_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

	for(size_t i = 0; i < servo_temps_msg.layout.dim.capacity; i++){
		servo_temps_msg.layout.dim.data[i].label.capacity = 10;
		servo_temps_msg.layout.dim.data[i].label.size = 0;
		servo_temps_msg.layout.dim.data[i].label.data = (char*) malloc(servo_temps_msg.layout.dim.data[i].label.capacity * sizeof(char));
	}

}



// =====================================================================================================
// LOOP
// =====================================================================================================

	
void loop() {	
	
	RCSOFTCHECK(rclc_executor_spin_some(&executor_multi_servo_move_cmd_sub, RCL_MS_TO_NS(100)));

	switch(state) {
		case WAITING_AGENT:
			//check for agent connection
			state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            break;
		
		case AGENT_AVAILABLE:
			// Create micro-ROS entities
			state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
			if (state == WAITING_AGENT) {
				// Creation failed, release allocated resources
				destroy_entities();
			};
			break;
		
		case AGENT_CONNECTED:
            // Check connection and spin on success
            state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            if (state == AGENT_CONNECTED)
            {
				if (publish_servo_pos_flag == true) {
					pos[0] = servo1.pos_read();
					pos[1] = servo2.pos_read();
					pos[2] = servo3.pos_read();
					pos[3] = servo4.pos_read();
					pos[4] = servo5.pos_read();
					pos[5] = servo6.pos_read();
					delay(10);

					effort_array[0]++;


					// Update servo_position_array_msg
					servo_position_array_msg.position.data = pos;
					servo_position_array_msg.effort.data = effort_array;
					servo_position_array_msg.velocity.data = vel;
					servo_position_array_msg.header.stamp.sec = (uint16_t)(rmw_uros_epoch_millis()/1000); 	// timestamp
					servo_position_array_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();		// timestamp

					
					// Publishes
					RCSOFTCHECK(rcl_publish(&publisher_servo_pos, &servo_position_array_msg, NULL));
					publish_servo_pos_flag = false;
				}

				if (publish_temp_and_volt_flag == true) {
					// Read servo voltages
					vin[0] = servo1.vin();
					vin[1] = servo2.vin();
					vin[2] = servo3.vin();
					vin[3] = servo4.vin();
					vin[4] = servo5.vin();
					vin[5] = servo6.vin();
					delay(10);

					if (vin[0] != 0) v_in_cached[0] = vin[0];
					if (vin[1] != 0) v_in_cached[1] = vin[1];
					if (vin[2] != 0) v_in_cached[2] = vin[2];
					if (vin[3] != 0) v_in_cached[3] = vin[3];
					if (vin[4] != 0) v_in_cached[4] = vin[4];
					if (vin[5] != 0) v_in_cached[5] = vin[5];

					// Give msg new data
					servo_voltages_msg.data.data = v_in_cached;
					

					// Read servo temps
					temp[0] = servo1.temp();
					temp[1] = servo2.temp();
					temp[2] = servo3.temp();
					temp[3] = servo4.temp();
					temp[4] = servo5.temp();
					temp[5] = servo6.temp();
					delay(10);

					if (temp[0] != 0) temp_cached[0] = temp[0];
					if (temp[1] != 0) temp_cached[1] = temp[1];
					if (temp[2] != 0) temp_cached[2] = temp[2];
					if (temp[3] != 0) temp_cached[3] = temp[3];
					if (temp[4] != 0) temp_cached[4] = temp[4];
					if (temp[5] != 0) temp_cached[5] = temp[5];

					// Give msg new data
					servo_temps_msg.data.data = temp_cached;

					// Publish messages
					RCSOFTCHECK(rcl_publish(&publisher_servo_vin, &servo_voltages_msg, NULL));
					RCSOFTCHECK(rcl_publish(&publisher_servo_temp, &servo_temps_msg, NULL));

					publish_temp_and_volt_flag = false;
				}
                rclc_executor_spin_some(&executor_multi_servo_move_cmd_sub, RCL_MS_TO_NS(100));
            }
            break;

		case AGENT_DISCONNECTED:
            // Connection is lost, destroy entities and go back to first step
            destroy_entities();
            state = WAITING_AGENT;
            break;

        default:
            break;
	}

	if (state == AGENT_CONNECTED) {
    	digitalWrite(ROS_LED_PIN, 1);
  	} else {
    	digitalWrite(ROS_LED_PIN, 0);
  }

}


