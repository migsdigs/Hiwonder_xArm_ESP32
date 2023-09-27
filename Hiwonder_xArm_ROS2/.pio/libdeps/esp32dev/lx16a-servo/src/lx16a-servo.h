#pragma once
#include <Arduino.h>
#define LX16A_BROADCAST_ID 0xFE
#define LX16A_SERVO_MOVE_TIME_WRITE 1
#define LX16A_SERVO_MOVE_TIME_READ 2
#define LX16A_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LX16A_SERVO_MOVE_TIME_WAIT_READ 8
#define LX16A_SERVO_MOVE_START 11
#define LX16A_SERVO_MOVE_STOP 12
#define LX16A_SERVO_ID_WRITE 13
#define LX16A_SERVO_ID_READ 14
#define LX16A_SERVO_ANGLE_OFFSET_ADJUST 17
#define LX16A_SERVO_ANGLE_OFFSET_WRITE 18
#define LX16A_SERVO_ANGLE_OFFSET_READ 19
#define LX16A_SERVO_ANGLE_LIMIT_WRITE 20
#define LX16A_SERVO_ANGLE_LIMIT_READ 21
#define LX16A_SERVO_VIN_LIMIT_WRITE 22
#define LX16A_SERVO_VIN_LIMIT_READ 23
#define LX16A_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LX16A_SERVO_TEMP_MAX_LIMIT_READ 25
#define LX16A_SERVO_TEMP_READ 26
#define LX16A_SERVO_VIN_READ 27
#define LX16A_SERVO_POS_READ 28
#define LX16A_SERVO_OR_MOTOR_MODE_WRITE 29
#define LX16A_SERVO_OR_MOTOR_MODE_READ 30
#define LX16A_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LX16A_SERVO_LOAD_OR_UNLOAD_READ 32
#define LX16A_SERVO_LED_CTRL_WRITE 33
#define LX16A_SERVO_LED_CTRL_READ 34
#define LX16A_SERVO_LED_ERROR_WRITE 35
#define LX16A_SERVO_LED_ERROR_READ 36

class LX16ABus {
private:
	int myTXFlagGPIO = -1;
	int myTXPin=-1;
	int lastCommand=0;
	void setTXFlag(int flag) {
		myTXFlagGPIO = flag;
		if (myTXFlagGPIO >= 0) {
			pinMode(myTXFlagGPIO, OUTPUT);
			digitalWrite(myTXFlagGPIO, 0);
		}
	}

public:

	bool _debug = false;
	bool _deepDebug = false;
	bool singlePinMode=false;
	LX16ABus() {
	}

	// debug enables/disables debug printf's for this servo
	void debug(bool on) {
		_debug = on;

	}
	void beginOnePinMode(HardwareSerial * port, int tXrXpin){
		_port = port;
		myTXPin=tXrXpin;
		singlePinMode=true;

#if defined ARDUINO_ARCH_ESP32
		port->begin(_baud, SERIAL_8N1,myTXPin,myTXPin);
		pinMode(myTXPin, OUTPUT|PULLUP);
#elif defined(CORE_TEENSY)
        pinMode(myTXPin, OUTPUT_OPENDRAIN);
		port->begin(_baud, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
		port->setTX(myTXPin, true);

#endif
		delay(3);
		while (port->available())
			port->read();
	}
	void begin(HardwareSerial * port, int tXpin, int TXFlagGPIO = -1) {
		_port = port;
		myTXPin=tXpin;
#if defined ARDUINO_ARCH_ESP32
        port->begin(_baud);
        pinMode(myTXPin, OUTPUT|PULLUP);
#elif defined(CORE_TEENSY)
        pinMode(myTXPin, OUTPUT_OPENDRAIN);
		port->begin(_baud, SERIAL_8N1);
		port->setTX(myTXPin, true);

#else
		pinMode(myTXPin, OUTPUT);
		port->begin(_baud, SERIAL_8N1);
#endif
		delay(3);
		while (port->available())
			port->read();
		if(TXFlagGPIO>=0)setTXFlag(TXFlagGPIO);

	}


	// time returns the number of ms to TX/RX n characters
	uint32_t time(uint32_t n) {
		return n * 10 * 1000 / _baud; // 10 bits per char
	}
	uint32_t timeus(uint32_t n) {
		return n * 10 * 1000000 / _baud; // 10 bits per char
	}
	// methods passed through to Serial
	bool available() {
		return _port->available();
	}
	int read() {
		return _port->read();
	}
	void write(const uint8_t *buf, int buflen) {

#if defined ARDUINO_ARCH_ESP32
		pinMode(myTXPin, OUTPUT|PULLUP);
#elif defined(CORE_TEENSY)
		_port->setTX(myTXPin, false);
#endif
//		_port->write(buf, buflen);
//		_port->flush();
		if(!singlePinMode){
			delayMicroseconds(10);
			for(int i=0;i<buflen;i++){
				_port->write(buf[i]);
				_port->flush();
			}
		}
		if(singlePinMode){
			_port->write(buf, buflen);
			_port->flush();
		}
#if defined ARDUINO_ARCH_ESP32
		if(singlePinMode)
			pinMode(myTXPin, INPUT|PULLUP);
		else
			pinMode(myTXPin, OUTPUT|PULLUP);
#elif defined(CORE_TEENSY)
		_port->setTX(myTXPin, true);
#endif


	}
	int retry = 3;
	void setRetryCount(int count) {
		retry = count;
	}
	// write a command with the provided parameters
	// returns true if the command was written without conflict onto the bus
	bool write_no_retry(uint8_t cmd, const uint8_t *params, int param_cnt,
			uint8_t MYID);

	// read sends a command to the servo and reads back the response into the params buffer.
	// returns true if everything checks out correctly.
	bool read_no_retry(uint8_t cmd, uint8_t *params, int param_len,
			uint8_t MYID);
	// read sends a command to the servo and reads back the response into the params buffer.
	// returns true if everything checks out correctly.
	bool rcv(uint8_t cmd, uint8_t *params, int param_len,
			uint8_t MYID);
	// write a command with the provided parameters
	// returns true if the command was written without conflict onto the bus
	bool write(uint8_t cmd, const uint8_t *params, int param_cnt,
			uint8_t MYID) {
		if (retry == 0) {
			return write_no_retry(cmd, params, param_cnt, MYID);
		} else {
			for (int i = 0; i < retry; i++) {
				bool ok = write_no_retry(cmd, params, param_cnt, MYID);
				if (ok)
					return true;
			}
		}
		return false;
	}

	// read sends a command to the servo and reads back the response into the params buffer.
	// returns true if everything checks out correctly.
	bool read(uint8_t cmd, uint8_t *params, int param_len, uint8_t MYID) {
		if (retry == 0) {
			return read_no_retry(cmd, params, param_len, MYID);
		} else {
			for (int i = 0; i < retry; i++) {
				bool ok = read_no_retry(cmd, params, param_len, MYID);
				if (ok)
					return true;
			}
		}
		return false;
	}

	HardwareSerial * _port = NULL;
	int _baud = 115200;
	/**
	 * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
	 Command value: 31 Length: 4
	 Parameter 1: Whether the internal motor of the servo is unloaded power-down
	 or not, the range 0 or 1, 0 represents the unloading power down, and the servo
	 has no torque output. 1 represents the loaded motor, then the servo has a
	 torque output, the default value is 0.
	 @return sucess
	 */
	bool disableAll() {
		uint8_t params[] = { 0 };
		return write(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, params, 1,
		LX16A_BROADCAST_ID);
	}
	/**
	 * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
	 Command value: 31 Length: 4
	 Parameter 1: Whether the internal motor of the servo is unloaded power-down
	 or not, the range 0 or 1, 0 represents the unloading power down, and the servo
	 has no torque output. 1 represents the loaded motor, then the servo has a
	 torque output, the default value is 0.
	 @return sucess
	 */
	bool enableAll() {
		uint8_t params[] = { 1 };
		return write(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, params, 1,
		LX16A_BROADCAST_ID);
	}
	/**
	 * Command name: SERVO_MOVE_START Command value: 11
	 Length: 3
	 With the use of command SERVO_MOVE_TIME_WAIT_WRITE, described in
	 point 3
	 @return sucess
	 */
	bool move_sync_start() {
		uint8_t params[1];
		return write(LX16A_SERVO_MOVE_START, params, 1,
		LX16A_BROADCAST_ID);
	}
	/**
	 * Command name: SERVO_MOVE_STOP Command value: 12
	 Length: 3
	 When the command arrives at the servo, it will stop running
	 This is sent to all servos at once
	 */
	void stopAll() {
		uint8_t params[1];
		write(LX16A_SERVO_MOVE_STOP, params, 1,
		LX16A_BROADCAST_ID);
	}
	// id_read returns the ID of the servo, useful if the id is 0xfe, which is broadcast...
	uint8_t id_read() {
		uint8_t params[1];
		if (!read(LX16A_SERVO_ID_READ, params, 1, LX16A_BROADCAST_ID)) {
			return 0;
		}
		return params[0];

	}

	// id_write sets the id of the servo, updates the object's id if write appears successful
	void id_write(uint8_t id) {
		uint8_t params[] = { id };
		write(LX16A_SERVO_ID_WRITE, params, 1, LX16A_BROADCAST_ID);
	}
};

class LX16AServo {
private:
	bool commandOK = true;
	int32_t lastKnownGoodPosition = 0;
	bool isMotorMode = false;
	bool isInitialized = false;
	//private:
	LX16ABus * _bus;


public:
	int32_t staticOffset =0;
	int32_t maxCentDegrees =240000;
	int32_t minCentDegrees =0;

	uint8_t _id = LX16A_BROADCAST_ID;
	LX16AServo(LX16ABus * bus, int id) :
			_bus(bus), _id(id) {
	}
	/**
	 * Set the current position as a specific angle.
	 *
	 * This function will first read the current position, then
	 * calculate an offset to be applied to all position reads and writes.
	 * The offset will make convert the current position to the value passed in
	 *
	 * This is a function to be called when the motor hits a limit switch
	 * and load in a specific value when the limit is reached
	 *
	 */
	bool calibrate(int32_t currentAngleCentDegrees,int32_t min_angle_cent_deg, int32_t max_angle_cent_deg){
		if(min_angle_cent_deg>=max_angle_cent_deg){
			Serial.println("Min can not be greater than max for  "+String(_id)+" halting");
			while(1);
		}
		int32_t current;
		initialize();
		do{
			pos_read();
			current=pos_read_cached();
			if(!isCommandOk()){
				Serial.println("Calibration read FAILED! on index "+String(_id));

			}
		}while(!isCommandOk());// this is a calibration and can not be allowed to fail

		staticOffset=currentAngleCentDegrees-current;
		int32_t min_angle_in_Ticks = (min_angle_cent_deg-staticOffset) / 24;
		int32_t max_angle_in_Ticks = (max_angle_cent_deg-staticOffset) / 24;
		int32_t currentTicks = current/24;
		int32_t angularOffset =1450;
		int32_t angularOffsetTicks =angularOffset/24;
		if(min_angle_in_Ticks<0||max_angle_in_Ticks>1000){
			int32_t  theoretivalMinError  =  currentAngleCentDegrees-min_angle_cent_deg;
			int32_t theoretivalMinErrorTicks = theoretivalMinError/24;
			int32_t newSetpointTicks = theoretivalMinErrorTicks+angularOffsetTicks;
			int32_t newAngle = (newSetpointTicks*24)+staticOffset;
			Serial.println("ERROR! bounds of servo ID "+String(_id)+" can not be outside hardware limit");
			Serial.println("\tlower "+String(min_angle_in_Ticks)+" ticks (Must be > 0)");
			Serial.println("\tcurrent "+String(currentTicks)+" ticks "+String(pos_read_cached()+staticOffset)+"deg");
			Serial.println("\tupper "+String(max_angle_in_Ticks)+" ticks (Must be < 1000)");
			Serial.println("\terror "+String(theoretivalMinErrorTicks)+" ticks ");
			Serial.println("\tnewset "+String(newSetpointTicks)+" ticks ");
			Serial.println("\tnewset deg "+String(newAngle)+" centDeg ");

			min_angle_in_Ticks=0;
			max_angle_in_Ticks=1000;
			minCentDegrees= (min_angle_in_Ticks*24)+staticOffset;
			maxCentDegrees= ((max_angle_in_Ticks)*24)+staticOffset;
			setLimitsTicks(min_angle_in_Ticks,max_angle_in_Ticks);
			move_time(newAngle,0);
			delay(500);
			return false;
		}else{
			Serial.println("\nBounds of servo ID "+String(_id)+" ok!");
			Serial.println("\tlower "+String(min_angle_in_Ticks)+" ticks ");
			Serial.println("\tcurrent "+String(currentTicks)+" ticks ");
			Serial.println("\tupper "+String(max_angle_in_Ticks)+" ticks ");
		}
		setLimitsTicks(min_angle_in_Ticks,max_angle_in_Ticks);
		minCentDegrees= (min_angle_in_Ticks*24)+staticOffset;
		maxCentDegrees= ((max_angle_in_Ticks)*24)+staticOffset;
		if(abs(min_angle_cent_deg-minCentDegrees)>24)
			Serial.println("FAULT Min angle desired was "+String(min_angle_cent_deg)+" got "+String(minCentDegrees));
		if(abs(max_angle_cent_deg-maxCentDegrees)>24)
			Serial.println("FAULT max angle desired was "+String(max_angle_cent_deg)+" got "+String(maxCentDegrees));
		return true;
	}
	void setLimitsTicks(int32_t lower,int32_t upper){
		if(lower<0)
			lower=0;
		if(upper>1000)
			upper=1000;
		for(int i=0;i<2;i++){
			uint8_t params[] = { (uint8_t) lower, (uint8_t) (lower
					>> 8),(uint8_t) upper, (uint8_t) (upper >> 8) };
			commandOK = _bus->write(LX16A_SERVO_ANGLE_LIMIT_WRITE, params,
					4, _id);

			if(isCommandOk()){
				Serial.println("Set Limit MotorID:"+String(_id)+" Min set "+String(((lower*24)+staticOffset))+" max = "+String((upper*24)+staticOffset));
				return;
			}
			if(_bus->_debug)
				Serial.println("Set Limits Failed ID "+String(_id)+" Lower "+String(lower)+" upper: "+String(upper)+" Retry #"+String(i));

		}
	}
	int32_t getMinCentDegrees(){
		return minCentDegrees;
	}
	int32_t getMaxCentDegrees(){
		return maxCentDegrees;
	}
	bool isCommandOk() {
		return commandOK;
	}
	void initialize() {
		if (isInitialized) {
			return;
		}
		isInitialized = true;
		motor_mode(0);
		pos_read();
		readLimits();
		delay(1);
	}

	void readLimits(){
		uint8_t params[4];
		int numFail=0;
		do{
			if (!_bus->read(LX16A_SERVO_ANGLE_LIMIT_READ, params, 4, _id)) {
				commandOK = false;
				if(_bus->_debug){
					Serial.println("ERROR reading limits #"+String(_id));
				}
			} else {
				commandOK = true;
				int lowicks = (params[0] | ((uint16_t) params[1] << 8));
				int highticks = (params[2] | ((uint16_t) params[3] << 8));

				minCentDegrees = (lowicks * 24) + staticOffset;
				maxCentDegrees = (highticks * 24) + staticOffset;
				if (minCentDegrees > maxCentDegrees) {
					Serial.println(
							"ERR MotorID:" + String(_id) + " Min set "
									+ String(minCentDegrees) + " max = "
									+ String(maxCentDegrees) + " Min ticks "
									+ String(lowicks) + " max ticks = "
									+ String(highticks));

					maxCentDegrees = 24000;
					minCentDegrees = 0;
				} else
					Serial.println(
							"MotorID:" + String(_id) + " Min set "
									+ String(minCentDegrees) + " max = "
									+ String(maxCentDegrees));
			}
		}while(!isCommandOk()&&numFail++<3);// this is a calibration and can not be allowed to fail
	}

	/**
	 * Length: 7
	 Parameter 1: lower 8 bits of angle value
	 Parameter 2: higher 8 bits of angle value.range 0~100. corresponding to the
	 servo angle of 0 ~ 240 °, that means the minimum angle of the servo can be
	 varied is 0.24 degree.
	 Parameter 3: lower 8 bits of time value
	 Parameter 4: higher 8 bits of time value. the range of time is 0~30000ms.
	 When the command is sent to servo, the servo will be rotated from current
	 angle to parameter angle at uniform speed within param
	 */
	void move_time(int32_t angle, uint16_t time) {
		initialize();
		if(angle> maxCentDegrees){

			Serial.println("ERROR Capped set at max "+String(maxCentDegrees)+" attempted "+String(angle));
			angle=maxCentDegrees;
		}
		if(angle<minCentDegrees){
			Serial.println("ERROR Capped set at min "+String(minCentDegrees)+" attempted "+String(angle));
			angle=minCentDegrees;
		}
		if (isMotorMode)
			motor_mode(0);
		angle = (angle-staticOffset) / 24;
		if(angle>1000){
			angle=1000;
		}
		if(angle<0){
			angle=0;
		}
		Serial.println("Setting ticks "+String(angle)+" on ID "+String(_id));
		uint8_t params[] = { (uint8_t) angle, (uint8_t) (angle >> 8),
				(uint8_t) time, (uint8_t) (time >> 8) };
		commandOK = _bus->write(LX16A_SERVO_MOVE_TIME_WRITE, params, 4, _id);

	}
	/**
	 * Command name: SERVO_MOVE_TIME_WAIT_WRITE
	 Command value: 7
	 Length : 7
	 Parameter1: lower 8 bits of preset angle
	 Parameter2: higher 8 bits of preset angle. range 0~100. corresponding to the
	 servo angle of 0 ~ 240 °. that means the minimum angle of the servo can be
	 varied is 0.24 degree.
	 Parameter3: lower 8 bits of preset time
	 Parameter3: higher 8 bits of preset time. the range of time is 0~30000ms.
	 The function of this command is similar to this
	 “SERVO_MOVE_TIME_WRITE” command in the first point. But the difference
	 is that the servo will not immediately turn when the command arrives at the
	 servo,the servo will be rotated from current angle to parameter angle at unifor
	 m speed within parameter time until the command name SERVO_MOVE_ST
	 ART sent to servo(command value of 11)
	 , then the servo will be rotate
	 */
	void move_time_and_wait_for_sync(int32_t angle, uint16_t time) {
		initialize();
		if(angle> maxCentDegrees){
			angle=maxCentDegrees;
			Serial.println("ERROR Capped set at max "+String(maxCentDegrees));
		}
		if(angle<minCentDegrees){
			angle=minCentDegrees;
			Serial.println("ERROR Capped set at min "+String(minCentDegrees));
		}
		if (isMotorMode)
			motor_mode(0);
//		if(_bus->_debug)
//			Serial.println("Setting motor to "+String(angle));
		angle = (angle-staticOffset) / 24;
		uint8_t params[] = { (uint8_t) angle, (uint8_t) (angle >> 8),
				(uint8_t) time, (uint8_t) (time >> 8) };
		commandOK = _bus->write(LX16A_SERVO_MOVE_TIME_WAIT_WRITE, params, 4,
				_id);

	}

	/**
	 * Command name: SERVO_MOVE_STOP Command value: 12
	 Length: 3
	 When the command arrives at the servo, it will stop running
	 */
	void stop() {
		uint8_t params[1];
		commandOK = _bus->write(LX16A_SERVO_MOVE_STOP, params, 1, _id);
	}

	/**
	 * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
	 Command value: 31 Length: 4
	 Parameter 1: Whether the internal motor of the servo is unloaded power-down
	 or not, the range 0 or 1, 0 represents the unloading power down, and the servo
	 has no torque output. 1 represents the loaded motor, then the servo has a
	 torque output, the default value is 0.
	 */
	void disable() {
		uint8_t params[] = { 0 };
		commandOK = _bus->write(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, params, 1, _id);
	}
	/**
	 * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
	 Command value: 31 Length: 4
	 Parameter 1: Whether the internal motor of the servo is unloaded power-down
	 or not, the range 0 or 1, 0 represents the unloading power down, and the servo
	 has no torque output. 1 represents the loaded motor, then the servo has a
	 torque output, the default value is 0.
	 */
	void enable() {
		uint8_t params[] = { 1 };
		commandOK = _bus->write(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, params, 1, _id);
	}

	/**
	 * Command name: SERVO_OR_MOTOR_MODE_WRITE
	 Command value: 29
	 Length: 7
	 Parameter 1: Servo mode, range 0 or 1, 0 for position control mode, 1 for
	 motor control mode, default 0,
	 Parameter 2: null value
	 Parameter 3: lower 8 bits of rotation speed value
	 Parameter 4: higher 8 bits of rotation speed value. range -1000~1000,
	 Only in the motor control mode is valid, control the motor speed, the value of
	 the negative value represents the reverse, positive value represents the
	 forward rotation. Write mode and speed do not support power-down save.
	 Note: Since the rotation speed is the “signed short int” type of data, it is forced
	 to convert the data to convert the data to “unsigned short int “type of data before sending the
	 command packet.
	 */
	void motor_mode(int16_t speed) {
		bool isMotorMode_tmp = speed != 0;
		uint8_t params[] = { (uint8_t) (isMotorMode_tmp ? 1 : 0), 0,
				(uint8_t) speed, (uint8_t) (speed >> 8) };
		commandOK = _bus->write(LX16A_SERVO_OR_MOTOR_MODE_WRITE, params, 4,
				_id);
		if (commandOK)
			isMotorMode = isMotorMode_tmp;
	}
	// angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
	void angle_offset_save() {
		uint8_t params[1];
		_bus-> write(LX16A_SERVO_ANGLE_OFFSET_WRITE, params, 1,
				_id);
	}
	// angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
	void angle_offset_adjust(int16_t angle) {
		int32_t tmp = (int32_t) angle;

		uint8_t params[] = { (uint8_t) tmp};
		commandOK = _bus->write(LX16A_SERVO_ANGLE_OFFSET_ADJUST, params, 1,
				_id);
	}
	// angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
	int16_t read_angle_offset() {
		uint8_t params[1];
		if (!_bus->read(LX16A_SERVO_ANGLE_OFFSET_READ, params, 1, _id)) {
			commandOK = false;
			return 0;
		}
		commandOK = true;
		return params[0] ;
	}


	// pos_read returns the servo position in centi-degrees (0..24000)
	int32_t pos_read() {
		initialize();
		uint8_t params[3];
		if (!_bus->read(LX16A_SERVO_POS_READ, params, 2, _id)) {
			if(_bus->_debug)
				Serial.print("Position Read failed "+String(_id)+"\n\n");
			commandOK = false;
			return pos_read_cached()+staticOffset;
		}
		commandOK = true;
		lastKnownGoodPosition = ((int16_t) params[0]
				| ((int16_t) params[1] << 8)) * 24;
		return pos_read_cached()+staticOffset;
	}
	/**
	 * Get the cached position from the most recent read
	 */
	int32_t pos_read_cached() {
		return lastKnownGoodPosition;
	}

	// id_read returns the ID of the servo, useful if the id is 0xfe, which is broadcast...
	uint8_t id_read() {
		uint8_t params[1];
		if (!_bus->read(LX16A_SERVO_ID_READ, params, 1, LX16A_BROADCAST_ID)) {
			commandOK = false;
			return 0;
		}
		commandOK = true;
		return params[0];

	}	// id_read returns the ID of the servo using its ID for verification
	uint8_t id_verify() {
		uint8_t params[1];
		if (!_bus->read(LX16A_SERVO_ID_READ, params, 1, _id)) {
			commandOK = false;
			return 0;
		}
		commandOK = true;
		return params[0];

	}

	// id_write sets the id of the servo, updates the object's id if write appears successful
	void id_write(uint8_t id) {
		uint8_t params[] = { id };
		bool ok = _bus->write(LX16A_SERVO_ID_WRITE, params, 1, LX16A_BROADCAST_ID);
		if (ok && _id != LX16A_BROADCAST_ID)
			_id = id;
		commandOK = ok;
	}
	/**
	 * Command name: SERVO_OR_MOTOR_MODE_READ
	 Command value: 30 Length: 3
	 Read the relative values of the servo, for the details of the command package
	 that the servo returns to host computer, please refer to the description of Table
	 4 below.
	 */
	bool readIsMotorMode() {

		uint8_t params[4];
		if (!_bus->read(LX16A_SERVO_OR_MOTOR_MODE_READ, params, 4, _id)) {
			commandOK = false;
			return false;
		}
		commandOK = true;
		isMotorMode = params[0] == 1;
		return isMotorMode;
	}


	// temp_read returns the servo temperature in centigrade
	uint8_t temp() {
		uint8_t params[1];
		if (!_bus->read(LX16A_SERVO_TEMP_READ, params, 1, _id)) {
			commandOK = false;
			return 0;
		}
		commandOK = true;
		return params[0];
	}

	// vin_read returns the servo input voltage in millivolts
	uint16_t vin() {
		uint8_t params[2];
		if (!_bus->read(LX16A_SERVO_VIN_READ, params, 2, _id)) {
			commandOK = false;
			return 0;
		}
		commandOK = true;
		return params[0] | ((uint16_t) params[1] << 8);
	}

};

