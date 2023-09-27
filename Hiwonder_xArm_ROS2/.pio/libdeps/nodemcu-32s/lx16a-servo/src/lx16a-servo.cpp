#include <Arduino.h>
#include <lx16a-servo.h>

// write a command with the provided parameters
// returns true if the command was written without conflict onto the bus
bool LX16ABus::write_no_retry(uint8_t cmd, const uint8_t *params, int param_cnt,
		uint8_t MYID) {
	if (param_cnt < 0 || param_cnt > 4)
		return false;

	// prepare packet in a buffer
	int buflen = 6 + param_cnt;
	uint8_t buf[buflen];
	uint8_t ret[buflen];
	buf[0] = 0x55;
	buf[1] = 0x55;
	buf[2] = MYID;
	buf[3] = buflen - 3;
	buf[4] = cmd;
	for (int i = 0; i < param_cnt; i++)
		buf[5 + i] = params[i];
	uint8_t cksum = 0;
	for (int i = 2; i < buflen - 1; i++)
		cksum += buf[i];
	buf[buflen - 1] = ~cksum;

// clear input buffer
	int junkCount = 0;
	delayMicroseconds(timeus(2));
	while (available()) {
		if (junkCount == 0) {
//			if (_debug)
//				Serial.print("\n\t\t[ ");
		}
//		if (_debug)
//			Serial.print(" " + String(read()) + ", ");
//		else
			read();
		delayMicroseconds(timeus(3));
		junkCount++;
	}

//	if (_debug && junkCount!=0) {
//		Serial.print("]\n ");
//		Serial.println(
//				"\t\t Junk bytes = " + String(junkCount) + " id " + String(MYID)
//						+ " cmd " + String(cmd) + " last cmd "
//						+ String(lastCommand));
//	}
	lastCommand = cmd;
	// send command packet
	uint32_t t0 = millis();
	if(!singlePinMode)
	if (myTXFlagGPIO >= 0) {
		digitalWrite(myTXFlagGPIO, 1);
	}
	delayMicroseconds(timeus(1));
	write(buf, buflen);
	delayMicroseconds(timeus(1));
	if(!singlePinMode)
	if (myTXFlagGPIO >= 0) {
		digitalWrite(myTXFlagGPIO, 0);
	}
	// expect to read back command by virtue of single-pin loop-back
	uint32_t tout = time(buflen+4) + 4; // 2ms margin
	int got = 0;
	bool ok = true;
	if(!singlePinMode){
		if (_deepDebug)
			Serial.println("RCV: ");
		while ((got < buflen) && ((millis() - t0) < tout)) {
			if (available()) {
				ret[got] = read();
				if (ret[got] != buf[got]) {
					ok = false;
				}
				got++;
			}
		}
		if (got<buflen){
			ok = false;

		}
		if (_debug) {
			if (!ok) {
				Serial.print("\n\n\tWrote:[ ");
				for(int i=0;i<buflen;i++){
					Serial.print(" " + String(buf[i]) + ", ");
				}
				Serial.print("] id " + String(MYID)
						+ " cmd " + String(cmd) + " last cmd "
						+ String(lastCommand));

				Serial.print("\n\tGot  :[ ");
				for(int i=0;i<got;i++){
					Serial.print(" " + String(ret[i]) + ", ");
				}
				Serial.print("] in "+String(millis()-t0)+"\n");
			}
		}
	}

	return ok;
}
bool LX16ABus::rcv(uint8_t cmd, uint8_t *params, int param_len, uint8_t MYID) {
	// read back the expected response
	uint32_t t0 = millis();
	uint32_t tout = time(param_len + 6) + 30; // time in ms for the servo to think
	int got = 0;
	uint8_t sum = 0;
//	if (_deepDebug)
//		Serial.println("RCV: ");
	int len = 7; // minimum length
	while (got < len && ((millis() - t0) < tout)) {
		if (available()) {
			int ch = read();
//			if (_deepDebug)
//				Serial.println(" 0x%02x", ch);
			switch (got) {
			case 0:
			case 1:
				if (ch != 0x55) {
					if (_debug)
						Serial.println(" ERR (hdr expected 0x55) 0x"+String(ch,HEX)+" cmd "+String(cmd)+" ID "+String(MYID)+" PacketIndex "+String(got)+"\n");
					return false;
				}
				break;
			case 2:
				if (ch != MYID && MYID != 0xfe) {
					if (_debug)
						Serial.println(" ERR (id)\n");
					return false;
				}
				break;
			case 3:
				if (ch < 3 || ch > 7) {
					if (_debug)
						Serial.println(" ERR (len)\n");
					return false;
				}
				len = ch + 3;
				if (len > param_len + 6) {
					if (_debug)
						Serial.println(
								" ERR (param_len) got " + String(len)
										+ " expected "
										+ String((param_len + 6)));
					return false;
				}
				break;
			case 4:
				if (ch != cmd) {
					if (_debug)
						Serial.println(" ERR (cmd)\n");
					return false;
				}
				break;
			default:
				if (got == len - 1) {
					if ((uint8_t) ch == (uint8_t) ~sum) {
						//if (_deepDebug)
						//	Serial.println(" OK\n");
						return true;
					} else {
						if (_debug)
							Serial.println(" ERR (cksum!="+String(~sum)+")\n");
						return false;
					}
				}
				if (got - 5 > param_len) {
					if (_debug)
						Serial.println(" ERR (cksum)\n");
					return false;
				}
				params[got - 5] = ch;
			}
			if (got > 1)
				sum += ch;
			got++;
		}
	}
	if (_debug){
		long done = millis();
		Serial.println(
				"Read TIMEOUT Expected " + String(len) + " got " + String(got)

						+ " on cmd: " + String(cmd) + " id " + String(MYID)
						+" took "+String(done-t0));
	}
	return false;
}
// read sends a command to the servo and reads back the response into the params buffer.
// returns true if everything checks out correctly.
bool LX16ABus::read_no_retry(uint8_t cmd, uint8_t *params, int param_len,
		uint8_t MYID) {
	// send the read command
	bool ok = write(cmd, NULL, 0, MYID);
	if (!ok) {
		if (_debug)
			Serial.println(
					"Command of read failed on cmd: " + String(cmd) + " id "
							+ String(MYID));
		return false;
	}

	return rcv(cmd, params, param_len, MYID);
}
