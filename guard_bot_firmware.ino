#include <SPI.h>
#include "RF24.h"
#include <L3G.h>
#include <Wire.h>

#define GYRO_ROW_TO_DPS 0.00875		//convers row gyro value to degree per second value
#define GYRO_BIAS_LERNING_RATE 0.001
#define HEADING_STEP 30.
#define MAX_RATE 180
#define SPEED_ENCODER_PERIOD 100
#define POWER_STEP 20				// PWM step increase/decrease

#define WHEEL_SPEED_CALIBRATION_FACTOR (11.92 / 3.)
#define PULSES_PER_TURN 20.
#define WHEEL_DIAMETER 0.065
#define PI 3.1415926535897932384626433832795
#define PID_DEAD_ZONE 50
#define CONTROLLER_DEBUG 	 		//enable debug to serial

//PID factors
static float k_p = 8.f;
static float k_i = 0.1f;
static float k_d = 200.f;


static float pos_x = 0.;
static float pos_y = 0.;
static float ref_speed = 0.;
static unsigned long start_time = 0;

static RF24 radio(10, 9); 			// CE, CSN
static byte address[6] = "1Node";	//receive address

// left motor pins
const int enA = 8;
const int in1 = 6;
const int in2 = 5;

// right motor pins
const int enB = 7;
const int in3 = 4;
const int in4 = 3;

static int motor_power = 0;			//current motors power (PWM value)

static L3G gyro;					//gyro interface used for estimate heading
static float heading = 0.f;			//current heading

//giroscope offsets, for best accuracy
static float gyro_offset_z = 0.f;


//current angular rate
static float cur_heading = 0.f;
static float int_value = 0.f;
static float old_error = 0.f;
static int controller_on = 0;

//speed sensors pins
const int encoder_left_pin = 18; 
const int encoder_right_pin = 19;

// counters, used for count impulses from wheel speed sensors
volatile byte pulses_left, pulses_right;

//used for calculate wheel speeds 
unsigned long encoder_last_time;


//process actions for controller
void process_actions(char action) {
	char update = 0;

	switch (action) {
		case 'q': {
			controller_on = 0;
			motor_power = 0;
			cur_heading = heading;
			int_value = 0.f;
			old_error = 0.f;
			break;
		}
		case 'w': {
			motor_power += POWER_STEP;
			update = 1;
			break;
		}
		case 's': {
			motor_power -= POWER_STEP;
			update = 1;
			break;
		}
		case ' ': {
			motor_power = 0;
			cur_heading = heading;
			int_value = 0.f;
			old_error = 0.f;
			controller_on = 0;
			break;
		}
		case 'a': {
			cur_heading = heading + HEADING_STEP;
			update = 1;
			break;
		}
		case 'd': {
			cur_heading = heading - HEADING_STEP;
			update = 1;
			break;
		}
	}

	if (update) {
		controller_on = 1;
		Serial.print("action: "); Serial.println(action);
		Serial.print("motor_power: ");	Serial.println(motor_power);
		Serial.print("cur_heading: "); Serial.println(cur_heading, 4);
	}

}

//update motor controller
void update_controller() {
	//0. stop motors
	if (!controller_on) {
		analogWrite(enA, 0);
		analogWrite(enB, 0);
	}

	unsigned long cur_time = micros();
 	unsigned long micros_dt = cur_time - start_time;

	// 1. check update, FPS = 190 ->equal gyro data
	if (micros_dt < 5264) {
		return;
	}

	start_time = cur_time;

	// 2. read gyro data
	gyro.read();

	// 3. calculate giro rate value
	float rate_z_no_bias = gyro.g.z * GYRO_ROW_TO_DPS;
	float gyro_rate_z = rate_z_no_bias - gyro_offset_z;

	//used for update rate
	//TODO: not calculate bias 1 sec after stop contoller
	if (!controller_on) {
		//update gyro bias
		gyro_offset_z += GYRO_BIAS_LERNING_RATE * (rate_z_no_bias - gyro_offset_z);
		return;
	}

	// 4. get current dt in sec
	float dt = micros_dt / 1000000.f;

	// 5. calculate heading angle
	heading +=	gyro_rate_z * dt;

	// 6. calculate position
	pos_x += ref_speed * cos(heading / 180. * PI) * dt;
	pos_y += ref_speed * sin(heading / 180. * PI) * dt;

	// 9. process PID controller for rate
	float error = cur_heading - heading;
	float p_value = error * k_p;
	float d_value = (error - old_error) * k_d;
	int_value += error * k_i;

	float common_value = p_value + int_value + d_value;
	old_error = error;

	int left_power = motor_power - common_value;
	int right_power = motor_power + common_value;

	#ifdef CONTROLLER_DEBUG
		// Serial.print(" t_h: "); Serial.print(cur_heading, 4);
		// Serial.print(" c_h: "); Serial.print(heading, 4);
		// Serial.print(" e: "); Serial.print(error, 4);
		// Serial2.print("p_v: "); Serial2.print(p_value, 4);
		// Serial2.print("  i_v: "); Serial2.print(int_value, 4);
		// Serial2.print("  d_v: "); Serial2.print(d_value, 4);
		// Serial2.print("  c_v: "); Serial2.println(common_value, 4);
		Serial.println(error, 4);
		#endif

	int abs_left_power = abs(left_power);
	if (abs_left_power < PID_DEAD_ZONE) {
		left_power = 0;
	} else  if (abs_left_power > 255) {
		left_power = left_power > 0 ? 255 : -255; 
	}

	int abs_right_power = abs(right_power);
	if (abs_right_power < PID_DEAD_ZONE) {
		right_power = 0;
	} else if (abs_right_power > 255) {
		right_power = right_power > 0 ? 255 : -255;	
	}

	// turn on motor A
	if (left_power > 0) {
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
	} else {
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
	}
 
	analogWrite(enA, abs(left_power));

	// turn on motor B
	if (right_power > 0) {
		digitalWrite(in3, HIGH);
		digitalWrite(in4, LOW);
	} else {
		digitalWrite(in3, LOW);
		digitalWrite(in4, HIGH);
	}
	analogWrite(enB, abs(right_power));

	#ifdef CONTROLLER_DEBUG
		//print count per second 
		static unsigned long start_count_time = 0;
		static int process_count = 0;
		process_count++;
	 
		unsigned long count_dt = cur_time - start_count_time;
		if (count_dt > 3000000) {
			start_count_time = cur_time;
			Serial.print("process per second:"); Serial.println(int(process_count / (count_dt / 1000000.)));
			process_count = 0;
		}
	#endif
}

//counting of left wheel impulses
void counter_left() {
	pulses_left++;
}

//counting of right wheel impulses
void counter_right() {
	pulses_right++;
}

void setup_speed_sensors() {
	pinMode(encoder_left_pin, INPUT);
	pinMode(encoder_right_pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(encoder_left_pin), counter_left, FALLING);
	attachInterrupt(digitalPinToInterrupt(encoder_right_pin), counter_right, FALLING);
	// Инициализация
	pulses_left = 0;
	pulses_right = 0;
	encoder_last_time = millis();
}


void update_encoders_speed() {
	unsigned long cur_time = millis();

	//update every
	unsigned long dt = cur_time - encoder_last_time;
	if (dt >= SPEED_ENCODER_PERIOD) {
		encoder_last_time = cur_time;
		//Не обрабатывать прерывания во время счёта
		detachInterrupt(digitalPinToInterrupt(encoder_left_pin));
		byte cur_pulses_left = pulses_left;
		pulses_left = 0;
		attachInterrupt(digitalPinToInterrupt(encoder_left_pin), counter_left, FALLING);

		detachInterrupt(digitalPinToInterrupt(encoder_right_pin));
		byte cur_pulses_right = pulses_right;
		pulses_right = 0;
		attachInterrupt(digitalPinToInterrupt(encoder_right_pin), counter_right, FALLING);

		//calculate reference speed
		float l_speed = (float(cur_pulses_left) / PULSES_PER_TURN * PI * WHEEL_DIAMETER) / 0.1 / WHEEL_SPEED_CALIBRATION_FACTOR;
		float r_speed = (float(cur_pulses_right) / PULSES_PER_TURN * PI * WHEEL_DIAMETER) / 0.1 / WHEEL_SPEED_CALIBRATION_FACTOR;
		ref_speed = (l_speed + r_speed) / 2.;

		// Serial2.print("dt: ");
		// Serial2.print(dt, 6);
		// Serial2.print(" l: ");
		// Serial2.print(cur_pulses_left, DEC);
		// Serial2.print(" r: ");
		// Serial2.print(cur_pulses_right, DEC);
		// Serial2.print(" h: ");
		// Serial2.print(heading, 6);
		// Serial2.print(" r: ");
		// Serial2.print(gyro_rate_z, 6);
		// Serial2.print(" s: ");
		// Serial2.print(ref_speed, 6);
		// Serial2.print(" x: ");
		// Serial2.print(pos_x, 6);
		// Serial2.print(" y: ");
		// Serial2.println(pos_y, 6);
	}
}


void setup() {
	//1. init serial
	Serial.begin(115200);
	Serial.println("Start car controller");

	//init debug serial BT
	Serial2.begin(115200);

	//2. init radio
	radio.begin();
	// Set the PA Level low to prevent power supply related issues since this is a
	// getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
	radio.setPALevel(RF24_PA_LOW);
	radio.openReadingPipe(1,address);
	// Start the radio listening for data
	radio.startListening();

	if (!radio.isChipConnected()) {
		Serial.println("Radio chip not Connected");
		while (1);
	}
	Serial.println("Radio chip Connected");

	delay(100);
	//3. set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	delay(200);

	//4. enable giroscope
	Wire.begin();

	if (!gyro.init()) {
		Serial.println("Failed to autodetect gyro type!");
		while (1);
	}
	Serial.println("Gyro device connected");
	gyro.enableDefault();

	setup_speed_sensors();
}

void process_bt_data() {
	/**
	this function used for pid settings:
	for setup PID factors, write: p=12.31 i=0.1 d=0.0
	for printing current PID factor, write: print
	*/
	static char buffer[30];
	static char * ptr = buffer;

	if (Serial2.available()) {
		*ptr = Serial2.read();

		//end of line
		if (*ptr == '\n') {
			*ptr = 0;
			String res(buffer);
			ptr = buffer;

			if (res.length()) {
				//search for p=
				char fail = 1;
				int index = res.indexOf(String("p="));
				if (index >= 0) {
					res = res.substring(index + 2);
					k_p = res.toFloat();
					Serial2.println("ok");
					fail = 0;
				}

				//search for p=
				index = res.indexOf(String("i="));
				if (index >= 0) {
					res = res.substring(index + 2);
					k_i = res.toFloat();
					Serial2.println("ok");
					fail = 0;
				}

				//search for d=
				index = res.indexOf(String("d="));
				if (index >= 0) {
					res = res.substring(index + 2);
					k_d = res.toFloat();
					Serial2.println("ok");
					fail = 0;
				}

				index = res.indexOf(String("print"));
				if (index >= 0) {
					Serial2.print("p=");
					Serial2.print(k_p, 6);
					Serial2.print(" i=");
					Serial2.print(k_i, 6);
					Serial2.print(" d=");
					Serial2.println(k_d, 6);
					fail = 0;
				}

				if (fail == 1) {
					Serial2.println("unk");
				}
			}
		}
		//end off buffer
		if (++ptr == buffer) {
			ptr = buffer;
		}
	}
}

void loop() {
	unsigned long data = 0;

	//use serial command channel
	if (Serial.available()) {
		data = Serial.read();
	}

	//use NRF as input channel
	if (radio.available()) {
		// While there is data ready
		while (radio.available()) {
			// Get the payload
			radio.read(&data, sizeof(data));
			Serial.print("Radio receive:"); Serial.println(data);
		}
	}

	process_actions(data);
	update_controller();
	update_encoders_speed();
	process_bt_data();
	//updateSensor();
}
