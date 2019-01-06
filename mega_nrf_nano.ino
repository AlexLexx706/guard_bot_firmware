#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <L3G.h>

static RF24 radio(10, 9); // CE, CSN
static byte address[6] = "1Node";  //receive address

// motor one
const int enA = 8;
const int in1 = 6;
const int in2 = 5;

// motor two
const int enB = 7;
const int in3 = 4;
const int in4 = 3;

static int power_step = 20;
static int motor_power = 0;

static L3G gyro;

//giroscope offsets, for best accuracy
const float gyro_offset_x = -0.301108079748;
const float gyro_offset_y = -0.232206190976;
const float gyro_offset_z = -0.301108079748;

//current angular rate
static float cur_rate = 0.0;
static float int_value = 0.0;
static float old_error = 0.0;

//PID factors
static float k_p = 1.0;
static float k_i = 0.1;
static float k_d = 0.3;
static float k_ff = 0.0;



//process actions from controllers
void process_actions(char action) {
  char update = 0;

  switch (action) {
    case 'w': {
      motor_power += power_step;
      update = 1;
      break;
    }
    case 's': {
      motor_power -= power_step;
      update = 1;
      break;
    }
    case ' ': {
      motor_power = 0;
      cur_rate = 0.0;
      int_value = 0.0;
      old_error = 0.0;

      update = 1;
      break;
    }
    case 'a': {
      // left_power -= power_step;
      // right_power += power_step;
      cur_rate += 10.;
      update = 1;
      break;
    }
    case 'd': {
      // left_power += power_step;
      // right_power -= power_step;
      cur_rate -= 10.;
      update = 1;
      break;
    }
  }

  if (update) {
    Serial.print("action: "); Serial.println(action);
    Serial.print("motor_power: ");  Serial.println(motor_power);
    Serial.print("cur_rate: "); Serial.println(cur_rate, 4);
  }

}

//update motor controller
void update_controller() {
  static unsigned long start_time = 0;
  unsigned long cur_time = micros();
 
  // 1. check update, FPS = 190 ->equal gyro data
  unsigned long micros_dt = cur_time - start_time;
  if (micros_dt < 5264) {
    return;
  }
  start_time = cur_time;

  // 2. get current dt in sec
  float dt = micros_dt / 1000000.f;
  // Serial.print("micros_dt: "); Serial.print(micros_dt);
  // Serial.print(" dt: "); Serial.println(dt, 6);

  // read gyro data
  gyro.read();

  // 3. calculate giro rate value
  float g_z = gyro.g.z * 0.00875 - gyro_offset_z;

  // 4. calculate heading angle
  static float heading = 0.0;
  heading +=  g_z * dt;

  // Serial.print("g_z: "); Serial.print(g_z, 4);
  // Serial.print(" heading: "); Serial.println(heading, 4);

  // 5. update controller 
  float error = cur_rate - g_z;
  float p_value = error * k_p;
  float d_value = (error - old_error) * k_d;
  float ff_value =  cur_rate * k_ff;

  int_value += error * k_i;
  float common_value = p_value + int_value + d_value + ff_value;
  old_error = error;

  int left_power = motor_power - common_value;
  int right_power = motor_power + common_value;

  Serial.print(" t_r: "); Serial.print(cur_rate, 4);
  Serial.print(" c_r: "); Serial.print(g_z, 4);
  Serial.print(" e: "); Serial.print(error, 4);
  Serial.print(" p_v: "); Serial.print(p_value, 4);
  Serial.print(" i_v: "); Serial.print(int_value, 4);
  Serial.print(" d_v: "); Serial.print(d_value, 4);
  Serial.print(" ff_v: "); Serial.print(ff_value, 4);
  Serial.print(" c_v: "); Serial.println(common_value, 4);

  if (abs(left_power) > 255) {
    left_power = left_power > 0 ? 255 : -255; 
  }
  if (abs(right_power) > 255) {
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
}



void setup() {
  //1. init serial
  Serial.begin(115200);
  Serial.println("Start car controller");

  //2. init radio
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1,address);
  // Start the radio listening for data
  radio.startListening();

  if (radio.isChipConnected()) {
    Serial.println("Chip Connected");
  } else {
    Serial.println("Chip not Connected");
  }

  //3. set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //4. enable giroscope
  Wire.begin();

  if (!gyro.init()) {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  Serial.print("Gyro device type:");
  Serial.println(gyro.getDeviceType());
  gyro.enableDefault();
}

void loop() {
  unsigned long data = 0;
  //data come

  if( radio.available()){
    while (radio.available()) {         // While there is data ready
      radio.read(&data, sizeof(data));  // Get the payload

      Serial.print("receave data:");
      Serial.println(data);
    }
  }
  
  if (Serial.available()) {
    data = Serial.read();
  }
  process_actions(data);
  update_controller();
}
