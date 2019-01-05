#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <L3G.h>

RF24 radio(10, 9); // CE, CSN
byte address[6] = "1Node";  //receive address

// motor one
const int enA = 8;
const int in1 = 6;
const int in2 = 5;

// motor two
const int enB = 7;
const int in3 = 4;
const int in4 = 3;

int left_power = 0;
int right_power = 0;
int power_step = 20;

L3G gyro;

//giroscope offsets, for best accuracy
const float gyro_offset_x = -0.301108079748;
const float gyro_offset_y = -0.232206190976;
const float gyro_offset_z = -0.301108079748;

//current angular rate
float cur_rate = 0.0;


//process actions from controllers
void process_actions(char action) {
  char update = 0;

  switch (action) {
    case 'w': {
      left_power += power_step;
      right_power += power_step;
      update = 1;
      break;
    }
    case 's': {
      left_power -= power_step;
      right_power -= power_step;
      update = 1;
      break;
    }
    case ' ': {
      left_power = 0;
      right_power = 0;
      cur_rate = 0.0;
      update = 1;
      break;
    }
    case 'a': {
      // left_power -= power_step;
      // right_power += power_step;
      cur_rate += 0.1;
      update = 1;
      break;
    }
    case 'd': {
      // left_power += power_step;
      // right_power -= power_step;
      cur_rate -= 0.1;
      update = 1;
      break;
    }
  }

  if (update) {
    Serial.print("action:"); Serial.println(action);
    Serial.print("left_power:");  Serial.println(left_power);
    Serial.print("right_power:"); Serial.println(right_power);
  }

}

//update motor controller
void update_controller() {
  static unsigned long start_time = 0;
  unsigned long cur_time = micros();
 
  //FPS = 190 ->equal gyro data
  if (cur_time - start_time < 5264) {
    return;
  }

  start_time = cur_time; 

  // static float x,y;
  static float z;
  //read gyro data
  gyro.read();

  //convert to deg per second and remove biase
  // x = gyro.g.x * 0.00875 - gyro_offset_x;
  // y = gyro.g.y * 0.00875 - gyro_offset_y;
  z = gyro.g.z * 0.00875 - gyro_offset_z;

  //controller
  // static float int_value = 0.0;
  // static float old_error = 0.0;
  // float error = cur_rate - z;
  // float p_value = error * 0;
  // float d_value = (old_error - error) * 0.1; 
  // int_value += error * 0.1;
  // float common_value = p_value + int_value + d_value;
  // old_error = error;

  // left_power = -common_value;
  // right_power = common_value;
  // Serial.print("e: "); Serial.print(error);
  // Serial.print(" i_v: "); Serial.print(int_value);
  // Serial.print(" c_v: "); Serial.println(common_value);

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
 
  unsigned long dt = cur_time - start_count_time;
  if (dt > 3000000) {
    start_count_time = cur_time;
    Serial.print("process per second:"); Serial.println(int(process_count / (dt / 1000000.)));
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
