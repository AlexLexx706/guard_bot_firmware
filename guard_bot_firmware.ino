#include <SPI.h>
#include "RF24.h"
#include <L3G.h>
#include <Wire.h>

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_MOTION_BURST          0x50

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30

// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE      0x17

// mouse camera values
SPISettings spiSettings(2e6, MSBFIRST, SPI_MODE3); // 2 MHz, mode 3
static const uint8_t RESET_PIN = 11; // pin 11
static const uint8_t SS_PIN = 2; // Pin 2
static int32_t x, y;


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

//PID factorsq
static float k_p = 1.0;
static float k_i = 0.1;
static float k_d = 0.3;
static float k_ff = 0.0;
static int controller_on = 1;

// mouse camera functions
void updateSensor(void) {
  // Read sensor
  uint8_t buf[4];
  spiRead(ADNS3080_MOTION_BURST, buf, 4);

  uint8_t motion = buf[0];
  //Serial.println(motion, BIN); // Resolution

  if (motion & 0x10) // Check if we've had an overflow
    Serial.println(F("ADNS-3080 overflow\n"));
  else if (motion & 0x80) {
    int8_t dx = buf[1];
    int8_t dy = buf[2];
    uint8_t surfaceQuality = buf[3];

    x += dx;
    y += dy;

    // Print values
    Serial.print(x);
    Serial.write(',');
    Serial.print(dx);
    Serial.write('\t');
    Serial.print(y);
    Serial.write(',');
    Serial.print(dy);
    Serial.write('\t');
    Serial.println(surfaceQuality);
    Serial.flush();
  }
  //delay(10);
}

void reset(void) {
  digitalWrite(SS_PIN, HIGH); // reset ss
  digitalWrite(RESET_PIN, HIGH); // Set high
  delayMicroseconds(10);
  digitalWrite(RESET_PIN, LOW); // Set low
  delayMicroseconds(500); // Wait for sensor to get ready
}

// Will cause the Delta_X, Delta_Y, and internal motion registers to be cleared
void clearMotion() {
  spiWrite(ADNS3080_MOTION_CLEAR, 0xFF); // Writing anything to this register will clear the sensor's motion registers
  x = y = 0;
}

void spiWrite(uint8_t reg, uint8_t data) {
  spiWrite(reg, &data, 1);
}

void spiWrite(uint8_t reg, uint8_t *data, uint8_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);

  SPI.transfer(reg | 0x80); // Indicate write operation
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  SPI.transfer(data, length); // Write data

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t spiRead(uint8_t reg) {
  uint8_t buf;
  spiRead(reg, &buf, 1);
  return buf;
}

void spiRead(uint8_t reg, uint8_t *data, uint8_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);

  SPI.transfer(reg); // Send register address
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  memset(data, 0, length); // Make sure data buffer is 0
  SPI.transfer(data, length); // Write data

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

void setup_mouse_camera() {
  SPI.begin();
  // Set SS and reset pin as output
  pinMode(SS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  reset();

  uint8_t id = spiRead(ADNS3080_PRODUCT_ID);
  if (id == ADNS3080_PRODUCT_ID_VALUE)
    Serial.println(F("ADNS-3080 found"));
  else {
    Serial.print(F("Could not find ADNS-3080: "));
    Serial.println(id, HEX);
    while (1);
  }

  uint8_t config = spiRead(ADNS3080_CONFIGURATION_BITS);
  spiWrite(ADNS3080_CONFIGURATION_BITS, config | 0x10); // Set resolution to 1600 counts per inch
}



//process actions from controllers
void process_actions(char action) {
  char update = 0;

  switch (action) {
    case 'q': {
      controller_on = 0;
      motor_power = 0;
      cur_rate = 0.0;
      int_value = 0.0;
      old_error = 0.0;
      break;
    }
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
      controller_on = 0;
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
    controller_on = 1;
    Serial.print("action: "); Serial.println(action);
    Serial.print("motor_power: ");  Serial.println(motor_power);
    Serial.print("cur_rate: "); Serial.println(cur_rate, 4);
  }

}


//#define CONTROLLER_DEBUG

//update motor controller
void update_controller() {
  if (!controller_on) {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    return;
  }

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

  #ifdef CONTROLLER_DEBUG
    Serial.print(" t_r: "); Serial.print(cur_rate, 4);
    Serial.print(" c_r: "); Serial.print(g_z, 4);
    Serial.print(" e: "); Serial.print(error, 4);
    Serial.print(" p_v: "); Serial.print(p_value, 4);
    Serial.print(" i_v: "); Serial.print(int_value, 4);
    Serial.print(" d_v: "); Serial.print(d_value, 4);
    Serial.print(" ff_v: "); Serial.print(ff_value, 4);
    Serial.print(" c_v: "); Serial.println(common_value, 4);
  #endif

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
  delay(100);

  delay(100);

  //4. enable giroscope
  Wire.begin();

  if (!gyro.init()) {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  Serial.println("Gyro device connected");
  gyro.enableDefault();

  //5. setup camera
  // setup_mouse_camera();
}

static float packet[2];

void loop() {
  unsigned long data = 0;
  //data come

  // if (radio.available()) {
  //   while (radio.available()) {         // While there is data ready
  //     radio.read(packet, sizeof(packet));  // Get the payload
  //     Serial.print("Radio receive: w:"); Serial.print(packet[0]); Serial.print(" v:"); Serial.println(packet[1]); 
  //     motor_power = packet[1];
  //     cur_rate = packet[0];
  //   }
  // }
  
  if (Serial.available()) {
    data = Serial.read();
  }


  if (radio.available()) {
    while (radio.available()) {         // While there is data ready
      radio.read(&data, sizeof(data));  // Get the payload
      Serial.print("Radio receive:"); Serial.println(data);
    }
  }

  process_actions(data);
  update_controller();
  // updateSensor();
}