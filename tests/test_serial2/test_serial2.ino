// Arduino Mega using all four of its Serial ports
// (Serial, Serial1, Serial2, Serial3),
// with different baud rates:

void setup() {
  Serial2.begin(115200);
  Serial2.println("Hello Serial 2");
}
static int i = 0;

void loop() {
  Serial2.print("i:");
  Serial2.println(i++);
}
