int encoder_left_pin = 18; // импульсные сигналы от модуля
int encoder_right_pin = 19; // импульсные сигналы от модуля

volatile byte pulses_left, pulses_right; // количество импульсов
unsigned long timeold;

void counter_left() {
	//обновление счета импульсов
	pulses_left++;
}

void counter_right() {
	//обновление счета импульсов
	pulses_right++;
}

void setup()
{
	Serial.begin(9600);
	Serial2.begin(9600);
	pinMode(encoder_left_pin, INPUT);
	pinMode(encoder_right_pin, INPUT);
	//Прерывание 0 на цифровой линии 2
	//Срабатывание триггера по спаду сигнала
	attachInterrupt(digitalPinToInterrupt(encoder_left_pin), counter_left, FALLING);
	attachInterrupt(digitalPinToInterrupt(encoder_right_pin), counter_right, FALLING);
	// Инициализация
	pulses_left = 0;
	pulses_right = 0;
	timeold = millis();
}
void loop()
{

	unsigned long cur_time = millis();	
	if (cur_time - timeold >= 100) {
		//Не обрабатывать прерывания во время счёта
		detachInterrupt(digitalPinToInterrupt(encoder_left_pin));
		byte cur_pulses_left = pulses_left;
		pulses_left = 0;
		attachInterrupt(digitalPinToInterrupt(encoder_left_pin), counter_left, FALLING);

		detachInterrupt(digitalPinToInterrupt(encoder_right_pin));
		byte cur_pulses_right = pulses_right;
		pulses_right = 0;
		attachInterrupt(digitalPinToInterrupt(encoder_right_pin), counter_right, FALLING);
		timeold = cur_time;
		Serial.print("left: ");
		Serial.print(cur_pulses_left, DEC);
		Serial.print(" right: ");
		Serial.println(cur_pulses_right, DEC);

		Serial2.print("left: ");
		Serial2.print(cur_pulses_left, DEC);
		Serial2.print(" right: ");
		Serial2.println(cur_pulses_right, DEC);
		//Перезагрузка процесса обработки прерываний
	}
}