


#define CTRL_PWR_PIN    D5
#define CTRL_DIM_PIN    D6
#define CTRL_TIM_PIN    D7

#define READ_PWR_PIN    D0
#define READ_DIM_PIN    D1
#define READ_TIM_PIN    D2

#define RESET_BTN_PIN   D3


void setup() {
  Serial.begin(9600);
  
  pinMode(CTRL_PWR_PIN, OUTPUT);
  pinMode(CTRL_DIM_PIN, OUTPUT);
  pinMode(CTRL_TIM_PIN, OUTPUT);
  pinMode(READ_PWR_PIN, INPUT);
  pinMode(READ_DIM_PIN, INPUT);
  pinMode(READ_TIM_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RESET_BTN_PIN, INPUT); 

  digitalWrite(CTRL_PWR_PIN, LOW);
  digitalWrite(CTRL_DIM_PIN, LOW);
  digitalWrite(CTRL_TIM_PIN, LOW);

  Serial.println(F("ESP8266 Begin"));
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  printState();
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  printState();
  delay(500);
}

void printState() {
  Serial.printf("PWR = %d\n", digitalRead(READ_PWR_PIN));
  Serial.printf("DIM = %d\n", digitalRead(READ_DIM_PIN));
  Serial.printf("TIM = %d\n", digitalRead(READ_TIM_PIN));
}
