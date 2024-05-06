// Using a POT on A15, controls the speed and direction of 4 motors on the
//L293D Aideepen motor control shield
//https://www.amazon.com/dp/B09Y1WB2VN?psc=1&ref=ppx_yo2ov_dt_b_product_details
//Needs the [Adafruit Motor Shield Library](https://github.com/adafruit/Adafruit-Motor-Shield-library

#include <AFMotor.h>

#define TIMER_DELAY 100
#define POT_MIDDLE  512
#define DEAD_ZONE   50

#define NUM_MOTORS 1
#define MAX_SPEED 150
#define SENSOR_PIN A15

#define ENCA 21
#define ENCB 53


AF_DCMotor* motors[NUM_MOTORS];
uint8_t direction = RELEASE;
uint8_t speed = 0;
int encCount = 0;
int pos = 0;
int encDirection = RELEASE;
int prevEncCount = 0;
unsigned long timer = 0;
unsigned long prevTime = 0;

void readEncoder() {
  int b=digitalRead(ENCB);
  if(b>0) {
    pos++;
  } else {
    pos--;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  Serial.println("Hello world motor");
}
//608-254-7877
void loop() {
  unsigned long now = millis();
  if(now>timer) {
    Serial.print("pos:");
    Serial.print(pos);
    Serial.println();
    timer=now+TIMER_DELAY;
  }
}
