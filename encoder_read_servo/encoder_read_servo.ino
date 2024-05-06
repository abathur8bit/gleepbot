//L293D Aideepen motor control shield
//https://www.amazon.com/dp/B09Y1WB2VN?psc=1&ref=ppx_yo2ov_dt_b_product_details
//Needs the [Adafruit Motor Shield Library](https://github.com/adafruit/Adafruit-Motor-Shield-library

#include <AFMotor.h>
//from AFMotor.h >>>
//#define FORWARD 1
//#define BACKWARD 2
//#define BRAKE 3
//#define RELEASE 4
//<<<

#define TIMER_DELAY 500
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
int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int encDirection = RELEASE;

int prevEncCount = 0;
unsigned long timer = 0;
unsigned long prevTime = 0;

void readEncoder() {
  int b=digitalRead(ENCB);
  if(b>0) {
    posi++;
  } else {
    posi--;
  }
}

void setup() {
  Serial.begin(115200);
  for(int m=0; m<NUM_MOTORS; m++) {
    motors[m] = new AF_DCMotor(m+1);
    motors[m]->setSpeed(0);
    motors[m]->run(RELEASE);
  }
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void setMotor(int dir,int speed) {
  if(dir==1) {
    motors[0]->run(FORWARD);
  } else {
    motors[0]->run(BACKWARD);
  }
  motors[0]->setSpeed(speed);
}

void loop() {
  unsigned long now = millis();

  // set target position aka SP or r(t)
  int target = 200;
//  int target = 250*sin(prevT/1e6);  //this makes a sine wave for the position

  // PID constants
  float kp = 1;     //1    10
  float kd = 0.025; //0.025 0.025
  float ki = 0.0;   //0.0 10

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = posi; 

  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr);


  // store previous error
  eprev = e;
  
  
  if(now>timer) 
  {
    Serial.print("pos:");
    Serial.print(pos);
    Serial.print(" target:");
    Serial.print(target);
    Serial.println();
    timer=now+TIMER_DELAY;
  }
  
}
