#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

struct Filburbot_Motor {
  Adafruit_DCMotor * motor;
  int motor_pin;
  int encoder1_pin;
  int encoder2_pin;
  long last_position;
};

Filburbot_Motor left, right;

void Filburbot_Motor_init(Filburbot_Motor *m) {
  m->motor = AFMS.getMotor(m->motor_pin);
  pinMode(m->encoder1_pin, INPUT);
  pinMode(m->encoder2_pin, INPUT);
  m->last_position = 0;
}

byte Filburbot_Motor_get_encoder_state(Filburbot_Motor *m) {
  return digitalRead(m->encoder2_pin) << 1 | digitalRead(m->encoder1_pin);
}

Encoder right_enc(6, 7);

void setup() {
  left.motor_pin = 3;
  left.encoder1_pin = 5;
  left.encoder2_pin = 4;
  Filburbot_Motor_init(&left);

  right.motor_pin = 1;
  right.encoder1_pin = 7;
  right.encoder2_pin = 6;
  Filburbot_Motor_init(&right);

  Serial.begin(9600);
  Serial.println("Filburbot");

  AFMS.begin();

  left.motor->setSpeed(255);
  left.motor->run(BACKWARD);
  right.motor->setSpeed(255);
  right.motor->run(FORWARD);
}

long position = -999;

unsigned long initial_time = millis();
unsigned long last_time = millis();
unsigned long count = 0;
bool triggered = false;

void loop() {
  unsigned long now = millis();
  long current_position = right_enc.read();
  // if (current_position != position) {
  //   position = current_position;
  //   Serial.println(position);
  // }

  unsigned long duration = now - last_time;
  if(duration >= 100) {
    int speed = (current_position - right.last_position);
    Serial.println(speed);
    right.last_position = current_position;
    last_time = now;
  }
  // // With any substantial delay added, Encoder can only track
  // // very slow motion.  You may uncomment this line to see
  // // how badly a delay affects your encoder.
  // //delay(50);

  // byte encoder_val = Filburbot_Motor_get_encoder_state(&right);
  //
  //
  // if(encoder_val != last_encoder_val) {
  //   Serial.println(encoder_val);
  //   last_encoder_val = encoder_val;
  // }

  // if(encoder_val == 3) {
  //   if(!triggered) {
  //     triggered = true;
  //     count++;
  //     if(count > 10) {
  //       Serial.println(millis() - last_time);
  //       last_time = millis();
  //       count = 0;
  //     }
  //   }
  // }
  // else {
  //   triggered = false;
  // }
}
