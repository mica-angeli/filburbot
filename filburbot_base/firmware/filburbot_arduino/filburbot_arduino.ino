#include <ros.h>
#include <filburbot_msgs/CmdDiffVel.h>
#include <filburbot_msgs/Encoders.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 512> nh;
filburbot_msgs::Encoders encoders_msg;
ros::Publisher pub_encoders("encoders", &encoders_msg);

struct Filburbot_Motor {
  Adafruit_DCMotor * motor;
  int motor_pin;
  int encoder1_pin;
  int encoder2_pin;
  long position;
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

Encoder left_enc(4, 5);
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

  // Serial.begin(9600);
  // Serial.println("Filburbot");

  AFMS.begin();

  left.motor->setSpeed(255);
  left.motor->run(BACKWARD);
  right.motor->setSpeed(255);
  right.motor->run(FORWARD);

  // Initialize ROS
  nh.initNode();
  nh.advertise(pub_encoders);
}
unsigned long last_time = millis();


void loop() {
  left.position = left_enc.read();
  right.position = right_enc.read();

  if((millis() - last_time) >= 100) {
    // int speed = (right_position - right.last_position);

    // Serial.println(speed);
    encoders_msg.left = left.position;
    encoders_msg.right = right.position;
    encoders_msg.header.stamp = nh.now();
    pub_encoders.publish(&encoders_msg);

    // right.last_position = right_position;
    last_time = millis();
  }

  nh.spinOnce();
}
