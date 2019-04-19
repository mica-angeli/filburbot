#include <ros.h>
#include <filburbot_msgs/CmdDiffVel.h>
#include <filburbot_msgs/Encoders.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 256> nh;
filburbot_msgs::Encoders encoders_msg;
ros::Publisher pub_encoders("encoders", &encoders_msg);

struct Filburbot_Motor {
  Adafruit_DCMotor * motor;
  int motor_pin;
  int encoder1_pin;
  int encoder2_pin;
  long position;
  long last_position;
  int speed;
  int setpoint;
  int last_setpoint;
};

Filburbot_Motor left, right;

void Filburbot_Motor_init(Filburbot_Motor *m) {
  m->motor = AFMS.getMotor(m->motor_pin);
  pinMode(m->encoder1_pin, INPUT);
  pinMode(m->encoder2_pin, INPUT);
  m->position = 0;
  m->last_position = 0;
  m->speed = 0;
  m->setpoint = 0;
  m->last_setpoint = 0;
}

void Filburbot_Motor_setSpeed(Filburbot_Motor *m, int speed) {
  if(speed > 0) {
    m->motor->run(FORWARD);
  }
  else if (speed < 0) {
    m->motor->run(BACKWARD);
  }
  else {
    m->motor->run(RELEASE);
  }

  m->motor->setSpeed(abs(speed));
}

Encoder left_enc(4, 5);
Encoder right_enc(6, 7);

void cmdDiffCallback(const filburbot_msgs::CmdDiffVel& msg) {
  left.setpoint = msg.left_speed;
  right.setpoint = msg.right_speed;
}

ros::Subscriber<filburbot_msgs::CmdDiffVel> sub_cmddiff("cmd_diff", &cmdDiffCallback );

void setup() {
  left.motor_pin = 3;
  left.encoder1_pin = 5;
  left.encoder2_pin = 4;
  Filburbot_Motor_init(&left);

  right.motor_pin = 1;
  right.encoder1_pin = 7;
  right.encoder2_pin = 6;
  Filburbot_Motor_init(&right);

  AFMS.begin();

  left.motor->setSpeed(0);
  left.motor->run(BACKWARD);
  right.motor->setSpeed(0);
  right.motor->run(FORWARD);

  // Initialize ROS
  nh.initNode();
  nh.advertise(pub_encoders);
  nh.subscribe(sub_cmddiff);
}
unsigned long last_time = millis();


void loop() {
  left.position = left_enc.read();
  right.position = right_enc.read();

  if((millis() - last_time) >= 100) {
    left.speed = (left.position - left.last_position);
    right.speed = (right.position - right.last_position);

    publishEncoders();

    Filburbot_Motor_setSpeed(&left, left.setpoint);
    Filburbot_Motor_setSpeed(&right, right.setpoint);

    left.last_position = left.position;
    right.last_position = right.position;

    last_time = millis();
  }

  nh.spinOnce();
}

void publishEncoders() {
  encoders_msg.left_speed = left.speed;
  encoders_msg.right_speed = right.speed;
  encoders_msg.left_position = left.position;
  encoders_msg.right_position = right.position;
  encoders_msg.header.stamp = nh.now();
  pub_encoders.publish(&encoders_msg);
}
