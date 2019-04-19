#include <ros.h>
#include <filburbot_msgs/CmdDiffVel.h>
#include <filburbot_msgs/Encoders.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

#include <FastPID.h>

FastPID left_pid(0.0, 1.0, 0.0, 10, 9, true);
FastPID right_pid(0.0, 1.0, 0.0, 10, 9, true);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 256> nh;
filburbot_msgs::Encoders encoders_msg;
ros::Publisher pub_encoders("encoders", &encoders_msg);

struct Filburbot_Motor {
  Adafruit_DCMotor * motor;
  long position;
  long last_position;
  int speed;
  int setpoint;
  int speed_command;
  int last_speed_command;
};

Filburbot_Motor left, right;

void Filburbot_Motor_init(Filburbot_Motor *m, int motor_pin, int encoder1_pin, int encoder2_pin) {
  m->motor = AFMS.getMotor(motor_pin);
  pinMode(encoder1_pin, INPUT);
  pinMode(encoder2_pin, INPUT);
  m->position = 0;
  m->last_position = 0;
  m->speed = 0;
  m->setpoint = 0;
  m->speed_command = 0;
  m->last_speed_command = 0;
}

void Filburbot_Motor_setSpeed(Filburbot_Motor *m, int speed_command) {
  if(speed_command == m->last_speed_command) {
    return;
  }

  if(speed_command > 0) {
    m->motor->run(BACKWARD);
  }
  else if (speed_command < 0) {
    m->motor->run(FORWARD);
  }
  else {
    m->motor->run(RELEASE);
  }

  m->motor->setSpeed(abs(speed_command) > 255 ? 255 : abs(speed_command));
  m->last_speed_command = speed_command;
}

Encoder left_enc(4, 5);
Encoder right_enc(6, 7);

unsigned long last_command = millis();

void cmdDiffCallback(const filburbot_msgs::CmdDiffVel& msg) {
  left.setpoint = msg.left_speed;
  right.setpoint = msg.right_speed;
  last_command = millis();
}

ros::Subscriber<filburbot_msgs::CmdDiffVel> sub_cmddiff("cmd_diff", &cmdDiffCallback );

void setup() {
  Filburbot_Motor_init(&left, 3, 5, 4);
  Filburbot_Motor_init(&right, 1, 7, 6);

  AFMS.begin();

  Filburbot_Motor_setSpeed(&left, 0);
  Filburbot_Motor_setSpeed(&right, 0);

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


    if((millis() - last_command) >= 1000) {
      // Stop motors and reset PIDs if command is dropped
      left.speed_command = 0;
      right.speed_command = 0;
      left_pid.clear();
      right_pid.clear();
    }
    else {
      // Compute commands from PID
      left.speed_command = left_pid.step(left.setpoint, left.speed);
      right.speed_command = right_pid.step(right.setpoint, right.speed);
    }

    Filburbot_Motor_setSpeed(&left, left.speed_command);
    Filburbot_Motor_setSpeed(&right, right.speed_command);

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
