#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>

// Pins bind
#define RGP_PIN_PITCH_CONTROL 2
#define RGP_PIN_HEADING_CONTROL 7
#define RGP_PIN_MODE_SWITCH 4

// Max angle values for servos
#define RGP_DEFAULT_PITCH_ANGLE 90
#define RGP_MIN_PITCH_ANGLE 60
#define RGP_MAX_PITCH_ANGLE 140
//TODO: define heading limits
#define RGP_NEUTRAL_HEADING_ANGLE 90
#define RGP_MIN_HEADING_ANGLE 0
#define RGP_MAX_HEADING_ANGLE 180

#define RGP_DEFAULT_MODE 3
#define RGP_MIN_PULSE_WIDTH 1000
#define RGP_MAX_PULSE_WIDTH 2000
#define RGP_MOVING_DELAY 100

// Ros topics names
#define RGP_TOPIC_PITCH "/gimbal/pitch_control"
#define RGP_TOPIC_HEADING "/gimbal/heading_control"
#define RGP_TOPIC_MODE "/gimbal/mode_switch"

ros::NodeHandle  nh;

Servo pitch; 
Servo heading;
Servo mode;

int currentPitchAngle;

int isAuthorized(int servo, int angle) {
  return (servo == RGP_PIN_PITCH_CONTROL && (RGP_MIN_PITCH_ANGLE <= angle && angle <= RGP_MAX_PITCH_ANGLE)) ||
    (servo == RGP_PIN_HEADING_CONTROL && (RGP_MIN_HEADING_ANGLE <= angle && angle <= RGP_MAX_HEADING_ANGLE));
}

void pitch_handler(const std_msgs::Int16& cmd_msg){
  int newAngle = currentPitchAngle + cmd_msg.data;
  if (isAuthorized(RGP_PIN_PITCH_CONTROL, newAngle)) {
    currentPitchAngle = newAngle;
    pitch.write(newAngle);
  }
}

void heading_handler(const std_msgs::Int16& cmd_msg){
  int newAngle = RGP_NEUTRAL_HEADING_ANGLE + cmd_msg.data;
  if (isAuthorized(RGP_PIN_HEADING_CONTROL, newAngle)) {
    heading.write(newAngle);
    delay(500);
    heading.write(RGP_NEUTRAL_HEADING_ANGLE);
  }
}

void mode_handler(const std_msgs::Int16& cmd_msg){
  if (1 <= cmd_msg.data && cmd_msg.data <= 3) {
    mode.write(cmd_msg.data);
  }
}

ros::Subscriber<std_msgs::Int16> sub_pitch(RGP_TOPIC_PITCH, pitch_handler);
ros::Subscriber<std_msgs::Int16> sub_heading(RGP_TOPIC_HEADING, heading_handler);
ros::Subscriber<std_msgs::Int16> sub_mode(RGP_TOPIC_MODE, mode_handler);

void setup(){
  pinMode(13, OUTPUT);

  mode.attach(RGP_PIN_MODE_SWITCH, RGP_MIN_PULSE_WIDTH, RGP_MAX_PULSE_WIDTH);
  mode.write(RGP_DEFAULT_MODE);
  
  pitch.attach(RGP_PIN_PITCH_CONTROL, RGP_MIN_PULSE_WIDTH, RGP_MAX_PULSE_WIDTH);
  currentPitchAngle = RGP_DEFAULT_PITCH_ANGLE;

  heading.attach(RGP_PIN_HEADING_CONTROL, 1400, 1600);  

  nh.initNode();
  nh.subscribe(sub_pitch);
  nh.subscribe(sub_heading);
  nh.subscribe(sub_mode);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
