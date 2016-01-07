#include <Servo.h>

// Pins bind
#define RGP_PIN_PITCH_CONTROL 2
#define RGP_PIN_HEADING_CONTROL 7
#define RGP_PIN_MODE_SWITCH 4

// Max angle values for servos
#define RGP_MIN_PITCH_ANGLE 60
#define RGP_MAX_PITCH_ANGLE 140
//TODO: define heading limits
#define RGP_MIN_HEADING_ANGLE 90
#define RGP_MAX_HEADING_ANGLE 90

#define RGP_MIN_PULSE_WIDTH 1000
#define RGP_MAX_PULSE_WIDTH 2000
#define RGP_MOVING_DELAY 100

Servo pitch; 
Servo heading;
Servo mode;

int pos = 0;

void setup() {
  mode.attach(RGP_PIN_MODE_SWITCH, RGP_MIN_PULSE_WIDTH, RGP_MAX_PULSE_WIDTH);
  mode.write(3);
  
  pitch.attach(RGP_PIN_PITCH_CONTROL, RGP_MIN_PULSE_WIDTH, RGP_MAX_PULSE_WIDTH);
  
  heading.attach(RGP_PIN_HEADING_CONTROL, RGP_MIN_PULSE_WIDTH, RGP_MAX_PULSE_WIDTH);  
}

int isAuthorized(int servo, int angle) {
  return (servo == RGP_PIN_PITCH_CONTROL && (RGP_MIN_PITCH_ANGLE <= angle && angle <= RGP_MAX_PITCH_ANGLE)) ||
         (servo == RGP_PIN_HEADING_CONTROL && (RGP_MIN_HEADING_ANGLE <= angle && angle <= RGP_MAX_HEADING_ANGLE));
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { 
    if (isAuthorized(RGP_PIN_PITCH_CONTROL, pos)) {
      pitch.write(pos);
      delay(RGP_MOVING_DELAY);
    }
    if (isAuthorized(RGP_PIN_HEADING_CONTROL, pos)) {
      heading.write(pos);
      delay(RGP_MOVING_DELAY);
    }
  }
  
  for (pos = 180; pos >= 0; pos -= 1) { 
    if (isAuthorized(RGP_PIN_PITCH_CONTROL, pos)) {
      pitch.write(pos);
      delay(RGP_MOVING_DELAY);
    }
    if (isAuthorized(RGP_PIN_HEADING_CONTROL, pos)) {
      heading.write(pos);
      delay(RGP_MOVING_DELAY);
    }
  }
}

