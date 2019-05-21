#include <ros.h>
#include <std_msgs/Int64.h>
#include <digitalWriteFast.h>

#define encoder0PinA  2
#define encoder0PinB  4

#define encoder1PinA  3
#define encoder1PinB  5

ros::NodeHandle nh;

std_msgs::Int64 encoder0Pos;
std_msgs::Int64 encoder1Pos;
std_msgs::Int64 force;
ros::Publisher p("ODROID", &encoder0Pos);
ros::Publisher q("ODROID", &encoder1Pos);
ros::Publisher r("ODROID", &force);

int pressurePin = A1;

void setup() {
  //Initiate Rosserial communication
  nh.initNode();
  nh.advertise(p);
  nh.advertise(q);
  nh.advertise(r);
  
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);       // turn on pull-up resistor


  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoder0, CHANGE);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoder1, CHANGE);

  pinMode(pressurePin, INPUT);
  
  Serial.begin (115200);
}

void loop() {
  force.data = analogRead(pressurePin);
  r.publish(&force);

}

void doEncoder0() {
  // look for a low-to-high on channel A
  if (digitalReadFast(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalReadFast(encoder0PinB) == LOW) {
      encoder0Pos.data = encoder0Pos.data + 1;         // CW
    }
    else {
      encoder0Pos.data = encoder0Pos.data - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalReadFast(encoder0PinB) == HIGH) {
      encoder0Pos.data = encoder0Pos.data + 1;          // CW
    }
    else {
      encoder0Pos.data = encoder0Pos.data - 1;          // CCW
    }
  }
  p.publish(&encoder0Pos);
  // use for debugging - remember to comment out
}

void doEncoder1() {
    // look for a low-to-high on channel A
  if (digitalReadFast(encoder1PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalReadFast(encoder1PinB) == LOW) {
      encoder1Pos.data = encoder1Pos.data + 1;         // CW
    }
    else {
      encoder1Pos.data = encoder1Pos.data - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalReadFast(encoder1PinB) == HIGH) {
      encoder1Pos.data = encoder1Pos.data + 1;          // CW
    }
    else {
      encoder1Pos.data = encoder1Pos.data - 1;          // CCW
    }
  }
  q.publish(&encoder1Pos);
  // use for debugging - remember to comment out
}
