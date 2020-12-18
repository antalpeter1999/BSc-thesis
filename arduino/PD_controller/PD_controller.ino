#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "PinChangeInt.h"             // interrupt
#include <stdio.h>
#include <math.h>

// pin layout
#define enc_pend1 4                   // pendulum angle
#define enc_pend2 5
#define enc_cart1 2                   // cart position
#define enc_cart2 3
#define rotR 12                       // motor direction
#define rotL 13
#define pwm 10                        // PWM signal
#define integral_null 8               // push button
#define poti 5
#define MAXPWM 255                    // maximal applicable PWM (absolute max: 255)
#define MAXCART 2700                  // maximal cart position
#define calibn 7                      // calibrating angle null point (push button)
#define calibp 11
#define xnull 9
#define initial 6

volatile int lastEncoded_pend = 0;
volatile int lastEncoded_cart = 0;

volatile long sum_pend = 0;           // pendulum angle
long last_sum_pend = 0;
volatile long sum_cart = 0;           // cart position
double last_sum_cart = 0;
long lastencoderValue_pend = 0;
long lastencoderValue_cart = 0;

int currPos = 0;
int lastPos = 0;
unsigned long currTime = 0;
unsigned long currTimeM = 0;
unsigned long lastTime = 0;
double sampleTime = 20;                 // sampling time

// PD parameters
double p_phi = -2750.19;
double d_phi = -12.51;
double p_x = -2806.35;
double d_x = -68.51;

// mapping to SI
double SI2ARD_phi = 2291.8;            // 7200/3.141592;
double SI2ARD_x = 28063.492;           // 8840/0.315;

int pwmValue;
bool R = 0;
int potiVal = 0;
char datas[100];
double u = 0;
double q[4];
bool safezone = 1;
char printing[30];                   // for serial communication

void setup() {
  Serial.begin (9600);

  pinMode(enc_pend1, INPUT_PULLUP);
  pinMode(enc_pend2, INPUT_PULLUP);
  pinMode(enc_cart1, INPUT);
  pinMode(enc_cart2, INPUT);
  pinMode(integral_null, INPUT);
  pinMode(poti, INPUT);
  pinMode(calibp, INPUT);
  pinMode(calibn, INPUT);
  pinMode(xnull, INPUT_PULLUP);
  pinMode(initial, OUTPUT);

  // activating pullup resistors
  digitalWrite(enc_pend1, HIGH);
  digitalWrite(enc_pend2, HIGH);
  digitalWrite(enc_cart1, HIGH);
  digitalWrite(enc_cart2, HIGH);
  digitalWrite(xnull, HIGH);

  // attaching interrupts for optical encoders
  attachPinChangeInterrupt(enc_pend1, updateEncoderPend, CHANGE);
  attachPinChangeInterrupt(enc_pend2, updateEncoderPend, CHANGE);
  attachInterrupt(0, updateEncoderCart, CHANGE);
  attachInterrupt(1, updateEncoderCart, CHANGE);

  // setup of motor driver pins
  pinMode(pwm, OUTPUT);
  pinMode(rotR, OUTPUT);
  pinMode(rotL, OUTPUT);

  pwmValue = 0;
  Left();
  analogWrite(pwm, pwmValue);
}

void loop() {
  currTime = micros();
  if (currTime - lastTime >= sampleTime * 1000) {     // sample time is milliseconds, currTime is microseconds
    // the PD controller itself:
    u = p_phi / SI2ARD_phi * double(sum_pend - 7200) + d_phi / SI2ARD_phi * double(sum_cart - last_sum_cart) / (sampleTime / 1000)
               + p_x / SI2ARD_x * double(sum_cart) + d_x / SI2ARD_x * double(sum_cart - last_sum_cart) / (sampleTime / 1000);

    pwmValue = u * 255 / 40;

    last_sum_pend = sum_pend;
    last_sum_cart = sum_cart;
    lastTime = currTime;
    if (pwmValue >= 0) Right();                       // direction of motor rotation
    else Left();
    pwmValue = abs(pwmValue);
    if (pwmValue > MAXPWM) pwmValue = MAXPWM;         // maximal PWM value
    if (safezone) analogWrite(pwm, pwmValue);         // if cart is within the allowed range
    else {
      analogWrite(pwm, 0);
      u = 0;
      if (abs(sum_cart) < 200) safezone = 1;
    }

    // save states
    q[0] = double(sum_cart) / SI2ARD_x * 1000;
    q[3] = double(sum_pend) / SI2ARD_phi;
    q[1] = double(sum_cart - last_sum_cart) / SI2ARD_x / (sampleTime / 1000) * 1000;
    q[2] = double(sum_pend - last_sum_pend) / SI2ARD_phi / (sampleTime / 1000);
    sprintf(printing, "%d\t%d\t%d\t", int(q[3] * 1000), int(q[0] * 10), int(u * 100));
    Serial.println(printing);                         // send states through serial communication

    if (q[3] > 3.215 && q[3] < 3.225) {               // initial condition for the angle (LED setup)
      digitalWrite(initial, HIGH);
    }
    else {
      digitalWrite(initial, LOW);
    }
  }

  // push buttons for calibrating the null point of the pendulum
  if (digitalRead(calibp)) {
    sum_pend = sum_pend + 1;
    delay(300);
  }
  if (digitalRead(calibn)) {
    sum_pend = sum_pend - 1;
    delay(300);
  }

  // push button for making the cart position zero and resetting safezone
  if (digitalRead(integral_null)) {
    sum_cart = 0;
    safezone = 1;
  }

  // the cart is near the end of the frame
  if (abs(sum_cart) >= MAXCART) {
    analogWrite(pwm, 0);
    safezone = 0;
  }
}

void updateEncoderPend() {
  int MSB = digitalRead(enc_pend1); //MSB = most significant bit
  int LSB = digitalRead(enc_pend2); //LSB = least significant bit
  int encoded = (MSB << 1) | LSB;
  int sum  = (lastEncoded_pend << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    sum_pend ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    sum_pend --;

  lastEncoded_pend = encoded;
}

void updateEncoderCart() {
  int MSB = digitalRead(enc_cart1);
  int LSB = digitalRead(enc_cart2);

  int encoded = (MSB << 1) | LSB;
  int sum  = (lastEncoded_cart << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    sum_cart --;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    sum_cart ++;

  lastEncoded_cart = encoded;
}

void Left()
{
  digitalWrite(rotR, LOW);
  digitalWrite(rotL, HIGH);
  R = 0;
}

void Right() {
  digitalWrite(rotL, LOW);
  digitalWrite(rotR, HIGH);
  R = 1;
}
