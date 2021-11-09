// Blocking.pde
// -*- mode: C++ -*-
//
// Shows how to use the blocking call runToNewPosition
// Which sets a new target position and then waits until the stepper has
// achieved it.
//  
// Copyright (C) 2009 Mike McCauley
// $Id: Blocking.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#define kinematic_coeff 3.35
const float max_speed = 5000; // maximum possible ~ 5000

float microsteps = 800; // from driver
float theta_max = 90; // degrees
long amplitude = microsteps/360*theta_max; // microsteps needed to reach wanted angle
//float accel = 90000;  // maximum possible ~ 85000
float freq_wanted = 0.2;

// theoretical calculations assume constant linear stepping acceleration
//float freq = 3.35 * sqrt(accel/(theta_max * microsteps)); // could be used to back calculate accel
float a = pow(freq_wanted*sqrt(theta_max*microsteps)/kinematic_coeff, 2); // a = (f * sqrt(theta * microsteps) / 3.35) ^ 2

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 3, 2); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(a);

  // following pins 4 and 5 could also just be GNDed
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
}

void loop()
{
  stepper.runToNewPosition(amplitude);
  stepper.runToNewPosition(-amplitude);
}
