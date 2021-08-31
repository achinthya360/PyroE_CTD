// Blocking.pde
// -*- mode: C++ -*-
//
// Shows how to use the blocking call runToNewPosition
// Which sets a new target position and then waits until the stepper has
// achieved it.
//
// Copyright (C) 2009 Mike McCauley
// $Id: Blocking.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

long amplitude = 200; // half revolution for 800 microstep setting of driver
float max_speed = 5000; // maximum possible ~ 5000
float accel = 90000;  // maximum possible ~ 85000

// theoretical calculations assume constant linear stepping acceleration
float freq = sqrt(accel)/80; // f = sqrt(a)/80 -> could be used to back calculate accel
float a = 6400*pow(freq, 2); // a = 6400*f^2

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 3, 2); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(accel);

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
