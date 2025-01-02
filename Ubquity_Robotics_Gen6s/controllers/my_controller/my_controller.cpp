// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 64
#define MAX_SPEED 6.28
#include <webots/Camera.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  // int timeStep = (int)robot->getBasicTimeStep();
  
  Motor *leftMotor = robot->getMotor("motor_1");
   Motor *rightMotor = robot->getMotor("motor_2");
   
   DistanceSensor *ds_left = robot->getDistanceSensor("ds_left");
   DistanceSensor *ds_right = robot->getDistanceSensor("ds_right");
   
   
   
   Camera *cm;
   cm=robot->getCamera("camera");
   cm->enable(TIME_STEP);
   
   ds_left->enable(TIME_STEP);
   ds_right->enable(TIME_STEP);
   
    leftMotor ->setPosition(INFINITY);
     rightMotor ->setPosition(INFINITY);
     
     leftMotor -> setVelocity(0.0);
     rightMotor -> setVelocity(0.0);

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    
    double ds_left_val = ds_left->getValue();
    double ds_right_val = ds_right->getValue();
    
    
    std::cout<< "Distance Sensor left:"<<ds_left_val<<std::endl;
    std::cout<< "Distance Sensor right:"<<ds_right_val<<std::endl;
    
    leftMotor -> setVelocity(MAX_SPEED);
     rightMotor -> setVelocity(MAX_SPEED);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
