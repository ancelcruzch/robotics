#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define SPEED 300

WbDeviceTag left_motor, right_motor;

int main() {
  wb_robot_init();

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
  wb_motor_set_velocity(left_motor, 0.00628 * SPEED);
  wb_motor_set_velocity(right_motor, 0.00628 * SPEED);

  while (wb_robot_step(TIME_STEP) != -1) {
    // Robot avanza en línea recta
  }

  wb_robot_cleanup();
  return 0;
}