#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define LEFT 0
#define RIGHT 1

#define NB_GROUND_SENS 3
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

WbDeviceTag gs[NB_GROUND_SENS];
unsigned short gs_value[NB_GROUND_SENS];

WbDeviceTag left_motor, right_motor;

#define FORWARD_SPEED 200
#define K_GS_SPEED 0.4

void LineFollowing(void) {
  int DeltaS = gs_value[GS_RIGHT] - gs_value[GS_LEFT];
  int speed[2];
  
  speed[LEFT] = FORWARD_SPEED - K_GS_SPEED * DeltaS;
  speed[RIGHT] = FORWARD_SPEED + K_GS_SPEED * DeltaS;
  
  wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
  wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
}

int main() {
  int i;
  char name[20];

  wb_robot_init();

  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < NB_GROUND_SENS; i++)
      gs_value[i] = wb_distance_sensor_get_value(gs[i]);

    LineFollowing();
    
    printf("GS: L=%d C=%d R=%d\n", gs_value[GS_LEFT], gs_value[GS_CENTER], gs_value[GS_RIGHT]);
  }

  wb_robot_cleanup();
  return 0;
}