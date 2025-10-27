/* max_speed.c - automatic transmission max speed test for libccar */
#include <math.h>
#include <stdio.h>
#include "libccar.h"
int main(void) {
  printf("libccar version: %s\n", lcc_version_string());
  /* set up a car from defaults */
  lcc_car_desc_t desc;
  lcc_car_desc_init_defaults(&desc);
  /* automatic transmission */
  desc.transmission.type = LCC_TRANS_AUTOMATIC;
  desc.ecu.abs_mode = LCC_ABS_OFF;
  desc.ecu.tc_mode = LCC_TC_OFF;
  desc.ecu.esc_mode = LCC_ESC_OFF;
  desc.environment.wind_world[0] = 0.0f;
  desc.environment.wind_world[1] = 0.0f;
  /* create car */
  lcc_car_t *car = lcc_car_create(&desc);
  if(!car) {
    fprintf(stderr, "Failed to create car\n");
    return 1;
  }
  /* controls */
  lcc_controls_t ctl = { 0 };
  ctl.ignition_switch = 1; /* turn ignition on */
  /* crank engine */
  {
    float dt = 1.0f / 120.0f;
    double t_end = car->car_state.time_s + 3.0;
    ctl.starter = 1;
    while(car->car_state.time_s < t_end && !car->engine_state.running) {
      lcc_car_set_controls(car, &ctl);
      lcc_car_step(car, dt);
    }
    ctl.starter = 0; /* release starter */
    lcc_car_set_controls(car, &ctl);
  }
  if(!car->engine_state.running) {
    fprintf(stderr, "Engine did not start. Exiting.\n");
    lcc_car_destroy(car);
    return 1;
  }
  /* Set to Drive */
  car->trans_state.auto_mode = LCC_AUTO_DRIVE;
  /* Full throttle */
  ctl.throttle = 1.0f;
  float dt = 1.0f / 240.0f;
  float previous_speed = -1.0f;
  int stable_speed_counter = 0;
  printf("Accelerating to max speed...\n");
  while(stable_speed_counter < 24000) { // roughly 100 seconds of stable speed
    lcc_car_set_controls(car, &ctl);
    lcc_car_step(car, dt);
    float current_speed = lcc_car_get_speed_kmh(car);
    if(fabs(current_speed - previous_speed) < 0.01f) stable_speed_counter++;
    else
      stable_speed_counter = 0;
    previous_speed = current_speed;
  }
  printf("\n--- Max Speed Reached ---\n");
  printf("Time: %.2f s\n", car->car_state.time_s);
  printf("Position: x=%.2f, y=%.2f\n", car->car_state.pos_world[0], car->car_state.pos_world[1]);
  printf("Max Speed: %.2f km/h\n", lcc_car_get_speed_kmh(car));
  printf("Engine RPM: %.0f\n", car->engine_state.rpm);
  printf("Gear: %d\n", car->trans_state.gear_index - 1);
  printf("Coolant Temp: %.1f C\n", car->cool_state.coolant_temp_c);
  lcc_car_destroy(car);
  return 0;
}