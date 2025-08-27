#include <stdio.h>
#include <time.h>
#define LIBCCAR_IMPLEMENTATION
#include "../libccar.h"

clock_t timer_start() {
  return clock();
}

double timer_end(clock_t start_time) {
  clock_t end_time = clock();
  return (double)(end_time - start_time) / CLOCKS_PER_SEC;
}

int main(void) {
  lcc_car_t car = lcc_car_create(LCC_PRESET_SPORTS);

  lcc_car_set_keypos(&car, LCC_KEY_RUN);
  lcc_car_set_ignition(&car, LCC_IGNITION_ON);
  for(int i = 0; i < 120; ++i) lcc_car_update(&car, 1.0f / 120.0f); /* simulate 1 second at 120Hz to start the engine */
  lcc_car_set_ignition(&car, LCC_IGNITION_OFF);

  for(int i = 0; i < 60; ++i) {
    lcc_car_set_inputs(&car, 1.0f, 0.0f, 0.0f, 0.0f); /* full throttle, no brake/steering/clutch */
    lcc_car_update(&car, 1.0f / 120.0f);
  }

  clock_t clock = timer_start();
  lcc_car_set_gear(&car, 1);
  for(int i = 0; i < 1e6; ++i) {
    lcc_car_set_inputs(&car, 1.0f, 0.0f, -1.0f, 0.0f);
    lcc_car_update(&car, 1.0f / 120.0f);
    lcc_car_refuel(&car, 1.0);
    /* printf("s: %.3f, Speed: %.1f km/h, RPM: %.0f, Fuel %.3f\n", i / 120.0, lcc_car_get_speed(&car), lcc_car_get_engine_rpm(&car),
      lcc_car_get_fuel_level_L(&car)); */
  }
  double time = timer_end(clock);
  printf("Took %f for 1e6 ticks. %f TPS\n", time, 1e6 / time);

  printf("Speed: %.1f km/h, RPM: %.0f\n", lcc_car_get_speed(&car), lcc_car_get_engine_rpm(&car));

  lcc_car_destroy(&car);
  return 0;
}
