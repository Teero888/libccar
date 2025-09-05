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

static const char *evt_name(lcc_event_type_t t) {
  switch(t) {
  case LCC_EVENT_ENGINE_START: return "ENGINE_START";
  case LCC_EVENT_ENGINE_STOP: return "ENGINE_STOP";
  case LCC_EVENT_GEAR_CHANGE: return "GEAR_CHANGE";
  case LCC_EVENT_ENGINE_STALL: return "ENGINE_STALL";
  case LCC_EVENT_OVERHEAT: return "OVERHEAT";
  case LCC_EVENT_FUEL_STARVATION: return "FUEL_STARVATION";
  default: return "UNKNOWN";
  }
}

static void on_event(const lcc_event_t *evt, void *user) {
  (void)user;
  printf("[%.3f] EVENT: %s", (double)evt->time_s, evt_name(evt->type));
  if(evt->type == LCC_EVENT_GEAR_CHANGE) printf(" -> gear=%d", evt->data_i32);
  else if(evt->type == LCC_EVENT_OVERHEAT)
    printf(" -> coolant=%.1f C", evt->data_f32);
  printf("\n");
}

static void print_status(const lcc_car_t *car, const lcc_controls_t *ctrl) {
  const lcc_car_state_t *cs = &car->car_state;

  int         gear     = car->trans_state.gear_index; /* 0=R, 1=N, 2..=forward */
  const char *gear_str = "N";
  char        buf[8]   = { 0 };
  if(gear == LCC_GEAR_REVERSE) gear_str = "R";
  else if(gear == LCC_GEAR_NEUTRAL)
    gear_str = "N";
  else {
    snprintf(buf, sizeof(buf), "%d", gear - 1);
    gear_str = buf;
  }

  float speed_kmh = cs->speed_mps * 3.6f;
  printf("[%.2f] v=%.1fkm/h  gear=%s  rpm=%.0f  thr=%.0f%% brake=%.0f%%  Vbus=%.2fV  Ialt=%.1fA  Ibatt=%.1fA  Coolant=%.1fC\n", (double)cs->time_s, speed_kmh, gear_str, car->engine_state.rpm, ctrl->throttle * 100.0f, ctrl->brake * 100.0f, car->elec_state.bus_voltage_v, car->elec_state.alt_current_a, car->elec_state.batt_current_a, car->cool_state.coolant_temp_c);
}

int main(void) {
  printf("libccar version: %s\n", lcc_version_string());

  lcc_car_desc_t desc;
  lcc_car_desc_init_defaults(&desc);

  desc.transmission.type = LCC_TRANS_MANUAL; 
  desc.ecu.auto_clutch   = 1;  
  desc.ecu.abs_mode      = LCC_ABS_ON;
  desc.ecu.tc_mode       = LCC_TC_ON;
  desc.ecu.esc_mode      = LCC_ESC_ON;

  desc.environment.ambient_temp_c = 22.0f;
  desc.environment.air_density    = 1.20f;
  desc.environment.wind_world[0]  = -2.0f;
  desc.environment.wind_world[1]  = 0.0f;

  /* create car */
  lcc_car_t *car = lcc_car_create(&desc);
  if(!car) {
    fprintf(stderr, "Failed to create car\n");
    return 1;
  }

  lcc_car_set_event_callback(car, on_event, NULL);

  {
    lcc_engine_simple_spec_t spec;
    lcc_engine_simple_spec_init_defaults(&spec);
    spec.rated_power_kw   = 220.0f; /* ~300 hp */
    spec.rated_power_rpm  = 6200.0f;
    spec.redline_rpm      = 6800.0f;
    spec.idle_rpm         = 850.0f;
    spec.forced_induction = LCC_FI_TURBO;
    spec.boost_target_kpa = 110.0f; /* ~1.1 bar gauge */

    lcc_result_t r = lcc_car_generate_engine_from_simple_spec(car, &spec);
    if(r != LCC_OK) fprintf(stderr, "Engine generation failed (%d)\n", r);
  }

  /* controls */
  lcc_controls_t ctl  = { 0 };
  ctl.ignition_switch = 1; /* turn ignition on */

  /* headlights off to start; we can toggle later */
  car->elec_state.consumers_headlights = 0;

  /* crank engine until it catches (or up to 3 s) */
  {
    float  dt    = 1.0f / 120.0f;
    double t_end = car->car_state.time_s + 3.0;
    ctl.starter  = 1;
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

  /* let it idle a bit */
  {
    float dt = 1.0f / 120.0f;
    for(int i = 0; i < 120; ++i) {
      lcc_car_set_controls(car, &ctl);
      lcc_car_step(car, dt);
    }
  }

  ctl.throttle  = 1.0f;
  lcc_car_shift_up(car);
  clock_t clock = timer_start();

  for(int i = 0; i < 1e6; ++i) {
    lcc_car_set_controls(car, &ctl);
    lcc_car_step(car, 1.0f / 240.0f);
  }
  double time = timer_end(clock);
  printf("Took %f for 1e6 ticks. %f TPS\n", time, 1e6 / time);
  printf("Speed: %.1f km/h, RPM: %.0f\n", lcc_car_get_speed_kmh(car), car->engine_state.rpm);

  lcc_car_destroy(car);
  return 0;
}
