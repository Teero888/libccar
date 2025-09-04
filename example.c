/* example.c - minimal demo for libccar */

#include <math.h>
#include <stdio.h>

#define LCC_IMPLEMENTATION
#include "libccar.h"

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

  /* set up a car from defaults */
  lcc_car_desc_t desc;
  lcc_car_desc_init_defaults(&desc);

  /* tweak some descriptor bits (optional) */
  desc.transmission.type = LCC_TRANS_MANUAL; /* keep manual; we will shift ourselves */
  desc.ecu.auto_clutch   = 1;                /* auto clutch during shifts for simplicity */
  desc.ecu.abs_mode      = LCC_ABS_ON;
  desc.ecu.tc_mode       = LCC_TC_ON;
  desc.ecu.esc_mode      = LCC_ESC_ON;

  /* slight breeze headwind and reasonable ambient */
  desc.environment.ambient_temp_c = 22.0f;
  desc.environment.air_density    = 1.20f;
  desc.environment.wind_world[0]  = -2.0f; /* headwind */
  desc.environment.wind_world[1]  = 0.0f;

  /* create car */
  lcc_car_t *car = lcc_car_create(&desc);
  if(!car) {
    fprintf(stderr, "Failed to create car\n");
    return 1;
  }

  /* subscribe to events */
  lcc_car_set_event_callback(car, on_event, NULL);

  /* optionally auto-generate a more exciting engine from typical datasheet values */
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

  /* select 1st gear (0=R, 1=N, 2=1st) */
  lcc_car_request_gear(car, 2);

  /* drive scenario:
     - 0..10 s: accelerate with throttle ramp, shift up near redline
     - 4..8 s: turn headlights on to show electrical behavior
     - 12..15 s: full brakes (ABS demo)
  */
  {
    float  dt         = 1.0f / 240.0f;
    double next_print = car->car_state.time_s;
    double t_final    = car->car_state.time_s + 16.0;

    while(car->car_state.time_s < t_final) {
      double t   = car->car_state.time_s;
      float  rpm = car->engine_state.rpm;

      /* headlights toggle for fun */
      if(t > 4.0 && t < 8.0) car->elec_state.consumers_headlights = 1;
      else
        car->elec_state.consumers_headlights = 0;

      /* control phases */
      if(t < 10.0) {
        /* accelerate */
        float ramp   = (float)fmin(1.0, (t) / 1.2); /* reach full throttle in ~1.2s */
        ctl.throttle = ramp;
        ctl.brake    = 0.0f;
        ctl.steer    = 0.0f;

        /* shift up near 6400 rpm if not already shifting */
        if(!car->trans_state.shifting && rpm > 6400.0f) lcc_car_shift_up(car);
      } else if(t < 12.0) {
        /* cruise a bit */
        ctl.throttle = 0.25f;
        ctl.brake    = 0.0f;
        ctl.steer    = 0.0f;
      } else if(t < 15.0) {
        /* hard braking */
        ctl.throttle = 0.0f;
        ctl.brake    = 1.0f;
        ctl.steer    = 0.0f;
      } else {
        /* roll to a stop */
        ctl.throttle = 0.0f;
        ctl.brake    = 0.2f;
      }

      lcc_car_set_controls(car, &ctl);
      lcc_car_step(car, dt);

      if((double)car->car_state.time_s >= next_print) {
        print_status(car, &ctl);
        next_print += 0.10; /* 10 Hz status */
      }
    }
  }

  lcc_car_destroy(car);
  return 0;
}
