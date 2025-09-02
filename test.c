/* libccar_test.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define LCC_IMPLEMENTATION
#include "libccar.h"

/* simple event recorder */
typedef struct test_events_s {
    lcc_event_t evts[256];
    int count;
} test_events_t;

static void on_event(const lcc_event_t* evt, void* user)
{
    test_events_t* te = (test_events_t*)user;
    if (te && te->count < (int)(sizeof(te->evts)/sizeof(te->evts[0]))) {
        te->evts[te->count++] = *evt;
    }
    const char* name = "event";
    switch (evt->type) {
        case LCC_EVENT_ENGINE_START:   name = "ENGINE_START"; break;
        case LCC_EVENT_ENGINE_STOP:    name = "ENGINE_STOP"; break;
        case LCC_EVENT_GEAR_CHANGE:    name = "GEAR_CHANGE"; break;
        case LCC_EVENT_ENGINE_STALL:   name = "ENGINE_STALL"; break;
        case LCC_EVENT_ABS_ACTIVE:     name = "ABS_ACTIVE"; break;
        case LCC_EVENT_TC_ACTIVE:      name = "TC_ACTIVE"; break;
        case LCC_EVENT_ESC_ACTIVE:     name = "ESC_ACTIVE"; break;
        case LCC_EVENT_OVERHEAT:       name = "OVERHEAT"; break;
        case LCC_EVENT_FUEL_STARVATION:name = "FUEL_STARVATION"; break;
        default: break;
    }
    printf("[%.3f] %s i32=%d f32=%.3f\n", evt->time_s, name, evt->data_i32, evt->data_f32);
}

/* helper: print a quick line of core telemetry */
static void print_line(const lcc_car_t* car, const char* tag)
{
    lcc_car_state_t cs; lcc_engine_state_t es; lcc_transmission_state_t ts;
    lcc_aero_state_t as; lcc_brake_state_t bs;
    lcc_car_get_state(car, &cs);
    lcc_car_get_engine_state(car, &es);
    lcc_car_get_transmission_state(car, &ts);
    lcc_car_get_aero_state(car, &as);
    lcc_car_get_brake_state(car, &bs);
    printf("%s t=%.2f v=%.2f m/s yaw=%.2fdeg rpm=%.0f gear=%d thr=%.0f%% DF(%.0f/%.0f) ABSmod[0]=%.2f\n",
        tag, cs.time_s, cs.speed_mps, lcc_rad_to_deg(cs.yaw_rad),
        es.rpm, ts.gear_index, 100.0f*es.throttle_effective,
        as.downforce_n_front, as.downforce_n_rear, bs.line_pressure_kpa /* placeholder, or use abs_mod if exposed */);
}

/* step helper with clamped dt */
static lcc_result_t step_seconds(lcc_car_t* car, double seconds, const lcc_controls_t* ctrl)
{
    const float dt = 1.0f / 200.0f; /* 200 Hz */
    int steps = (int)ceil(seconds / dt);
    lcc_controls_t c = *ctrl;
    for (int i = 0; i < steps; ++i) {
        lcc_car_set_controls(car, &c);
        lcc_result_t r = lcc_car_step(car, dt);
        if (r != LCC_OK) return r;
    }
    return LCC_OK;
}

/* build a simple engine map (WOT torque and friction) */
static void fill_engine_maps(lcc_engine_desc_t* eng)
{
    static const lcc_curve1d_point_t wot_pts[] = {
        { 800, 120 }, { 1200, 150 }, { 1800, 175 }, { 2400, 195 },
        { 3000, 210 }, { 3600, 220 }, { 4200, 225 }, { 4800, 225 },
        { 5400, 215 }, { 6000, 200 }, { 6500, 180 }
    };
    static const lcc_curve1d_point_t fric_pts[] = {
        { 0, 8 }, { 1000, 10 }, { 2000, 14 }, { 3000, 18 },
        { 4000, 24 }, { 5000, 32 }, { 6000, 42 }, { 7000, 54 }
    };
    static const lcc_curve1d_point_t thr_pts[] = { {0.0f, 0.0f}, {1.0f, 1.0f} };

    eng->wot_torque_nm_vs_rpm.points = wot_pts;
    eng->wot_torque_nm_vs_rpm.count  = (int)(sizeof(wot_pts)/sizeof(wot_pts[0]));

    eng->friction_torque_nm_vs_rpm.points = fric_pts;
    eng->friction_torque_nm_vs_rpm.count  = (int)(sizeof(fric_pts)/sizeof(fric_pts[0]));

    eng->throttle_map.points = thr_pts;
    eng->throttle_map.count  = (int)(sizeof(thr_pts)/sizeof(thr_pts[0]));
}

/* run-through test, returns 0 on success */
int lcc_run_self_test(void)
{
    printf("libccar version: %s\n", lcc_version_string());

    /* 1) Build a default car */
    lcc_car_desc_t desc;
    lcc_car_desc_init_defaults(&desc);
    strncpy(desc.chassis.name, "lcc test car", sizeof(desc.chassis.name)-1);

    /* steerable fronts, driven fronts (FWD by default) */
    desc.driveline.layout = LCC_LAYOUT_FWD;
    desc.transmission.type = LCC_TRANS_MANUAL;
    desc.transmission.auto_upshift_rpm = 6200.0f;
    desc.transmission.auto_downshift_rpm = 1200.0f;

    fill_engine_maps(&desc.engine);

    /* optionally tweak tires for more dynamic response */
    for (int i = 0; i < desc.wheel_count; ++i) {
        desc.tires[i].mu_nominal = 1.1f;
        desc.tires[i].pressure_kpa = 230.0f;
    }

    /* environment */
    lcc_environment_init_defaults(&desc.environment);
    desc.environment.global_friction_scale = 1.0f;
    desc.environment.ambient_temp_c = 22.0f;

    /* 2) Create car */
    lcc_car_t* car = lcc_car_create(&desc);
    if (!car) { fprintf(stderr, "Failed to create car\n"); return -1; }

    /* 3) Subscribe to events */
    test_events_t ev = {0};
    lcc_car_set_event_callback(car, on_event, &ev);

    /* 4) Start engine */
    if (lcc_car_engine_start(car) != LCC_OK) {
        fprintf(stderr, "Engine start failed\n");
        lcc_car_destroy(car);
        return -2;
    }

    /* 5) Quick utility checks */
    float bmin[2], bmax[2];
    lcc_car_get_local_bounds(car, bmin, bmax);
    printf("local bounds: min(%.2f,%.2f) max(%.2f,%.2f)\n", bmin[0], bmin[1], bmax[0], bmax[1]);
    float wheel_pos[8][2];
    int wc = lcc_car_get_wheel_local_positions(car, wheel_pos, 8);
    for (int i = 0; i < wc; ++i)
        printf("wheel[%d] local pos = (%.3f, %.3f)\n", i, wheel_pos[i][0], wheel_pos[i][1]);

    /* 6) Drive phases */
    lcc_controls_t c = {0};
    c.ignition_switch = 1;
    c.starter = 0;
    c.clutch = 0.0f;
    c.steer = 0.0f;
    c.throttle = 0.2f;

    /* phase A: gentle rollout 1.5s */
    step_seconds(car, 1.5, &c); print_line(car, "phaseA");

    /* ramp throttle to 1.0 and hold for 3s to trigger upshifts/TC */
    c.throttle = 1.0f;
    step_seconds(car, 3.0, &c); print_line(car, "phaseB");

    /* phase C: induce some yaw error by steering for 1.5s to possibly see ESC */
    c.steer = 0.5f;
    step_seconds(car, 1.5, &c); print_line(car, "phaseC");

    /* phase D: hard braking 2s to trigger ABS */
    c.steer = 0.0f;
    c.throttle = 0.0f;
    c.brake = 1.0f;
    step_seconds(car, 2.0, &c); print_line(car, "phaseD");

    /* back to coasting 1s */
    c.brake = 0.0f;
    step_seconds(car, 1.0, &c); print_line(car, "phaseE");

    /* 7) Unit helpers sanity */
    float d = 90.0f;
    float r = lcc_deg_to_rad(d);
    float d2 = lcc_rad_to_deg(r);
    printf("units: %.2f deg -> %.3f rad -> %.2f deg\n", d, r, d2);

    /* 8) Wrap up */
    printf("events captured: %d\n", ev.count);
    lcc_car_destroy(car);
    return 0;
}

int main(void)
{
    int rc = lcc_run_self_test();
    printf("self-test %s (rc=%d)\n", rc == 0 ? "PASSED" : "FAILED", rc);
    return rc;
}