/*
 * car_sim_step.c - the VCU vehicle dynamics layer (Core/Src/vehicle_dynamics.c), one control period per call, for
 * tools/car_sim.py. Built with the firmware toolchain and run on the emulated Cortex-M7 (Unicorn), as
 * tests/vehicle_dynamics/run_tests.py does; car_sim.py owns the car model around it: it calls Reset_Handler() once,
 * then per period writes sim_input, calls sim_step() and reads sim_output.
 *
 * Between the layer and the inverters, as throttle_control.c limit_current() with regen allowed by the pack voltage,
 * no inverter fault and no BSPD: THROTTLE_REGEN_ENABLE and the torque rise limit THROTTLE_TORQUE_RISE_MAX_PCT_S.
 */
#include <math.h>
#include <stdint.h>

#include "throttle_control.h"                   // THROTTLE_* switches, friction, rise limit
#include "traction_control_torque_vectoring.h"   // TRACTION_CONTROL_ENABLE, TORQUE_VECTORING_ENABLE
#include "vehicle_dynamics.h"

enum {  // sim_input.flags (car_sim.py FLAG_*)
    SIM_RESET = 1u << 0,                // fresh layer (as throttle_control_reset())
    SIM_TRACTION_CONTROL = 1u << 1,
    SIM_TORQUE_VECTORING = 1u << 2,
    SIM_IMU_ACCELERATIONS = 1u << 3,    // ax / ay valid (THROTTLE_IMU_ACCELERATIONS)
    SIM_IMU_GYRO = 1u << 4,             // yaw rate valid (THROTTLE_IMU_GYRO)
};

typedef struct {  // car_sim.py packs "<10fI"
    float rear_left_rad_s, rear_right_rad_s;  // wheel speed as the last ERPM frame gives it (sign ignored)
    float steering_wheel_angle_rad;           // > 0 = turning left
    float imu_longitudinal_ms2, imu_lateral_ms2, yaw_rate_rad_s;  // car axes, at the IMU
    float throttle_percent, brake_bar;
    float vcu_friction;                       // the friction the VCU assumes, x tyre_LMUX / LMUY (THROTTLE_FRICTION_*)
    float dt_s;
    uint32_t flags;
} sim_input_t;

typedef struct {  // car_sim.py OUTPUTS, same order
    float motor_current_a[2];                 // rear left, rear right: after the glue limits = what the inverters get
    float wheel_force_allocated_n[2];         // layer step 8, before traction control
    float wheel_force_n[2];                   // layer final
    float traction_control_torque_factor[2];
    float slip_ratio[2];                      // the layer's estimate
    float peak_slip_drive[2];
    float driver_force_request_n;
    float yaw_rate_reference_rad_s, yaw_rate_rad_s;
    float yaw_moment_request_nm, yaw_moment_delivered_nm;
    float vehicle_speed_mps, sideslip_angle_rad;
    float torque_vectoring_active;            // 0 / 1
} sim_output_t;

sim_input_t sim_input;
sim_output_t sim_output;
uint32_t sim_firmware_flags;                  // the #define switches, as SIM_* flags (set by Reset_Handler)
float sim_firmware_friction;                  // THROTTLE_FRICTION_WET or _DRY, as THROTTLE_WET_TRACK picks

static vehicle_dynamics_t vehicle;
static float last_current_a[2];

/* throttle_control.c limit_current(), the parts that apply here */
static float limit_current(float current_a, float last_a, float dt_s) {
    if (current_a < 0.0f && !THROTTLE_REGEN_ENABLE) current_a = 0.0f;
    if (THROTTLE_TORQUE_RISE_MAX_PCT_S > 0) {
        const float maximum_rise = THROTTLE_TORQUE_RISE_MAX_PCT_S * 0.01f * vehicle.config.motor_maximum_drive_current_a * dt_s;
        current_a = fminf(current_a, fmaxf(last_a, 0.0f) + maximum_rise);
    }
    return current_a;
}

void sim_step(void) {
    const sim_input_t *in = &sim_input;
    if (in->flags & SIM_RESET) {
        vehicle_dynamics_init(&vehicle, &VEHICLE_DYNAMICS_T26_CONFIG);
        last_current_a[0] = last_current_a[1] = 0.0f;
    }
    // what throttle_control_reset() takes from the #defines, from the GUI instead (live)
    vehicle.config.tyre_LMUX = VEHICLE_DYNAMICS_T26_CONFIG.tyre_LMUX * in->vcu_friction;
    vehicle.config.tyre_LMUY = VEHICLE_DYNAMICS_T26_CONFIG.tyre_LMUY * in->vcu_friction;
    vehicle.config.straight_line_grip_scale = THROTTLE_STRAIGHT_LINE_BOOST;
    vehicle.config.traction_control_enable = (in->flags & SIM_TRACTION_CONTROL) != 0;
    vehicle.config.torque_vectoring_enable = (in->flags & SIM_TORQUE_VECTORING) != 0;

    const vehicle_dynamics_input_t input = {
        .wheel_speed_rad_s = {0.0f, 0.0f, in->rear_left_rad_s, in->rear_right_rad_s},  // AQT2 off (THROTTLE_FRONT_WHEEL_SPEEDS 0)
        .rear_wheel_speeds_valid = true,
        .steering_wheel_angle_rad = in->steering_wheel_angle_rad,
        .steering_valid = true,
        .imu_longitudinal_acceleration_ms2 = in->imu_longitudinal_ms2,
        .imu_lateral_acceleration_ms2 = in->imu_lateral_ms2,
        .imu_valid = (in->flags & SIM_IMU_ACCELERATIONS) != 0,
        .yaw_rate_rad_s = in->yaw_rate_rad_s,
        .yaw_rate_valid = (in->flags & SIM_IMU_GYRO) != 0,
        .throttle_percent = in->throttle_percent,
        .throttle_valid = true,
        .brake_pressure_bar = in->brake_bar,
        .brake_pressure_valid = true,
    };
    const vehicle_dynamics_torque_t torque = vehicle_dynamics_compute(&vehicle, &input, in->dt_s);
    last_current_a[0] = limit_current(torque.rear_left.motor_current_a, last_current_a[0], in->dt_s);
    last_current_a[1] = limit_current(torque.rear_right.motor_current_a, last_current_a[1], in->dt_s);

    sim_output_t *out = &sim_output;
    for (int side = 0; side < 2; side++) {
        const int wheel = WHEEL_REAR_LEFT + side;
        out->motor_current_a[side] = last_current_a[side];
        out->wheel_force_allocated_n[side] = vehicle.wheel_force_allocated_n[wheel];
        out->wheel_force_n[side] = vehicle.wheel_force_n[wheel];
        out->traction_control_torque_factor[side] = vehicle.traction_control_torque_factor[wheel];
        out->slip_ratio[side] = vehicle.slip_ratio[wheel];
        out->peak_slip_drive[side] = vehicle.peak_slip_drive[wheel];
    }
    out->driver_force_request_n = vehicle.driver_force_request_n;
    out->yaw_rate_reference_rad_s = vehicle.yaw_rate_reference_rad_s;
    out->yaw_rate_rad_s = vehicle.yaw_rate_rad_s;
    out->yaw_moment_request_nm = vehicle.yaw_moment_request_nm;
    out->yaw_moment_delivered_nm = vehicle.yaw_moment_delivered_nm;
    out->vehicle_speed_mps = vehicle.vehicle_speed_mps;
    out->sideslip_angle_rad = vehicle.sideslip_angle_rad;
    out->torque_vectoring_active = vehicle.torque_vectoring_active ? 1.0f : 0.0f;
}

/* car_sim.py stops the emulator when execution gets here (the return address of every call) */
void sim_return(void) {
    for (;;) {
    }
}

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

/* Called once: what the startup code does (FPU on, .data, .bss), then the firmware switches for the GUI defaults */
void Reset_Handler(void) {
    *(volatile uint32_t *)0xE000ED88u |= 0xFu << 20;  // CPACR: CP10 + CP11 full access = FPU on
    __asm volatile("dsb\n\tisb");
    for (uint32_t *source = &_sidata, *target = &_sdata; target < &_edata;) *target++ = *source++;
    for (uint32_t *target = &_sbss; target < &_ebss;) *target++ = 0;
    sim_firmware_flags = (TRACTION_CONTROL_ENABLE ? SIM_TRACTION_CONTROL : 0u) | (TORQUE_VECTORING_ENABLE ? SIM_TORQUE_VECTORING : 0u)
                         | (THROTTLE_IMU_ACCELERATIONS ? SIM_IMU_ACCELERATIONS : 0u) | (THROTTLE_IMU_GYRO ? SIM_IMU_GYRO : 0u);
    sim_firmware_friction = THROTTLE_WET_TRACK ? THROTTLE_FRICTION_WET : THROTTLE_FRICTION_DRY;
}

/* Not an interrupt table: the linker script (tests/vehicle_dynamics/test.ld) KEEPs .isr_vector, so the entry points
 * car_sim.py calls survive --gc-sections */
__attribute__((section(".isr_vector"), used)) void (*const sim_entry_points[])(void) = {Reset_Handler, sim_step, sim_return};
