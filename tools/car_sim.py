"""
Car simulator - the VCU torque vectoring and traction control on a track, to SEE what each wheel gets.

CONTROLLER = THE FIRMWARE: Core/Src/vehicle_dynamics.c (plus the torque rise limit of throttle_control.c), built with
the firmware toolchain and run on the emulated Cortex-M7 (Unicorn) as tests/vehicle_dynamics/run_tests.py does, one
5 ms control period per call (tools/car_sim_step.c). TV and TC start ON (the point of this tool); IMU, gyro and the
friction the VCU assumes ("Atrito no VCU", THROTTLE_FRICTION_*) start as the #defines. All of them change live.

CAR = A MODEL in this file: two-track, Magic Formula tyres (the .tir coefficients of vehicle_dynamics.c x "Aderencia
real", combined slip), wheel spin, load transfer, aero, hydraulic brakes, motor current -> torque (DTI equation).
The rear wheel speeds reach the VCU as ERPM frames every 33 ms, the gyro and IMU every period. Values marked
PLACEHOLDER are guesses: the behaviour is right, the numbers are not measured.

DRIVER: steers along the centre line (pure pursuit). The pedals are yours (keys, sliders) or the automatic
driver's (speed for the curvature at "g lateral").

    python tools/car_sim.py             # open the simulator
    python tools/car_sim.py --selftest  # 40 s laps without the GUI, TV on and off: on track, no NaN, never above the pedal

Keys: Up / W = throttle 100 %, Down / S = brake (regen from 2 bar), Space = pause, R = restart. Mouse wheel = zoom.
Needs: pip install unicorn numpy; arm-none-eabi-gcc (PATH or STM32CubeIDE, as run_tests.py). tkinter comes with Python.
"""

import math
import re
import struct
import subprocess
import sys
import tempfile
import time
from collections import deque
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tests" / "vehicle_dynamics"))
import run_tests as firmware_build  # noqa: E402  toolchain(), FLAGS, INCLUDES, memory map: the unit tests' build
from unicorn import UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, Uc  # noqa: E402
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_SP, UC_CPU_ARM_CORTEX_M7  # noqa: E402

CONTROL_PERIOD_S = 0.005        # VCU control loop
PLANT_SUBSTEPS = 10             # car model steps per control period (wheel spin is stiff)
ERPM_PERIOD_S = 0.033           # inverter ERPM frames (08-10 log)
GRAVITY = 9.81
AIR_DENSITY = 1.225
SLIP_MINIMUM_SPEED = 3.0        # slip ratio / angle denominators never below this [m/s] (stable at low speed)
LOAD_TIME_CONSTANT_S = 0.05     # chassis pitch / roll lag of the load transfer
# Car model only: the layer has no such values
DRAG_AREA_M2 = 1.3              # PLACEHOLDER Cd * A
ROLLING_RESISTANCE = 0.015      # PLACEHOLDER
WHEEL_INERTIA_KG_M2 = 0.25      # PLACEHOLDER wheel + tyre + hub + disc
MOTOR_INERTIA_KG_M2 = 0.0015    # PLACEHOLDER rotor, x gear ratio^2 at the rear wheels
BRAKE_NM_PER_BAR = (7.5, 7.5, 4.0, 4.0)   # PLACEHOLDER hydraulic, FL FR RL RR: ~1.6 g at 40 bar, 65 % front
STEERING_RATE_RAD_S = 8.0       # driver: steering wheel speed
TRACK_WIDTH_M = 3.5
KEY_BRAKE_BAR = 30.0

# sim_input.flags of car_sim_step.c
FLAG_RESET, FLAG_TRACTION_CONTROL, FLAG_TORQUE_VECTORING, FLAG_IMU_ACCELERATIONS, FLAG_IMU_GYRO = (1 << i for i in range(5))
# sim_output_t of car_sim_step.c, same order
OUTPUTS = ["current_left_a", "current_right_a", "allocated_left_n", "allocated_right_n", "force_left_n", "force_right_n",
           "tc_factor_left", "tc_factor_right", "slip_left", "slip_right", "peak_slip_left", "peak_slip_right",
           "driver_force_request_n", "yaw_rate_reference_rad_s", "yaw_rate_estimate_rad_s", "yaw_moment_request_nm",
           "yaw_moment_delivered_nm", "vehicle_speed_mps", "sideslip_rad", "tv_active"]
WHEELS = ("FL", "FR", "RL", "RR")


def read_layer_config():
    """VEHICLE_DYNAMICS_T26_CONFIG of vehicle_dynamics.c, one value per line -> {name: float | bool | [bool]}"""
    text = (ROOT / "Core" / "Src" / "vehicle_dynamics.c").read_text(encoding="utf-8", errors="replace")
    block = text[text.index("VEHICLE_DYNAMICS_T26_CONFIG = {"):]
    block = block[:block.index("\n};")]
    config = {}
    for name, value in re.findall(r"^\s*\.(\w+) = ([^/\n]+?),\s*(?://.*)?$", block, re.MULTILINE):
        if value.startswith("{"):
            config[name] = [item.strip() == "true" for item in value.strip("{}").split(",")]
        elif value in ("true", "false"):
            config[name] = value == "true"
        else:
            config[name] = float(value.rstrip("f"))
    return config


class Firmware:
    """vehicle_dynamics.c (car_sim_step.c) on the emulated Cortex-M7, its state kept from one period to the next"""

    def __init__(self, layer_source=ROOT / "Core" / "Src" / "vehicle_dynamics.c"):
        build = firmware_build
        sources = [Path(__file__).with_name("car_sim_step.c"), Path(layer_source)]   # another layer_source: try a change
        with tempfile.TemporaryDirectory() as build_dir:
            elf, image = Path(build_dir, "car_sim.elf"), Path(build_dir, "car_sim.bin")
            subprocess.run([build.toolchain("arm-none-eabi-gcc"), "-O0", *build.FLAGS,   # -O0 = the firmware Debug build
                            *(f"-I{ROOT / folder}" for folder in build.INCLUDES), *map(str, sources), "-lm", "-o", str(elf)],
                           check=True)
            subprocess.run([build.toolchain("arm-none-eabi-objcopy"), "-O", "binary", str(elf), str(image)], check=True)
            nm = subprocess.run([build.toolchain("arm-none-eabi-nm"), str(elf)], capture_output=True, text=True, check=True).stdout
            self.symbols = {p[2]: int(p[0], 16) for p in (line.split() for line in nm.splitlines()) if len(p) == 3}
            code = image.read_bytes()
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
        self.cpu.mem_map(build.FLASH, 1024 * 1024)
        self.cpu.mem_map(build.RAM, 512 * 1024)
        self.cpu.mem_map(build.PPB, 1024 * 1024)   # system registers as plain memory (CPACR write)
        self.cpu.mem_write(build.FLASH, code)
        self.call("Reset_Handler")
        self.default_flags = struct.unpack("<I", self.cpu.mem_read(self.symbols["sim_firmware_flags"], 4))[0]
        self.default_friction = struct.unpack("<f", self.cpu.mem_read(self.symbols["sim_firmware_friction"], 4))[0]

    def call(self, function):
        stop = self.symbols["sim_return"] & ~1                 # the return address of every call
        self.cpu.reg_write(UC_ARM_REG_SP, self.symbols["_estack"])
        self.cpu.reg_write(UC_ARM_REG_LR, stop | 1)
        self.cpu.emu_start(self.symbols[function] | 1, stop)  # no timeout: Unicorn starts a thread per call (40x slower)

    def step(self, inputs, flags):
        """inputs: rear left / right wheel rad/s, steering wheel rad, IMU ax, ay, gyro, throttle %, brake bar,
        the VCU's friction, dt -> OUTPUTS"""
        self.cpu.mem_write(self.symbols["sim_input"], struct.pack("<10fI", *inputs, flags))
        self.call("sim_step")
        raw = self.cpu.mem_read(self.symbols["sim_output"], 4 * len(OUTPUTS))
        return dict(zip(OUTPUTS, struct.unpack(f"<{len(OUTPUTS)}f", raw)))


# ================================================== car model ==================================================

def peak_slip(stiffness_factor, curvature, shape_factor):
    """Slip of the Magic Formula peak (= vehicle_dynamics.c magic_formula_peak_slip)"""
    phi = math.tan(math.pi / (2 * shape_factor))
    x = phi
    for _ in range(6):
        x -= (x * (1 - curvature) + curvature * math.atan(x) - phi) / ((1 - curvature) + curvature / (1 + x * x))
    return x / stiffness_factor


def magic_formula(slip, stiffness_factor, shape_factor, peak, curvature):
    x = stiffness_factor * slip
    return peak * math.sin(shape_factor * math.atan(x - curvature * (x - math.atan(x))))


class Tyre:
    """Magic Formula 5.2 at one load (the layer's .tir coefficients, camber 0): pure slip curves, combined by the slips
    as fractions of their peaks - a spinning wheel loses its lateral grip"""

    def __init__(self, c, load_n, friction):
        load = max(load_n, 20.0)
        dfz = (load - c["tyre_FNOMIN"]) / c["tyre_FNOMIN"]
        self.cx = c["tyre_PCX1"]
        self.dx = (c["tyre_PDX1"] + c["tyre_PDX2"] * dfz) * c["tyre_LMUX"] * friction * load
        slip_stiffness = load * (c["tyre_PKX1"] + c["tyre_PKX2"] * dfz) * math.exp(c["tyre_PKX3"] * dfz)
        self.bx = slip_stiffness / (self.cx * self.dx)
        ex = c["tyre_PEX1"] + c["tyre_PEX2"] * dfz + c["tyre_PEX3"] * dfz * dfz
        self.ex = (min(ex * (1 - c["tyre_PEX4"]), 0.98), min(ex * (1 + c["tyre_PEX4"]), 0.98))     # driving, braking
        self.cy = c["tyre_PCY1"]
        self.dy = (c["tyre_PDY1"] + c["tyre_PDY2"] * dfz) * c["tyre_LMUY"] * friction * load
        cornering_stiffness = abs(c["tyre_PKY1"]) * c["tyre_FNOMIN"] * math.sin(2 * math.atan(load / (c["tyre_PKY2"] * c["tyre_FNOMIN"])))
        self.by = cornering_stiffness / (self.cy * self.dy)
        ey = c["tyre_PEY1"] + c["tyre_PEY2"] * dfz
        self.ey = (min(ey * (1 - c["tyre_PEY3"]), 0.98), min(ey * (1 + c["tyre_PEY3"]), 0.98))     # slip angle > 0, < 0
        self.slip_peak = tuple(peak_slip(self.bx, e, self.cx) for e in self.ex)
        self.angle_peak = tuple(peak_slip(self.by, e, self.cy) for e in self.ey)

    def forces(self, slip, angle):
        """slip ratio, slip angle [rad] -> (force along the wheel, force to its left) [N]"""
        driving, left = slip >= 0, angle >= 0
        slip_peak, angle_peak = self.slip_peak[0 if driving else 1], self.angle_peak[0 if left else 1]
        slip_share, angle_share = slip / slip_peak, angle / angle_peak
        combined = math.hypot(slip_share, angle_share)
        if combined < 1e-9:
            return 0.0, 0.0
        fx = magic_formula(math.copysign(combined * slip_peak, slip), self.bx, self.cx, self.dx, self.ex[0 if driving else 1])
        fy = magic_formula(math.copysign(combined * angle_peak, angle), self.by, self.cy, self.dy, self.ey[0 if left else 1])
        return fx * abs(slip_share) / combined, fy * abs(angle_share) / combined


def road_wheel_angles(c, steering_wheel_rad):
    """steering wheel -> (FL, FR, RL, RR) road wheel angles [rad], as vehicle_dynamics.c compute_road_wheel_angles"""
    x = steering_wheel_rad
    mean = c["steering_linear_gain"] * x + c["steering_cubic_gain"] * x ** 3
    size = abs(mean)
    inner = outer = size
    if size > 1e-4:
        radius = c["wheelbase_m"] / math.tan(size)
        half_track = c["track_front_m"] / 2
        inner += c["ackermann_fraction"] * (math.atan(c["wheelbase_m"] / max(radius - half_track, 0.1)) - size)
        outer += c["ackermann_fraction"] * (math.atan(c["wheelbase_m"] / (radius + half_track)) - size)
    if mean >= 0:
        return inner, outer, 0.0, 0.0
    return -outer, -inner, 0.0, 0.0


class Car:
    """Two-track model: body (vx, vy, yaw rate) + 4 wheel spins, MF tyres, load transfer, aero, brakes, rear motors.
    Axes as the layer: x forward, y left; the world: x east, y north, heading from x."""

    def __init__(self, config):
        c = self.c = config
        front = c["center_of_gravity_to_front_axle_m"]
        rear = c["wheelbase_m"] - front
        self.wheel_x = (front, front, -rear, -rear)
        self.wheel_y = (c["track_front_m"] / 2, -c["track_front_m"] / 2, c["track_rear_m"] / 2, -c["track_rear_m"] / 2)
        self.static_load = [c["mass_kg"] * GRAVITY * (rear if i < 2 else front) / (2 * c["wheelbase_m"]) for i in range(4)]
        self.torque_constant = 1.5 * c["motor_pole_pairs"] * c["motor_flux_linkage_v_s"]     # DTI: torque = this x Iq
        rear_inertia = WHEEL_INERTIA_KG_M2 + MOTOR_INERTIA_KG_M2 * c["gear_ratio"] ** 2
        self.inertia = (WHEEL_INERTIA_KG_M2, WHEEL_INERTIA_KG_M2, rear_inertia, rear_inertia)
        self.reset(0.0, 0.0, 0.0)

    def reset(self, x, y, heading):
        self.x, self.y, self.heading = x, y, heading
        self.vx = self.vy = self.yaw_rate = self.yaw_acceleration = 0.0
        self.ax = self.ay = self.ax_filtered = self.ay_filtered = 0.0  # at the centre of gravity, gravity not included
        self.omega = [0.0] * 4
        self.steer = (0.0,) * 4
        self.load = list(self.static_load)
        self.fx, self.fy, self.slip, self.angle = [0.0] * 4, [0.0] * 4, [0.0] * 4, [0.0] * 4
        self.motor_torque = [0.0] * 4       # at the wheel, > 0 = driving
        self.brake_torque = [0.0] * 4       # hydraulic, at the wheel
        self.tyres = [Tyre(self.c, load, 1.0) for load in self.static_load]

    def imu(self):
        """(ax, ay) at the IMU and the gyro: what the layer reads (it moves them back to the centre of gravity)"""
        imu_x = self.c["imu_ahead_of_center_of_gravity_m"]
        return self.ax - self.yaw_rate ** 2 * imu_x, self.ay + self.yaw_acceleration * imu_x, self.yaw_rate

    def yaw_moment_of_longitudinal_forces(self):
        """what the tyres' Fx make about the centre of gravity: the real counterpart of the TV yaw moment"""
        return sum(self.wheel_x[i] * self.fx[i] * math.sin(self.steer[i]) - self.wheel_y[i] * self.fx[i] * math.cos(self.steer[i])
                   for i in range(4))

    def step(self, steering_wheel_rad, brake_bar, motor_current_a, friction, dt):
        """One control period with the rear motor currents (left, right) and the brake pressure held"""
        c = self.c
        self.steer = road_wheel_angles(c, steering_wheel_rad)
        # quasi-static loads from the filtered accelerations and the tyres at them, once per period
        downforce = 0.5 * AIR_DENSITY * c["aero_lift_coefficient_area_m2"] * self.vx ** 2
        height, wheelbase = c["center_of_gravity_height_m"], c["wheelbase_m"]
        longitudinal = c["mass_kg"] * self.ax_filtered * height / (2 * wheelbase)            # per wheel, front -> rear
        lateral_front = c["mass_kg"] * self.ay_filtered * height * c["roll_stiffness_front_share"] / c["track_front_m"]
        lateral_rear = c["mass_kg"] * self.ay_filtered * height * (1 - c["roll_stiffness_front_share"]) / c["track_rear_m"]
        front_aero, rear_aero = downforce * c["aero_front_share"] / 2, downforce * (1 - c["aero_front_share"]) / 2
        self.load = [max(load, 20.0) for load in (
            self.static_load[0] + front_aero - longitudinal - lateral_front, self.static_load[1] + front_aero - longitudinal + lateral_front,
            self.static_load[2] + rear_aero + longitudinal - lateral_rear, self.static_load[3] + rear_aero + longitudinal + lateral_rear)]
        tyres = self.tyres = [Tyre(c, self.load[i], friction) for i in range(4)]
        # torques held over the period. DTI: motor torque = 1.5 p psi Iq; SetBrakeCurrent brakes towards 0 rpm
        self.brake_torque = [BRAKE_NM_PER_BAR[i] * brake_bar for i in range(4)]
        self.motor_torque = [0.0, 0.0] + [self.torque_constant * current * c["gear_ratio"]
                                          * (c["gear_efficiency"] if current >= 0 else 1 / c["gear_efficiency"]) for current in motor_current_a]
        drive = [max(torque, 0.0) for torque in self.motor_torque]
        stop = [self.brake_torque[i] + max(-self.motor_torque[i], 0.0) for i in range(4)]
        h = dt / PLANT_SUBSTEPS
        for _ in range(PLANT_SUBSTEPS):
            self._substep(tyres, drive, stop, h)
        if math.hypot(self.vx, self.vy) < 0.2 and max(drive) == 0.0:      # standing still
            self.vx = self.vy = self.yaw_rate = 0.0
            self.omega = [0.0] * 4

    def _substep(self, tyres, drive, stop, h):
        c = self.c
        radius, mass = c["wheel_radius_m"], c["mass_kg"]
        force_x = force_y = moment_z = 0.0
        for i in range(4):
            x, y = self.wheel_x[i], self.wheel_y[i]
            cos_steer, sin_steer = math.cos(self.steer[i]), math.sin(self.steer[i])
            u, v = self.vx - self.yaw_rate * y, self.vy + self.yaw_rate * x        # wheel centre velocity, car axes
            along, across = u * cos_steer + v * sin_steer, -u * sin_steer + v * cos_steer
            reference = max(abs(along), SLIP_MINIMUM_SPEED)
            self.slip[i] = (self.omega[i] * radius - along) / reference
            self.angle[i] = -math.atan2(across, reference)                    # = steer - direction of travel
            fx, fy = tyres[i].forces(self.slip[i], self.angle[i])
            self.fx[i], self.fy[i] = fx, fy
            # wheel spin: drive torque and tyre, then brake + regen as friction (stops the wheel, never turns it back)
            spin = self.omega[i] + h * (drive[i] - fx * radius) / self.inertia[i]
            braking = h * stop[i] / self.inertia[i]
            self.omega[i] = 0.0 if abs(spin) <= braking else spin - math.copysign(braking, spin)
            car_x, car_y = fx * cos_steer - fy * sin_steer, fx * sin_steer + fy * cos_steer
            force_x += car_x
            force_y += car_y
            moment_z += x * car_y - y * car_x
        drag = 0.5 * AIR_DENSITY * DRAG_AREA_M2 * self.vx * abs(self.vx) + ROLLING_RESISTANCE * mass * GRAVITY * math.tanh(self.vx / 0.5)
        self.ax, self.ay = (force_x - drag) / mass, force_y / mass
        self.yaw_acceleration = moment_z / c["yaw_inertia_kg_m2"]
        self.vx += h * (self.ax + self.yaw_rate * self.vy)
        self.vy += h * (self.ay - self.yaw_rate * self.vx)
        self.yaw_rate += h * self.yaw_acceleration
        self.heading += h * self.yaw_rate
        self.x += h * (self.vx * math.cos(self.heading) - self.vy * math.sin(self.heading))
        self.y += h * (self.vx * math.sin(self.heading) + self.vy * math.cos(self.heading))
        k = h / (LOAD_TIME_CONSTANT_S + h)
        self.ax_filtered += (self.ax - self.ax_filtered) * k
        self.ay_filtered += (self.ay - self.ay_filtered) * k


# ================================================== track and driver ==================================================

# Autocross-like closed loop [m], counter-clockwise: straight, left hairpin, slalom, left hairpin back to the start
AUTOCROSS_POINTS = [(0, 0), (25, 0), (50, 0), (63, 5), (68, 17), (62, 29), (50, 33), (40, 31), (28, 35), (16, 31),
                    (4, 35), (-8, 33), (-18, 26), (-21, 14), (-15, 4)]
TRACK_STEP_M = 0.5


def make_track(kind):
    """-> centre line every TRACK_STEP_M: x, y, heading, curvature (numpy arrays), closed; starts at index 0"""
    if kind == "Skidpad":
        radius = 9.125                                         # FS skidpad centre line (inner circle 15.25 m), left turns
        angle = np.arange(0, 2 * math.pi * radius, TRACK_STEP_M) / radius
        x, y = radius * np.sin(angle), radius * (1 - np.cos(angle))
    else:                                                      # closed Catmull-Rom spline through the points
        points = np.array(AUTOCROSS_POINTS, float)
        t = np.linspace(0, 1, 40, endpoint=False)[:, None]
        segments = []
        for i in range(len(points)):
            p0, p1, p2, p3 = points[i - 1], points[i], points[(i + 1) % len(points)], points[(i + 2) % len(points)]
            segments.append(0.5 * (2 * p1 + (p2 - p0) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2 + (3 * p1 - p0 - 3 * p2 + p3) * t ** 3))
        dense = np.vstack(segments + [points[:1]])
        distance = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(dense, axis=0).T))])
        s = np.arange(0, distance[-1] - TRACK_STEP_M / 2, TRACK_STEP_M)
        x, y = np.interp(s, distance, dense[:, 0]), np.interp(s, distance, dense[:, 1])
    heading = np.arctan2(np.roll(y, -1) - np.roll(y, 1), np.roll(x, -1) - np.roll(x, 1))
    turn = np.angle(np.exp(1j * (np.roll(heading, -1) - np.roll(heading, 1))))
    curvature = turn / (2 * TRACK_STEP_M)
    curvature = np.convolve(np.r_[curvature[-4:], curvature, curvature[:4]], np.ones(9) / 9, "valid")   # closed: wraps
    return {"kind": kind, "x": x, "y": y, "heading": heading, "curvature": curvature, "length": len(x) * TRACK_STEP_M}


def speed_profile(track, lateral_g, braking_g=1.2, accelerating_g=0.8):
    """the automatic driver's target speed per point: lateral_g in the corners, braking / accelerating between them"""
    speed = np.minimum(np.sqrt(lateral_g * GRAVITY / np.maximum(np.abs(track["curvature"]), 1e-3)), 28.0)
    n = len(speed)
    for _ in range(2):                                         # twice round: the lap is closed
        for i in range(n):
            speed[i] = min(speed[i], math.sqrt(speed[i - 1] ** 2 + 2 * accelerating_g * GRAVITY * TRACK_STEP_M))
        for i in reversed(range(n)):
            speed[i] = min(speed[i], math.sqrt(speed[(i + 1) % n] ** 2 + 2 * braking_g * GRAVITY * TRACK_STEP_M))
    return speed


class Driver:
    def __init__(self, config):
        self.c = config
        self.steering = self.throttle = self.brake = 0.0

    def steer(self, car, track, nearest):
        """pure pursuit from the rear axle -> steering wheel angle [rad], at most STEERING_RATE_RAD_S"""
        c = self.c
        rear = c["wheelbase_m"] - c["center_of_gravity_to_front_axle_m"]
        axle_x, axle_y = car.x - rear * math.cos(car.heading), car.y - rear * math.sin(car.heading)
        lookahead = min(max(2.0 + 0.3 * car.vx, 3.5), 12.0)
        target = (nearest + int(lookahead / TRACK_STEP_M)) % len(track["x"])
        dx, dy = track["x"][target] - axle_x, track["y"][target] - axle_y
        sideways = -dx * math.sin(car.heading) + dy * math.cos(car.heading)
        road = math.atan(2 * c["wheelbase_m"] * sideways / max(dx * dx + dy * dy, 1.0))
        wheel = road / c["steering_linear_gain"]                # invert road = linear x + cubic x^3
        for _ in range(3):
            wheel -= (c["steering_linear_gain"] * wheel + c["steering_cubic_gain"] * wheel ** 3 - road) / (
                c["steering_linear_gain"] + 3 * c["steering_cubic_gain"] * wheel ** 2)
        wheel = min(max(wheel, -2.6), 2.6)
        step = STEERING_RATE_RAD_S * CONTROL_PERIOD_S
        self.steering += min(max(wheel - self.steering, -step), step)
        return self.steering

    def pedals(self, speed, target):
        """automatic driver -> (throttle %, brake bar): proportional to the speed error [m/s], a foot of 0.1 s"""
        error = target - speed
        k = CONTROL_PERIOD_S / (0.1 + CONTROL_PERIOD_S)
        self.throttle += (min(max(25 + 40 * error, 0.0), 100.0) - self.throttle) * k
        self.brake += (min(max(-8 * (error + 1.0), 0.0), 40.0) - self.brake) * k
        return self.throttle, self.brake


class Simulation:
    """car + driver + the firmware, one control period per step(); no GUI (the self-test runs it alone)"""

    def __init__(self, firmware, config):
        self.firmware, self.config = firmware, config
        self.car, self.driver = Car(config), Driver(config)
        self.flags = firmware.default_flags | FLAG_TRACTION_CONTROL | FLAG_TORQUE_VECTORING
        self.friction = 1.0                                    # the real track x the tyre model
        self.vcu_friction = firmware.default_friction          # what the VCU assumes (THROTTLE_FRICTION_*)
        self.automatic = True
        self.lateral_g = 1.1
        self.set_track("Autocross")

    def set_track(self, kind):
        self.track = make_track(kind)
        self.profile = speed_profile(self.track, self.lateral_g)
        self.restart()

    def set_lateral_g(self, lateral_g):
        self.lateral_g = lateral_g
        self.profile = speed_profile(self.track, lateral_g)

    def restart(self):
        t = self.track
        self.car.reset(t["x"][0], t["y"][0], t["heading"][0])
        self.driver.steering = self.driver.throttle = self.driver.brake = 0.0
        self.time = 0.0
        self.reset_layer = True
        self.erpm_age, self.erpm_speed = ERPM_PERIOD_S, (0.0, 0.0)
        self.nearest, self.offset = 0, 0.0
        self.lap_start, self.laps = 0.0, []
        self.throttle = self.brake = self.steering = 0.0
        self.out = dict.fromkeys(OUTPUTS, 0.0)

    def step(self, throttle_percent=0.0, brake_bar=0.0):
        car, track = self.car, self.track
        distance2 = (track["x"] - car.x) ** 2 + (track["y"] - car.y) ** 2
        previous, self.nearest = self.nearest, int(np.argmin(distance2))
        self.offset = math.sqrt(distance2[self.nearest])
        n = len(track["x"])
        if previous > 0.8 * n and self.nearest < 0.2 * n:      # crossed the start line
            self.laps.append(self.time - self.lap_start)
            self.lap_start = self.time
        if self.automatic:
            ahead = (self.nearest + int(max(car.vx, 3.0) * 0.6 / TRACK_STEP_M)) % n
            throttle_percent, brake_bar = self.driver.pedals(car.vx, min(self.profile[self.nearest], self.profile[ahead]))
        steering = self.driver.steer(car, track, self.nearest)
        # the inverters send ERPM every 33 ms: the VCU sees the rear wheel speeds in steps
        self.erpm_age += CONTROL_PERIOD_S
        if self.erpm_age >= ERPM_PERIOD_S:
            self.erpm_age, self.erpm_speed = 0.0, (abs(car.omega[2]), abs(car.omega[3]))
        ax, ay, yaw_rate = car.imu()
        flags = self.flags | (FLAG_RESET if self.reset_layer else 0)
        self.reset_layer = False
        self.out = self.firmware.step((*self.erpm_speed, steering, ax, ay, yaw_rate, throttle_percent, brake_bar,
                                       self.vcu_friction, CONTROL_PERIOD_S), flags)
        self.throttle, self.brake, self.steering = throttle_percent, brake_bar, steering
        car.step(steering, brake_bar, (self.out["current_left_a"], self.out["current_right_a"]), self.friction, CONTROL_PERIOD_S)
        self.time += CONTROL_PERIOD_S


# ================================================== GUI ==================================================

CAR_OUTLINE = [(1.45, 0.22), (1.15, 0.45), (-0.9, 0.55), (-1.2, 0.45), (-1.2, -0.45), (-0.9, -0.55), (1.15, -0.45), (1.45, -0.22)]
WHEEL_SIZE_M = (0.46, 0.2)
FORCE_SCALE_N_PER_M = 600.0     # force arrows: this many N per metre of arrow
CHART_SECONDS = 10.0
GREEN, RED, BLUE, ORANGE, PURPLE, GREY = "#2a9d2a", "#d62828", "#1f77b4", "#e08000", "#8e44ad", "#888888"


class Chart:
    """The last CHART_SECONDS of some values, on part of a canvas, fixed y range"""

    def __init__(self, canvas, box, title, y_range, series):
        self.canvas, self.box, self.y_range = canvas, box, y_range
        x0, y0, x1, y1 = box
        canvas.create_rectangle(x0, y0, x1, y1, outline="#bbbbbb")
        zero = self._y(0.0)
        canvas.create_line(x0, zero, x1, zero, fill="#dddddd")
        canvas.create_text(x0 + 4, y0 + 2, text=title, anchor="nw", font=("Segoe UI", 9, "bold"))
        canvas.create_text(x1 - 3, y0 + 2, text=f"{y_range[1]:g}", anchor="ne", fill=GREY, font=("Segoe UI", 8))
        canvas.create_text(x1 - 3, y1 - 2, text=f"{y_range[0]:g}", anchor="se", fill=GREY, font=("Segoe UI", 8))
        label_x = x0 + 4
        self.lines = []
        for label, colour, dash in series:
            text = canvas.create_text(label_x, y1 - 3, text=label, anchor="sw", fill=colour, font=("Segoe UI", 8))
            label_x = canvas.bbox(text)[2] + 8
            self.lines.append(canvas.create_line(0, 0, 0, 0, fill=colour, width=1.5, dash=dash))
        self.data = deque()

    def _y(self, value):
        low, high = self.y_range
        value = min(max(value, low), high)
        return self.box[3] - (value - low) / (high - low) * (self.box[3] - self.box[1])

    def add(self, time_s, values):
        self.data.append((time_s, values))
        while self.data and self.data[0][0] < time_s - CHART_SECONDS:
            self.data.popleft()

    def clear(self):
        self.data.clear()

    def draw(self):
        if len(self.data) < 2:
            for line in self.lines:
                self.canvas.coords(line, 0, 0, 0, 0)
            return
        x0, _, x1, _ = self.box
        end = self.data[-1][0]
        xs = [x1 - (end - t) / CHART_SECONDS * (x1 - x0) for t, _ in self.data]
        for k, line in enumerate(self.lines):
            coords = []
            for x, (_, values) in zip(xs, self.data):
                coords += (x, self._y(values[k]))
            self.canvas.coords(line, *coords)


class App:
    VIEW_SIZE = (640, 430)      # the window fits 1536 x 864 (1080p at 125 %)
    WORLD_MARGIN_M = 40.0

    def __init__(self, root, simulation):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk, self.root, self.sim = tk, ttk, root, simulation
        root.title("Simulador do carro - TV / TC do VCU (firmware emulado)")
        self.paused = False
        self.keys = set()
        self.time_debt = 0.0
        self.last_tick = time.perf_counter()
        self.pixels_per_metre = 45.0
        self.trail = deque(maxlen=900)

        left, middle, right = ttk.Frame(root), ttk.Frame(root, padding=(8, 0)), ttk.Frame(root)
        left.grid(row=0, column=0, sticky="n")
        middle.grid(row=0, column=1, sticky="n")
        right.grid(row=0, column=2, sticky="n", padx=(0, 6))
        self.view = tk.Canvas(left, width=self.VIEW_SIZE[0], height=self.VIEW_SIZE[1], bg="#f4f1ea", highlightthickness=0)
        self.view.grid(row=0, column=0)
        self.charts_canvas = tk.Canvas(left, width=self.VIEW_SIZE[0], height=290, bg="white", highlightthickness=0)
        self.charts_canvas.grid(row=1, column=0, pady=(4, 0))
        w, h = self.VIEW_SIZE[0] // 2 - 4, 141
        boxes = [(2, 2, w, h), (w + 6, 2, 2 * w + 4, h), (2, h + 6, w, 2 * h + 4), (w + 6, h + 6, 2 * w + 4, 2 * h + 4)]
        self.charts = [
            Chart(self.charts_canvas, boxes[0], "Yaw rate [deg/s]", (-150, 150),
                  [("real", "black", None), ("referencia VCU", ORANGE, (4, 2)), ("estimado VCU", BLUE, None)]),
            Chart(self.charts_canvas, boxes[1], "Momento de guinada Mz [Nm]", (-600, 600),
                  [("pedido TV", ORANGE, (4, 2)), ("entregue (alocacao)", PURPLE, None), ("real (Fx pneus)", "black", None)]),
            Chart(self.charts_canvas, boxes[2], "Binario motor na roda [Nm]", (-500, 500),
                  [("RL", BLUE, None), ("RR", RED, None), ("travao hidr. RL", GREY, (2, 2))]),
            Chart(self.charts_canvas, boxes[3], "Slip traseiro [%] / fator TC [x100]", (-20, 115),
                  [("slip RL", BLUE, None), ("slip RR", RED, None), ("TC RL", BLUE, (3, 2)), ("TC RR", RED, (3, 2))]),
        ]

        # ---------- middle: controls ----------
        self.minimap = tk.Canvas(middle, width=300, height=150, bg="white", highlightthickness=1, highlightbackground="#cccccc")
        self.minimap.grid(row=0, column=0, sticky="w", pady=(0, 4))

        controls = ttk.LabelFrame(middle, text="Simulacao", padding=4)
        controls.grid(row=1, column=0, sticky="we")
        self.pause_button = ttk.Button(controls, text="Pausa (espaco)", command=self.toggle_pause, takefocus=False)
        self.pause_button.grid(row=0, column=0)
        ttk.Button(controls, text="Recomecar (R)", command=self.restart, takefocus=False).grid(row=0, column=1)
        self.track_kind = tk.StringVar(value=self.sim.track["kind"])
        box = ttk.Combobox(controls, textvariable=self.track_kind, values=["Autocross", "Skidpad"], width=10, state="readonly")
        box.grid(row=0, column=2, padx=4)
        box.bind("<<ComboboxSelected>>", lambda _: self.change_track())
        self.sim_speed = self._scale(controls, 1, "Velocidade sim [x]", 0.1, 2.0, 1.0, 0.1)

        vcu = ttk.LabelFrame(middle, text="VCU (firmware) - #define entre ( )", padding=4)
        vcu.grid(row=2, column=0, sticky="we", pady=4)
        default = self.sim.firmware.default_flags
        self.switches = {}
        for i, (flag, text) in enumerate([(FLAG_TORQUE_VECTORING, "Vetorizacao de binario (TV)"),
                                          (FLAG_TRACTION_CONTROL, "Controlo de tracao (TC)"),
                                          (FLAG_IMU_GYRO, "Giroscopio (yaw rate)"),
                                          (FLAG_IMU_ACCELERATIONS, "IMU ax / ay")]):
            variable = tk.BooleanVar(value=bool(self.sim.flags & flag))
            ttk.Checkbutton(vcu, text=f"{text}  ({1 if default & flag else 0})", variable=variable, takefocus=False,
                            command=self.update_flags).grid(row=i, column=0, sticky="w")
            self.switches[flag] = variable

        road = ttk.LabelFrame(middle, text="Pista e pedais", padding=4)
        road.grid(row=3, column=0, sticky="we")
        self.friction = self._scale(road, 0, "Aderencia real [x pneu]", 0.4, 1.3, 1.0, 0.05,
                                    lambda value: setattr(self.sim, "friction", value))
        self._scale(road, 1, f"Atrito no VCU ({self.sim.vcu_friction:.2f})", 0.4, 1.3, self.sim.vcu_friction, 0.05,
                    lambda value: setattr(self.sim, "vcu_friction", value))
        self.automatic = tk.BooleanVar(value=self.sim.automatic)
        ttk.Checkbutton(road, text="Pedais automaticos (velocidade da curva)", variable=self.automatic, takefocus=False,
                        command=self.toggle_automatic).grid(row=2, column=0, columnspan=3, sticky="w")
        self.lateral_g = self._scale(road, 3, "g lateral (auto)", 0.6, 1.8, self.sim.lateral_g, 0.05, self.sim.set_lateral_g)
        self.throttle_scale = self._scale(road, 4, "Acelerador [%]", 0, 100, 0, 1)
        self.brake_scale = self._scale(road, 5, "Travao [bar]", 0, 50, 0, 0.5)
        ttk.Label(road, text="Teclas: cima/W acelerar, baixo/S travar (regen de 2 bar), espaco pausa, R recomecar, roda zoom",
                  foreground=GREY, wraplength=300).grid(row=6, column=0, columnspan=3, sticky="w")

        # ---------- right: numbers ----------
        now = ttk.LabelFrame(right, text="Agora", padding=4)
        now.grid(row=0, column=0, sticky="we")
        self.readout = ttk.Label(now, font=("Consolas", 9), justify="left")
        self.readout.grid(row=0, column=0, sticky="w")

        table = ttk.LabelFrame(right, text="Por roda", padding=4)
        table.grid(row=1, column=0, sticky="we", pady=4)
        self.rows = ["Carga Fz [N]", "Fx pneu [N]", "Fy pneu [N]", "Slip [%]", "Angulo slip [deg]", "Motor na roda [Nm]",
                     "Travao hidr. [Nm]", "Fator TC"]
        for col, name in enumerate(WHEELS):
            ttk.Label(table, text=name, font=("Segoe UI", 9, "bold")).grid(row=0, column=col + 1, padx=6)
        self.cells = {}
        for row, name in enumerate(self.rows):
            ttk.Label(table, text=name).grid(row=row + 1, column=0, sticky="w")
            for col in range(4):
                cell = ttk.Label(table, font=("Consolas", 9), width=7, anchor="e")
                cell.grid(row=row + 1, column=col + 1)
                self.cells[row, col] = cell

        for key in ("Up", "w", "W"):
            root.bind(f"<KeyPress-{key}>", lambda _: self.keys.add("throttle"))
            root.bind(f"<KeyRelease-{key}>", lambda _: self.keys.discard("throttle"))
        for key in ("Down", "s", "S"):
            root.bind(f"<KeyPress-{key}>", lambda _: self.keys.add("brake"))
            root.bind(f"<KeyRelease-{key}>", lambda _: self.keys.discard("brake"))
        root.bind("<space>", lambda _: self.toggle_pause())
        root.bind("<KeyPress-r>", lambda _: self.restart())
        root.bind("<KeyPress-R>", lambda _: self.restart())
        self.view.bind("<MouseWheel>", self.zoom)

        self.draw_world()
        root.after(30, self.tick)

    def _scale(self, parent, row, text, low, high, value, resolution, command=None):
        self.ttk.Label(parent, text=text).grid(row=row, column=0, sticky="w")
        variable = self.tk.DoubleVar(value=value)
        scale = self.tk.Scale(parent, variable=variable, from_=low, to=high, resolution=resolution, orient="horizontal",
                              length=190, showvalue=True, highlightthickness=0, takefocus=0)   # keys drive the car
        scale.grid(row=row, column=1, columnspan=2, sticky="w")
        if command:
            scale.configure(command=lambda _: command(variable.get()))
        return variable

    # ---------- actions ----------
    def update_flags(self):
        flags = self.sim.flags & ~sum(self.switches)
        self.sim.flags = flags | sum(flag for flag, variable in self.switches.items() if variable.get())

    def toggle_automatic(self):
        self.sim.automatic = self.automatic.get()
        if not self.sim.automatic:                             # your pedals start released
            self.throttle_scale.set(0)
            self.brake_scale.set(0)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.configure(text="Continuar (espaco)" if self.paused else "Pausa (espaco)")

    def restart(self):
        self.sim.restart()
        self.trail.clear()
        for chart in self.charts:
            chart.clear()

    def change_track(self):
        self.sim.set_track(self.track_kind.get())
        self.restart()
        self.draw_world()
        self.view.focus_set()                                  # keys back to the car, not the combobox

    def zoom(self, event):
        self.pixels_per_metre = min(max(self.pixels_per_metre * (1.25 if event.delta > 0 else 0.8), 8.0), 120.0)
        self.trail.clear()                                     # canvas coordinates at the old zoom
        self.draw_world()

    # ---------- drawing ----------
    def to_view(self, x, y):
        return (x - self.world_x0) * self.pixels_per_metre, (self.world_y1 - y) * self.pixels_per_metre

    def polygon(self, points, x, y, heading):
        """body-frame points [m] -> flat canvas coordinates"""
        c, s = math.cos(heading), math.sin(heading)
        coords = []
        for px, py in points:
            coords += self.to_view(x + px * c - py * s, y + px * s + py * c)
        return coords

    def draw_world(self):
        view, track = self.view, self.sim.track
        view.delete("all")
        self.world_x0 = track["x"].min() - self.WORLD_MARGIN_M
        self.world_y1 = track["y"].max() + self.WORLD_MARGIN_M
        width = (track["x"].max() - track["x"].min() + 2 * self.WORLD_MARGIN_M) * self.pixels_per_metre
        height = (track["y"].max() - track["y"].min() + 2 * self.WORLD_MARGIN_M) * self.pixels_per_metre
        self.world_size = (width, height)
        view.configure(scrollregion=(0, 0, width, height))
        normal_x, normal_y = -np.sin(track["heading"]), np.cos(track["heading"])
        centre = [coordinate for i in range(len(track["x"])) for coordinate in self.to_view(track["x"][i], track["y"][i])]
        view.create_line(*centre, *centre[:2], fill="#d8d2c4", width=TRACK_WIDTH_M * self.pixels_per_metre)
        view.create_line(*centre, *centre[:2], fill="#bdb6a6", dash=(6, 6))
        cone = max(2.0, 0.15 * self.pixels_per_metre)
        for i in range(0, len(track["x"]), 5):
            for side, colour in ((1, "#1f4fd1"), (-1, "#f2c400")):          # FS: blue left, yellow right
                cx, cy = self.to_view(track["x"][i] + side * normal_x[i] * TRACK_WIDTH_M / 2,
                                      track["y"][i] + side * normal_y[i] * TRACK_WIDTH_M / 2)
                view.create_oval(cx - cone, cy - cone, cx + cone, cy + cone, fill=colour, outline="")
        view.create_line(*self.to_view(track["x"][0] + normal_x[0] * 2, track["y"][0] + normal_y[0] * 2),
                         *self.to_view(track["x"][0] - normal_x[0] * 2, track["y"][0] - normal_y[0] * 2), width=3)
        self.trail_line = view.create_line(0, 0, 0, 0, fill="#a0a0a0", width=2)
        self.body = view.create_polygon(0, 0, 0, 0, fill="#3a3a3a", outline="black")
        self.wheels = [view.create_polygon(0, 0, 0, 0, fill="#111111", outline="#111111", width=3) for _ in range(4)]
        self.lateral_arrows = [view.create_line(0, 0, 0, 0, fill=BLUE, width=2, arrow="last") for _ in range(4)]
        self.force_arrows = [view.create_line(0, 0, 0, 0, width=5, arrow="last", arrowshape=(10, 12, 4)) for _ in range(4)]
        self.velocity_arrow = view.create_line(0, 0, 0, 0, fill="#00a0a0", width=2, arrow="last", dash=(4, 2))
        self.wheel_labels = [view.create_text(0, 0, font=("Consolas", 9, "bold"), justify="center") for _ in range(4)]
        self.hud = view.create_text(0, 0, anchor="nw", font=("Segoe UI", 10, "bold"))           # fixed on the view
        self.hud_warning = view.create_text(0, 0, anchor="nw", fill=RED, font=("Segoe UI", 12, "bold"))
        self.legend = view.create_text(0, 0, anchor="sw", fill="#555555", font=("Segoe UI", 8), text=(
            f"setas: verde tracao / vermelha travagem (Fx), azul lateral (Fy), {FORCE_SCALE_N_PER_M:.0f} N por metro; "
            "tracejado = velocidade\nroda laranja: slip acima do pico do pneu, vermelha: a patinar / bloquear; "
            "contorno magenta: TC a cortar"))
        # minimap
        self.minimap.delete("all")
        span = max(np.ptp(track["x"]), np.ptp(track["y"])) + 6
        scale = min(250 / span, 160 / span)
        self.minimap_transform = (track["x"].min() - 3, track["y"].max() + 3, scale)
        points = [value for i in range(len(track["x"])) for value in self.to_minimap(track["x"][i], track["y"][i])]
        self.minimap.create_line(*points, *points[:2], fill="#cfc8b8", width=5)
        self.minimap_car = self.minimap.create_oval(0, 0, 0, 0, fill=RED, outline="")

    def to_minimap(self, x, y):
        x0, y1, scale = self.minimap_transform
        return 5 + (x - x0) * scale, 5 + (y1 - y) * scale

    def draw(self):
        sim, car, out, view = self.sim, self.sim.car, self.sim.out, self.view
        self.trail.append(self.to_view(car.x, car.y))
        if len(self.trail) > 1:
            view.coords(self.trail_line, *[value for point in self.trail for value in point])
        view.coords(self.body, *self.polygon(CAR_OUTLINE, car.x, car.y, car.heading))
        c, s = math.cos(car.heading), math.sin(car.heading)
        length, width = WHEEL_SIZE_M
        tc_factor = (1.0, 1.0, out["tc_factor_left"], out["tc_factor_right"])
        for i in range(4):
            wx = car.x + car.wheel_x[i] * c - car.wheel_y[i] * s
            wy = car.y + car.wheel_x[i] * s + car.wheel_y[i] * c
            angle = car.heading + car.steer[i]
            box = [(length / 2, width / 2), (-length / 2, width / 2), (-length / 2, -width / 2), (length / 2, -width / 2)]
            peak = car.tyres[i].slip_peak[0 if car.slip[i] >= 0 else 1]      # orange past the MF peak, red past 2.5x
            slip = abs(car.slip[i])
            fill = RED if slip > 2.5 * peak else ORANGE if slip > peak else "#111111"
            view.coords(self.wheels[i], *self.polygon(box, wx, wy, angle))
            view.itemconfigure(self.wheels[i], fill=fill, outline="#ff00ff" if tc_factor[i] < 0.99 else fill)
            # force arrows: along the wheel Fx (green drive / red braking), across it Fy
            fx_m, fy_m = car.fx[i] / FORCE_SCALE_N_PER_M, car.fy[i] / FORCE_SCALE_N_PER_M
            view.coords(self.force_arrows[i], *self.to_view(wx, wy),
                        *self.to_view(wx + fx_m * math.cos(angle), wy + fx_m * math.sin(angle)))
            view.itemconfigure(self.force_arrows[i], fill=GREEN if car.fx[i] >= 0 else RED)
            view.coords(self.lateral_arrows[i], *self.to_view(wx, wy),
                        *self.to_view(wx - fy_m * math.sin(angle), wy + fy_m * math.cos(angle)))
            # label outside the wheel, front ones ahead, rear ones behind
            side, along = (1.3 if car.wheel_y[i] > 0 else -1.3), (0.7 if i < 2 else -0.7)
            lx, ly = self.to_view(wx + along * c - side * s, wy + along * s + side * c)
            if i >= 2:
                text = f"{car.motor_torque[i]:+.0f} Nm\nslip {100 * car.slip[i]:.0f}%"
                if tc_factor[i] < 0.99:
                    text += f"\nTC x{tc_factor[i]:.2f}"
            else:
                text = f"trav {car.brake_torque[i]:.0f} Nm" if sim.brake >= 1.0 else f"Fy {car.fy[i]:.0f} N"
            view.coords(self.wheel_labels[i], lx, ly)
            view.itemconfigure(self.wheel_labels[i], text=text)
        speed_m = 0.12
        view.coords(self.velocity_arrow, *self.to_view(car.x, car.y),
                    *self.to_view(car.x + speed_m * (car.vx * c - car.vy * s), car.y + speed_m * (car.vx * s + car.vy * c)))
        cx, cy = self.to_view(car.x, car.y)
        view.xview_moveto((cx - self.VIEW_SIZE[0] / 2) / self.world_size[0])
        view.yview_moveto((cy - self.VIEW_SIZE[1] / 2) / self.world_size[1])
        mx, my = self.to_minimap(car.x, car.y)
        self.minimap.coords(self.minimap_car, mx - 4, my - 4, mx + 4, my + 4)

        # charts
        real_moment = car.yaw_moment_of_longitudinal_forces()
        degrees = 180 / math.pi
        values = [
            (car.yaw_rate * degrees, out["yaw_rate_reference_rad_s"] * degrees, out["yaw_rate_estimate_rad_s"] * degrees),
            (out["yaw_moment_request_nm"], out["yaw_moment_delivered_nm"], real_moment),
            (car.motor_torque[2], car.motor_torque[3], car.brake_torque[2]),
            (100 * car.slip[2], 100 * car.slip[3], 100 * out["tc_factor_left"], 100 * out["tc_factor_right"]),
        ]
        for chart, chart_values in zip(self.charts, values):
            chart.add(sim.time, chart_values)
            chart.draw()

        # text
        laps = sim.laps
        lap_text = f"  |  volta {len(laps) + 1}: {sim.time - sim.lap_start:.1f} s"
        if laps:
            lap_text += f" (ultima {laps[-1]:.2f} s, melhor {min(laps):.2f} s)"
        sideslip = math.degrees(math.atan2(car.vy, max(abs(car.vx), 0.5)))
        warnings = []
        if sim.offset > TRACK_WIDTH_M / 2:
            warnings.append("FORA DE PISTA")
        if abs(sideslip) > 25:
            warnings.append("PIAO - R para recomecar")
        if self.paused:
            warnings.append("PAUSA")
        view.itemconfigure(self.hud, text=f"t {sim.time:.1f} s{lap_text}   |   {3.6 * math.hypot(car.vx, car.vy):.0f} km/h")
        view.itemconfigure(self.hud_warning, text="   ".join(warnings))
        view.coords(self.hud, view.canvasx(8), view.canvasy(6))
        view.coords(self.hud_warning, view.canvasx(8), view.canvasy(26))
        view.coords(self.legend, view.canvasx(8), view.canvasy(self.VIEW_SIZE[1] - 4))
        tv = "ativo" if out["tv_active"] else ("desligado" if not sim.flags & FLAG_TORQUE_VECTORING else "inativo (sem giro / lento)")
        self.readout.configure(text=(
            f"velocidade  {3.6 * math.hypot(car.vx, car.vy):6.1f} km/h   VCU {3.6 * out['vehicle_speed_mps']:6.1f} km/h\n"
            f"ax / ay     {car.ax / GRAVITY:+6.2f} / {car.ay / GRAVITY:+5.2f} g\n"
            f"sideslip    {sideslip:+6.1f} deg      VCU {math.degrees(out['sideslip_rad']):+6.1f} deg\n"
            f"pedais      acel {sim.throttle:5.1f} %   travao {sim.brake:5.1f} bar\n"
            f"forca pedida {out['driver_force_request_n']:7.0f} N  (soma traseiras {out['force_left_n'] + out['force_right_n']:7.0f} N)\n"
            f"TV {tv}\n"
            f"Mz pedido {out['yaw_moment_request_nm']:+6.0f}  entregue {out['yaw_moment_delivered_nm']:+6.0f}  real {real_moment:+6.0f} Nm"))
        rows = [car.load, car.fx, car.fy, [100 * slip for slip in car.slip], [math.degrees(a) for a in car.angle],
                car.motor_torque, car.brake_torque, tc_factor]
        formats = ["{:.0f}", "{:+.0f}", "{:+.0f}", "{:+.1f}", "{:+.1f}", "{:+.0f}", "{:.0f}", "{:.2f}"]
        for row, (values_row, form) in enumerate(zip(rows, formats)):
            for col in range(4):
                text = form.format(values_row[col]) if not (row == 7 and col < 2) else "-"
                self.cells[row, col].configure(text=text)

    # ---------- loop ----------
    def tick(self):
        now = time.perf_counter()
        elapsed, self.last_tick = min(now - self.last_tick, 0.1), now
        if not self.paused:
            if not self.sim.automatic:
                throttle = 100.0 if "throttle" in self.keys else self.throttle_scale.get()
                brake = KEY_BRAKE_BAR if "brake" in self.keys else self.brake_scale.get()
            else:
                throttle = brake = 0.0
            self.time_debt += elapsed * self.sim_speed.get()
            steps = 0
            while self.time_debt >= CONTROL_PERIOD_S and steps < 80:
                self.sim.step(throttle, brake)
                self.time_debt -= CONTROL_PERIOD_S
                steps += 1
            if steps == 80:
                self.time_debt = 0.0                           # this PC is slower than real time: drop the backlog
            if self.sim.automatic:
                self.throttle_scale.set(round(self.sim.throttle))
                self.brake_scale.set(round(self.sim.brake * 2) / 2)
        self.draw()
        self.root.after(30, self.tick)


# ================================================== self-test and main ==================================================

def selftest():
    firmware, config = Firmware(), read_layer_config()
    sim = Simulation(firmware, config)
    radius = 1 / np.abs(sim.track["curvature"]).max()
    print(f"track: {sim.track['length']:.0f} m, tightest radius {radius:.1f} m; firmware #define flags {firmware.default_flags:#x}")
    failures = []
    for tv in (True, False):
        sim.flags = firmware.default_flags | FLAG_TRACTION_CONTROL | (FLAG_TORQUE_VECTORING if tv else 0)
        sim.set_track("Autocross")
        worst_offset = excess = 0.0
        tv_steps = 0
        started = time.perf_counter()
        for _ in range(int(40 / CONTROL_PERIOD_S)):
            sim.step()
            out, car = sim.out, sim.car
            if not all(math.isfinite(v) for v in (*out.values(), car.x, car.y, car.vx, car.vy, car.yaw_rate, *car.omega)):
                failures.append(f"TV {tv}: NaN at {sim.time:.2f} s")
                break
            worst_offset = max(worst_offset, sim.offset)
            if out["driver_force_request_n"] > 0:
                excess = max(excess, out["force_left_n"] + out["force_right_n"] - out["driver_force_request_n"])
            tv_steps += out["tv_active"] > 0
        speed = 40 / (time.perf_counter() - started)
        print(f"TV {'on ' if tv else 'off'}: laps {[round(lap, 2) for lap in sim.laps]}, worst offset {worst_offset:.2f} m, "
              f"rear forces above the pedal {excess:.1f} N, TV active {100 * tv_steps * CONTROL_PERIOD_S / 40:.0f} % "
              f"({speed:.1f}x real time)")
        if worst_offset > TRACK_WIDTH_M / 2:
            failures.append(f"TV {tv}: off the track by {worst_offset - TRACK_WIDTH_M / 2:.2f} m")
        if not sim.laps:
            failures.append(f"TV {tv}: not one lap in 40 s")
        if tv and (tv_steps == 0 or excess > 1.0):
            failures.append(f"TV on: active {tv_steps} steps, rear forces above the pedal by {excess:.1f} N")
    print("\n".join(failures) if failures else "selftest OK")
    return 1 if failures else 0


def main():
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    import tkinter as tk
    root = tk.Tk()
    root.title("Simulador do carro - a compilar o firmware...")
    root.update()
    App(root, Simulation(Firmware(), read_layer_config()))
    root.mainloop()


if __name__ == "__main__":
    main()
