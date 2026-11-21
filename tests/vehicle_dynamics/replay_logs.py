"""Test day logs (ROS 2 bags) through the VCU vehicle dynamics layer on the emulated Cortex-M7 (run_tests.py), the way
the VCU runs it today: rear motor ERPM only (no front wheels, no IMU), pedal and front brake pressure (AQT1, the VCU's
0x710), steering (AQT4), traction control on, dry track, without and with the IMU (0x530 / 0x531 = the ROS /imu
topics). Open loop: the logged wheels do not answer the layer's torque.

Truth (as relatorio_milagres_2026-10-13.py): car speed = the rear wheels where both grip, the IMU acceleration
integrated across spins / locks (no IMU: a straight line between). Spin: a rear wheel over the car by max(3 km/h, 15 %)
for >= 0.1 s with the pedal pressed.

Run: python tests/vehicle_dynamics/replay_logs.py FOLDER_WITH_BAGS [...]     (pip install mcap mcap-ros2-support unicorn)
"""
import collections
import glob
import math
import os
import sys
import tempfile

import numpy as np
import pandas as pd

import run_tests

DT = 0.005                                       # VCU control period
GEAR, POLE_PAIRS, RADIUS = 14.73, 4, 0.200       # vehicle_dynamics.c
MAX_CURRENT_A = 120.0                            # DTI MaxAcCurrent: SetRelCurrent 100 % (the old map)
KT, GEAR_EFFICIENCY = 1.5 * 4 * 0.0504, 0.90
STEERING_OFFSET_DEG, STEERING_SIGN = 5.0, -1.0   # traction_control_torque_vectoring.h
STATE_READY_MANUAL = 6
FIELDS = {"/pwt/vcu_states": ["vcu_state"], "/pwt/inv1_erpm_duty_voltage": ["inv1_actual_erpm"],
          "/pwt/inv2_erpm_duty_voltage": ["inv2_actual_erpm"], "/data/aqt1": ["throtle_percentage", "frt_brk_press"],
          "/pwt/aqt4": ["st_angle"], "/imu/acceleration": ["vector.x", "vector.y"], "/imu/angular_velocity": ["vector.z"],
          "/pwt/inv1_setrelcurrent": ["inv1_cmd_targetrelativecurrent"], "/pwt/inv2_setrelcurrent": ["inv2_cmd_targetrelativecurrent"]}
CACHE = os.path.join(tempfile.gettempdir(), "t26_vcu_replay_v2")


def extract(bag):
    """bag -> {topic|field: values, topic|t: times}, cached"""
    os.makedirs(CACHE, exist_ok=True)
    cache = os.path.join(CACHE, os.path.basename(bag)[:-5] + ".npz")
    if not (os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(bag)):
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
        data = collections.defaultdict(list)
        with open(bag, "rb") as handle:
            reader = make_reader(handle, decoder_factories=[DecoderFactory()])
            topics = [c.topic for c in reader.get_summary().channels.values() if c.topic in FIELDS]
            for _, channel, message, ros in reader.iter_decoded_messages(topics=topics, log_time_order=True):
                data[channel.topic + "|t"].append(message.log_time / 1e9)
                for field in FIELDS[channel.topic]:
                    value = ros
                    for part in field.split("."):
                        value = getattr(value, part)
                    data[channel.topic + "|" + field].append(float(value))
        np.savez_compressed(cache, **{k: np.asarray(v, float) for k, v in data.items()})
    return dict(np.load(cache))


def smooth(x, seconds):
    return pd.Series(x).rolling(max(1, round(seconds / DT)), center=True, min_periods=1).mean().to_numpy()


def grips(v):
    """True where the wheel (m/s) rolls with the car: rises <= 8, falls <= 15 m/s2, no lock dip (report 13-10)"""
    acceleration = np.gradient(smooth(v, 0.05), DT)
    dip = pd.Series((acceleration < -15).astype(float)).rolling(round(0.6 / DT), center=True, min_periods=1).max().to_numpy() > 0
    n = np.arange(len(v))
    if dip.all():
        return ~dip
    vv = np.interp(n, n[~dip], v[~dip])
    forward, backward = vv.copy(), vv.copy()
    for k in range(1, len(v)):
        forward[k] = min(vv[k], forward[k - 1] + 8 * DT)
    for k in range(len(v) - 2, -1, -1):
        backward[k] = min(vv[k], backward[k + 1] + 15 * DT)
    return ~dip & (v - np.minimum(forward, backward) < 0.5 / 3.6)


def bridge(ok, reference, acceleration):
    """reference where ok; across the gaps the acceleration integrated, drift spread to close on the next anchor"""
    v = np.where(ok, reference, np.nan)
    index = np.where(ok)[0]
    if len(index) < 2:
        return reference.copy()
    integral = np.cumsum(acceleration) * DT
    for g in np.where(np.diff(index) > 1)[0]:
        a, b = index[g], index[g + 1]
        segment = reference[a] + integral[a:b + 1] - integral[a]
        v[a:b + 1] = segment + (reference[b] - segment[-1]) * np.arange(b - a + 1) / (b - a)
    v[:index[0]] = reference[index[0]]
    v[index[-1]:] = reference[index[-1]]
    return np.maximum(v, 0)


class Run:
    def __init__(self, bag):
        z = extract(bag)
        self.name = os.path.basename(bag)
        self.ok = "/pwt/vcu_states|t" in z and "/pwt/inv1_erpm_duty_voltage|t" in z and "/data/aqt1|t" in z
        if not self.ok:
            return
        t_state = z["/pwt/vcu_states|t"]
        self.T = np.arange(t_state[0], t_state[-1], DT)
        hold = lambda key, default=0.0: (z[key][np.clip(np.searchsorted(z[key.split("|")[0] + "|t"], self.T, "right") - 1, 0, None)]
                                         if key in z and len(z[key]) else np.full(len(self.T), default))
        self.ready = hold("/pwt/vcu_states|vcu_state") == STATE_READY_MANUAL
        to_mps = 2 * math.pi / 60 / POLE_PAIRS / GEAR * RADIUS
        self.left = np.abs(hold("/pwt/inv1_erpm_duty_voltage|inv1_actual_erpm")) * to_mps    # INV1 = rear left
        self.right = np.abs(hold("/pwt/inv2_erpm_duty_voltage|inv2_actual_erpm")) * to_mps
        self.throttle = np.clip(hold("/data/aqt1|throtle_percentage"), 0, 100)
        self.brake = hold("/data/aqt1|frt_brk_press")
        self.steering = (hold("/pwt/aqt4|st_angle", STEERING_OFFSET_DEG) - STEERING_OFFSET_DEG) * STEERING_SIGN * math.pi / 180
        self.steering_valid = "/pwt/aqt4|t" in z
        self.command_a = np.maximum(hold("/pwt/inv1_setrelcurrent|inv1_cmd_targetrelativecurrent"),
                                    hold("/pwt/inv2_setrelcurrent|inv2_cmd_targetrelativecurrent")) * MAX_CURRENT_A / 100
        # IMU (13-10): x backwards -> car ax = -x, bias from standing still
        self.imu = np.zeros(len(self.T), bool)
        ax = np.zeros(len(self.T))
        if "/imu/acceleration|t" in z:
            t_imu = z["/imu/acceleration|t"]
            inside = (self.T >= t_imu[0]) & (self.T <= t_imu[-1])
            ax = np.where(inside, -smooth(np.interp(self.T, t_imu, z["/imu/acceleration|vector.x"]), 0.05), 0.0)
            still = self.ready & inside & (self.left < 0.05) & (self.right < 0.05)
            ax -= np.median(ax[still]) if still.sum() > 100 else 0.0
            self.imu = inside
        self.ax = ax
        # what the VCU gets from the IMU frames: last value, car axes (traction_control_torque_vectoring.h IMU_*_SIGN),
        # valid while newer than IMU_TIMEOUT_MS
        self.imu_input = np.zeros((len(self.T), 3))
        self.imu_input_valid = np.zeros((len(self.T), 2), bool)
        for column, (topic, field, sign) in enumerate((("/imu/acceleration", "vector.x", -1), ("/imu/acceleration", "vector.y", -1),
                                                       ("/imu/angular_velocity", "vector.z", 1))):
            if topic + "|" + field in z:
                t = z[topic + "|t"]
                last = np.searchsorted(t, self.T, "right") - 1
                self.imu_input[:, column] = sign * z[topic + "|" + field][np.clip(last, 0, None)]
                self.imu_input_valid[:, min(column, 1) if column < 2 else 1] = (last >= 0) & (self.T - t[np.clip(last, 0, None)] <= 0.2)
        both_grip = grips(self.left) & grips(self.right)
        self.car = bridge(both_grip, (self.left + self.right) / 2, ax)
        self.grip = both_grip
        over = lambda wheel: wheel - self.car > np.maximum(3 / 3.6, 0.15 * self.car)
        self.spinning = (over(self.left) | over(self.right)) & (self.throttle > 5)

    def replay(self, image, symbols, margin, use_imu=False):
        rows, where = [], np.where(self.ready)[0]
        previous = -2
        for k in where:
            flags = 1 | 2 | (4 if self.steering_valid else 0) | (8 if k != previous + 1 else 0)
            if use_imu:                                                  # True: accelerations and gyro, "gyro": gyro only
                flags |= (16 if self.imu_input_valid[k, 0] and use_imu is True else 0) | (32 if self.imu_input_valid[k, 1] else 0)
            rows.append((self.left[k] / RADIUS, self.right[k] / RADIUS, self.throttle[k], self.brake[k], self.steering[k],
                         *self.imu_input[k], flags))
            previous = k
        if not rows:
            return None
        out = run_tests.replay(image, symbols, rows, margin=margin)
        full = {name: np.full(len(self.T), np.nan) for name in out}
        for name, values in out.items():
            full[name][where] = values
        return full


def events(mask, minimum_s):
    """[(start, end)] of runs of mask lasting >= minimum_s"""
    edges = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    return [(a, b) for a, b in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]) if (b - a) * DT >= minimum_s]


def report(folder, image, symbols, margins):
    runs = [r for r in (Run(b) for b in sorted(glob.glob(os.path.join(folder, "**", "*.mcap"), recursive=True))) if r.ok and r.ready.sum() > 400]
    print(f"\n=== {os.path.basename(os.path.normpath(folder))}: {len(runs)} runs, {sum(r.ready.sum() for r in runs) * DT:.0f} s in READY,"
          f" IMU in {sum(1 for r in runs if np.any(r.imu))}")
    cat = lambda f: np.concatenate([f(r) for r in runs])
    ready, throttle, brake = cat(lambda r: r.ready), cat(lambda r: r.throttle), cat(lambda r: r.brake)
    on_throttle = ready & (throttle > 20)
    braking = ready & (brake > 3)
    print(f"brake, driver on the throttle (> 20 %): p50 {np.percentile(brake[on_throttle], 50):.1f}  p99 {np.percentile(brake[on_throttle], 99):.1f}"
          f"  max {brake[on_throttle].max():.1f} bar  -> {100 * np.mean(brake[on_throttle] > 2):.1f} % of the time over the 2 bar regen start")
    if braking.any():
        print(f"brake, braking (> 3 bar): p50 {np.percentile(brake[braking], 50):.1f}  p90 {np.percentile(brake[braking], 90):.1f}"
              f"  p99 {np.percentile(brake[braking], 99):.1f} bar, {100 * np.mean(brake[braking] >= 10):.0f} % of it at >= 10 bar (full regen)")
    for margin, use_imu in margins:
        if use_imu and not any(r.imu_input_valid[:, 1].any() for r in runs):
            continue
        outs = [r.replay(image, symbols, margin, use_imu) for r in runs]
        got = lambda name: np.concatenate([o[name] for o in outs])
        vx, tc = got("vehicle_speed_mps"), np.fmin(got("torque_factor_left"), got("torque_factor_right"))
        car, grip, spinning = cat(lambda r: r.car), cat(lambda r: r.grip), cat(lambda r: r.spinning)
        moving = ready & (car > 3)
        driving_grip = moving & grip & (throttle > 5) & ~spinning
        error = np.abs(vx - car)[moving]
        caught, total = 0, 0
        for r, o in zip(runs, outs):
            for a, b in events(r.spinning & r.ready, 0.1):
                total += 1
                factor = np.fmin(o["torque_factor_left"], o["torque_factor_right"])[a:min(b, a + round(0.2 / DT))]
                caught += bool(np.nanmin(factor) < 0.95)
        mode = {False: "rear wheels only", "gyro": "gyro only", True: "IMU (accelerations + gyro)"}[use_imu]
        print(f"{mode}, margin {margin or 1.2:.1f}: |vx - car| moving p50 {np.percentile(error, 50):.2f} p95 {np.percentile(error, 95):.2f} m/s;"
              f" TC cuts a gripping wheel {100 * np.mean(tc[driving_grip] < 0.98):.1f} % of {driving_grip.sum() * DT:.0f} s;"
              f" spins caught in 0.2 s {caught}/{total}")
        # car acceleration (IMU) against the layer's no-IMU limit: over it, vx lags the car and TC sees slip
        imu = cat(lambda r: r.imu)
        ax, amax = cat(lambda r: smooth(r.ax, 0.3)), got("maximum_acceleration_ms2")
        for lo, hi in ((3, 10), (10, 20), (20, 40)):
            sample = driving_grip & imu & (throttle > 80) & (car >= lo) & (car < hi)
            if sample.sum() > 100:
                print(f"    {lo}-{hi} m/s, > 80 % throttle, grip: IMU ax p99 {np.percentile(ax[sample], 99):.1f} m/s2,"
                      f" layer limit p1 {np.percentile(amax[sample], 1):.1f} p50 {np.percentile(amax[sample], 50):.1f},"
                      f" ax over the limit {100 * np.mean(ax[sample] > amax[sample]):.2f} %")
    ratios = []
    for r, o in zip(runs, outs):
        for a, b in events(r.spinning & r.ready, 0.1):
            k0, k1 = max(a - round(0.1 / DT), 0), a
            if r.car[a] < 2 or abs(np.mean(r.steering[k0:k1])) > 0.17:      # straight line, rolling
                continue
            force = np.mean(r.command_a[k0:k1]) * KT * GEAR * GEAR_EFFICIENCY / RADIUS
            available = np.nanmean(o["available_drive_force_n"][k0:k1])
            if force > 50 and available > 0:
                ratios.append(force / available)
    if ratios:
        print(f"spin onsets, straight: logged drive force / layer available (dry): median {np.median(ratios):.2f}"
              f" (p25 {np.percentile(ratios, 25):.2f}, p75 {np.percentile(ratios, 75):.2f}, n {len(ratios)})"
              f" -> track grip ~{np.median(ratios):.2f} x the model")
    full = ready & (throttle > 90) & moving
    if full.any():
        layer_a = np.fmax(got("current_left_a"), got("current_right_a"))
        command = cat(lambda r: r.command_a)
        for lo, hi in ((0, 10), (10, 20), (20, 40)):
            band = full & (car >= lo) & (car < hi)
            if band.sum() > 50:
                print(f"> 90 % throttle, {lo}-{hi} m/s: old map {np.mean(command[band]):.0f} A, layer {np.nanmean(layer_a[band]):.0f} A")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    image, symbols = run_tests.build("-O2")
    for folder in sys.argv[1:]:
        report(folder, image, symbols, margins=((0.0, False), (0.0, "gyro"), (0.0, True)))


if __name__ == "__main__":
    main()
