"""Unit tests of the manual driving pipeline (vehicle_dynamics.c + throttle_control.c) on an emulated Cortex-M7.

Builds test_main.c with the firmware sources and the firmware toolchain (arm-none-eabi-gcc, as CubeIDE), then RUNS the
ARM machine code on the Unicorn CPU emulator: no host C compiler, no board. Exit code 1 if a check fails.

Needs: pip install unicorn        Run: python tests/vehicle_dynamics/run_tests.py
-mfpu=fpv5-sp-d16, not the firmware's fpv5-d16: Unicorn's Cortex-M7 has no double-precision FPU. The tested files are
float only (-Wdouble-promotion), so their machine code is the same; only newlib's internal doubles go to software.
"""
import glob
import os
import shutil
import struct
import subprocess
import sys
import tempfile

from unicorn import UC_ARCH_ARM, UC_HOOK_INTR, UC_MODE_MCLASS, UC_MODE_THUMB, Uc
from unicorn.arm_const import UC_ARM_REG_PC, UC_ARM_REG_SP, UC_CPU_ARM_CORTEX_M7

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
SOURCES = [os.path.join(HERE, "test_main.c")] + [os.path.join(ROOT, "Core", "Src", f + ".c") for f in
                                                  ("vehicle_dynamics", "throttle_control", "traction_control_torque_vectoring", "APPS")]
INCLUDES = ["Core/Inc", "Core/DBC", "Drivers/STM32F7xx_HAL_Driver/Inc", "Drivers/STM32F7xx_HAL_Driver/Inc/Legacy",
            "Drivers/CMSIS/Device/ST/STM32F7xx/Include", "Drivers/CMSIS/Include"]
FLAGS = ["-std=gnu11", "-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-sp-d16", "-mfloat-abi=hard", "-g", "-Wall",
         "-Wdouble-promotion", "-Werror", "-Wno-error=cpp", "-DSTM32F767xx", "-DUSE_HAL_DRIVER", "-DDEBUG", "-ffunction-sections",
         "-fdata-sections", "-nostartfiles", "-T", os.path.join(HERE, "test.ld"), "--specs=nano.specs",
         "--specs=nosys.specs", "-Wl,--gc-sections"]
FLASH, RAM, PPB, REPLAY_IN, REPLAY_OUT = 0x08000000, 0x20000000, 0xE0000000, 0x60000000, 0x64000000
REPLAY_INPUT_SIZE, REPLAY_OUTPUT_SIZE = 36, 60          # sizeof(replay_input_t), sizeof(replay_output_t)
REPLAY_OUTPUTS = ["vehicle_speed_mps", "speed_derivative_ms2", "maximum_acceleration_ms2", "slip_left", "slip_right",
                  "torque_factor_left", "torque_factor_right", "current_left_a", "current_right_a",
                  "available_drive_force_n", "maximum_deceleration_ms2", "driver_force_request_n",
                  "lateral_velocity_mps", "yaw_rate_rad_s", "peak_slip_drive"]


def replay(image, symbols, inputs, friction=1.0, traction_control=True, margin=0.0):
    """inputs: rows at 5 ms, (rear_left_rad_s, rear_right_rad_s, throttle_percent, brake_bar, steering_rad, flags) or
    with the IMU in car axes (..., steering_rad, ax_ms2, ay_ms2, yaw_rate_rad_s, flags); flags: 1 throttle, 2 brake,
    4 steering, 16 IMU accelerations, 32 gyro valid, 8 reset first -> {output name: list}"""
    data = b"".join(struct.pack("<8fI", *(row if len(row) == 9 else (*row[:5], 0.0, 0.0, 0.0, row[5]))) for row in inputs)
    header = struct.pack("<IfIf", len(inputs), friction, int(traction_control), margin)
    cpu = emulate(image, symbols, header, data, REPLAY_OUTPUT_SIZE * len(inputs))
    raw = bytes(cpu.mem_read(REPLAY_OUT, REPLAY_OUTPUT_SIZE * len(inputs)))
    values = struct.unpack(f"<{len(REPLAY_OUTPUTS) * len(inputs)}f", raw)
    return {name: values[k::len(REPLAY_OUTPUTS)] for k, name in enumerate(REPLAY_OUTPUTS)}


def toolchain(tool):
    found = shutil.which(tool)
    if found:
        return found
    cube = sorted(glob.glob("C:/ST/STM32CubeIDE*/STM32CubeIDE/plugins/*gnu-tools-for-stm32*/tools/bin"))
    if not cube:
        sys.exit(f"{tool} not found (PATH or STM32CubeIDE)")
    return os.path.join(cube[-1], tool + ".exe")


def build(optimisation="-O0"):
    """-> (flash image, symbol addresses). -O0 = the firmware Debug build."""
    with tempfile.TemporaryDirectory() as build_dir:
        elf, image = os.path.join(build_dir, "test.elf"), os.path.join(build_dir, "test.bin")
        subprocess.run([toolchain("arm-none-eabi-gcc"), optimisation, *FLAGS, *(f"-I{os.path.join(ROOT, i)}" for i in INCLUDES),
                        *SOURCES, "-lm", "-o", elf], check=True)
        subprocess.run([toolchain("arm-none-eabi-objcopy"), "-O", "binary", elf, image], check=True)
        nm = subprocess.run([toolchain("arm-none-eabi-nm"), elf], capture_output=True, text=True, check=True).stdout
        symbols = {p[2]: int(p[0], 16) for p in (line.split() for line in nm.splitlines()) if len(p) == 3}
        return open(image, "rb").read(), symbols


def emulate(image, symbols, header=b"", inputs=b"", output_size=0):
    """Runs the image until its BKPT. -> the CPU (to read RAM). header + inputs at REPLAY_IN (zeros = unit tests)."""
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
    page = lambda size: max(0x1000, (size + 0xFFF) & ~0xFFF)
    cpu.mem_map(FLASH, 1024 * 1024)
    cpu.mem_map(RAM, 512 * 1024)
    cpu.mem_map(PPB, 1024 * 1024)                # system registers as plain memory (CPACR write)
    cpu.mem_map(REPLAY_IN, page(16 + len(inputs)))
    cpu.mem_map(REPLAY_OUT, page(output_size))
    cpu.mem_write(FLASH, image)
    cpu.mem_write(REPLAY_IN, header.ljust(16, b"\0") + inputs)
    cpu.reg_write(UC_ARM_REG_SP, symbols["_estack"])
    stop = {}
    cpu.hook_add(UC_HOOK_INTR, lambda uc, number, user: (stop.setdefault("pc", uc.reg_read(UC_ARM_REG_PC)), uc.emu_stop()))
    cpu.emu_start(symbols["Reset_Handler"] | 1, 0xFFFFFFFF, timeout=3600 * 1000 * 1000)
    if struct.unpack("<I", cpu.mem_read(symbols["done"], 4))[0] != 1:
        sys.exit(f"did not finish: stopped at pc {stop.get('pc', 0):#x} (BKPT #1 = fault)")
    return cpu


def read_string(cpu, address):
    data = bytes(cpu.mem_read(address, 256))
    return data[:data.index(0)].decode()


def main():
    image, symbols = build()
    cpu = emulate(image, symbols)
    word = lambda name: struct.unpack("<I", cpu.mem_read(symbols[name], 4))[0]
    tests, checks, failure_count = word("test_count"), word("check_count"), word("failure_count")
    failed = {}
    for i in range(min(failure_count, 64)):
        test, expression, line, value = struct.unpack("<IIIf", cpu.mem_read(symbols["failures"] + 16 * i, 16))
        failed.setdefault(read_string(cpu, test), []).append(f"line {line}: {read_string(cpu, expression)}  (value {value:g})")
    for test, lines in failed.items():
        print(f"FAIL {test}")
        for line in lines:
            print(f"     {line}")
    print(f"{tests - len(failed)}/{tests} tests passed, {checks} checks, {failure_count} failed")
    return 1 if failure_count else 0


if __name__ == "__main__":
    sys.exit(main())
