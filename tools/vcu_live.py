"""
Live read of VCU variables over the ST-LINK (SWD) while the firmware runs - like the
STM32CubeIDE Live Expressions, without stopping or resetting the CPU.

- Addresses, sizes and types come from the ELF (Debug/VCU_STM32_Firmware_V1.elf) through
  the arm-none-eabi-gdb that ships with STM32CubeIDE. This needs no board.
- Values are read with pyocd (pip install pyocd) in "attach" mode: no reset, no halt.
- Before any value is trusted, flash on the board is compared with the ELF (vector table
  and the throttle map). If the flashed firmware is not this build, the connection is
  refused - otherwise the addresses would point at the wrong data.
- The ST-LINK can only be used by one program: do not debug in CubeIDE while connected.
- The only write: write_apps_cal() saves the APPS calibration record (APPS_FlashCal_t in
  APPS.h) to the reserved flash block with the openocd that ships with STM32CubeIDE, which
  resets the VCU. The firmware loads it at boot (APPS_Init).

    python tools/vcu_live.py             # print the live values twice per second (no GUI)
    python tools/vcu_live.py --symbols   # only resolve the variables from the ELF (no board needed)
"""
import contextlib
import glob
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ELF_FILE = ROOT / "Debug" / "VCU_STM32_Firmware_V1.elf"

# (C expression, label, scale). Add more lines here; type and size come from the ELF.
LIVE_VARS = [
    ("current_state", "Estado da VCU", 1),
    ("throttle.status", "Estado do acelerador", 1),
    ("throttle.pedal_1000", "Pedal (%)", 0.1),
    ("throttle.speed_kmh", "Velocidade (km/h)", 1),
    ("throttle.map_1000", "Mapa (%)", 0.1),
    ("throttle.drive_cmd_1000", "Tracao enviada (%)", 0.1),
    ("throttle.regen_cmd_1000", "Regen enviada (%)", 0.1),
    ("throttle.bspd_active", "BSPD ativo", 1),
    ("throttle.apps_error", "Erro APPS", 1),
    ("throttle.inverter_fault", "Falha inversor", 1),
    ("throttle.dc_voltage_v", "Tensao DC (V)", 1),
    ("apps_data.state.apps1_raw", "APPS1 (bits)", 1),
    ("apps_data.state.apps2_raw", "APPS2 (bits)", 1),
    ("apps_data.state.apps2_adjusted", "APPS2 na escala do APPS1 (bits)", 1),
    ("apps_data.state.disagreement", "Discordancia APPS (bits)", 1),
    ("apps_data.config.from_flash", "Calibracao APPS da flash", 1),
    ("apps_data.config.min_value", "APPS1 a 0 % (bits)", 1),
    ("apps_data.config.max_value", "APPS1 a 100 % (bits)", 1),
    ("vcu.brake_pressure", "Travao (bar)", 1),
]
# Flash regions that must match the ELF before any address is trusted
CHECK_SYMBOLS = ["g_pfnVectors", "throttle_map"]
CHECK_BYTES = 256
# SWD clock. Lower = fewer read errors from inverter noise on SWCLK/SWDIO with the motors
# running (was 4 MHz); 1 MHz still reads all LIVE_VARS well within the 200 ms poll.
SWD_FREQUENCY_HZ = 1_000_000
# APPS calibration record in flash - same layout as APPS_FlashCal_t in APPS.h
CAL_SYMBOL = "apps_cal_flash"  # linker symbol, start of the reserved block
CAL_MAGIC = 0x43505041  # APPS_CAL_MAGIC, "APPC"
CAL_FORMAT = "<IHHHH"  # magic, apps1_rest, apps1_full, apps2_rest, apps2_full; then CRC32
CAL_SIZE = struct.calcsize(CAL_FORMAT) + 4


def pack_apps_cal(cal):
    """{"rest": (apps1, apps2), "full": (apps1, apps2)} -> the 16 bytes written to flash."""
    body = struct.pack(CAL_FORMAT, CAL_MAGIC, cal["rest"][0], cal["full"][0], cal["rest"][1], cal["full"][1])
    return body + struct.pack("<I", zlib.crc32(body))


def parse_apps_cal(raw):
    """Inverse of pack_apps_cal. None if erased or corrupted (magic/CRC), like the firmware."""
    body, (crc,) = raw[:-4], struct.unpack("<I", raw[-4:])
    magic, a1r, a1f, a2r, a2f = struct.unpack(CAL_FORMAT, body)
    if magic != CAL_MAGIC or crc != zlib.crc32(body):
        return None
    return {"rest": (a1r, a2r), "full": (a1f, a2f)}


@dataclass
class Var:
    expr: str
    label: str
    scale: float
    address: int = 0
    size: int = 0
    kind: str = ""  # "u", "i", "f", "bool", "enum"
    enum_names: dict = field(default_factory=dict)

    def decode(self, raw: bytes):
        if self.kind == "f":
            return struct.unpack("<f" if self.size == 4 else "<d", raw)[0]
        value = int.from_bytes(raw, "little", signed=(self.kind == "i"))
        if self.kind == "bool":
            return bool(value)
        return value

    def text(self, value):
        if self.kind == "bool":
            return "SIM" if value else "nao"
        if self.kind == "enum":
            return self.enum_names.get(value, str(value))
        if self.kind == "f" or self.scale != 1:
            return f"{value * self.scale:.1f}"
        return str(value)


def find_gdb():
    found = shutil.which("arm-none-eabi-gdb")
    if found:
        return found
    pattern = "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/*gnu-tools-for-stm32*/tools/bin/arm-none-eabi-gdb.exe"
    matches = sorted(glob.glob(pattern))
    if not matches:
        raise RuntimeError("arm-none-eabi-gdb nao encontrado (vem com o STM32CubeIDE)")
    return matches[-1]


def find_openocd():
    """(openocd.exe, ST scripts folder) from the STM32CubeIDE plugins."""
    plugins = "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/"
    exe = sorted(glob.glob(plugins + "*externaltools.openocd.win32*/tools/bin/openocd.exe"))
    scripts = sorted(glob.glob(plugins + "*mcu.debug.openocd*/resources/openocd/st_scripts"))
    if not exe or not scripts:
        raise RuntimeError("openocd nao encontrado (vem com o STM32CubeIDE)")
    return exe[-1], scripts[-1]


def kind_from_ptype(ptype):
    if ptype.startswith("enum"):
        return "enum"
    if ptype in ("float", "double"):
        return "f"
    if ptype in ("_Bool", "bool"):
        return "bool"
    return "u" if "unsigned" in ptype else "i"


def enum_names(ptype):
    """'enum {A, B, C = 5, D}' -> {0: 'A', 1: 'B', 5: 'C', 6: 'D'}"""
    body = ptype[ptype.find("{") + 1:ptype.rfind("}")]
    names, value = {}, 0
    for item in (s.strip() for s in body.split(",") if s.strip()):
        if "=" in item:
            item, number = (s.strip() for s in item.split("=", 1))
            value = int(number, 0)
        names[value] = item
        value += 1
    return names


def resolve(elf=ELF_FILE):
    """Resolve LIVE_VARS and the check regions from the ELF with gdb (offline).
    Returns (vars, missing_exprs, checks, cal_address) - checks = [(symbol, address, expected_bytes)],
    cal_address = APPS calibration block (None if this firmware has none)."""
    if not elf.exists():
        raise RuntimeError(f"{elf.name} nao existe - compila o firmware no CubeIDE primeiro")
    lines = []
    for i, (expr, _, _) in enumerate(LIVE_VARS):
        lines += [f"echo @@v{i}\\n", f"print (unsigned long)&({expr})", f"print sizeof({expr})", f"ptype {expr}"]
    for j, sym in enumerate(CHECK_SYMBOLS):
        lines += [f"echo @@c{j}\\n", f"print (unsigned long)&{sym}", f"x/{CHECK_BYTES}xb &{sym}"]
    lines += ["echo @@cal\\n", f"print (unsigned long)&{CAL_SYMBOL}"]
    with tempfile.NamedTemporaryFile("w", suffix=".gdb", delete=False) as script:
        script.write("\n".join(lines) + "\n")
    try:
        out = subprocess.run([find_gdb(), "-batch", "-nx", "-q", str(elf), "-x", script.name],
                             capture_output=True, text=True, timeout=60).stdout
    finally:
        Path(script.name).unlink(missing_ok=True)

    sections = dict(re.findall(r"@@(\w+)\n(.*?)(?=@@|\Z)", out, flags=re.DOTALL))
    variables, missing = [], []
    for i, (expr, label, scale) in enumerate(LIVE_VARS):
        text = sections.get(f"v{i}", "")
        numbers = re.findall(r"^\$\d+ = (\d+)", text, flags=re.MULTILINE)
        ptype = re.search(r"^type = (.*?)$", text, flags=re.MULTILINE | re.DOTALL)
        if len(numbers) < 2 or not ptype:
            missing.append(expr)
            continue
        ptype_text = " ".join(ptype.group(1).split())
        var = Var(expr, label, scale, int(numbers[0]), int(numbers[1]), kind_from_ptype(ptype_text))
        if var.kind == "enum":
            var.enum_names = enum_names(ptype_text)
        variables.append(var)
    checks = []
    for j, sym in enumerate(CHECK_SYMBOLS):
        text = sections.get(f"c{j}", "")
        address = re.search(r"^\$\d+ = (\d+)", text, flags=re.MULTILINE)
        data = [int(b, 16) for line in text.splitlines() if ":" in line
                for b in re.findall(r"0x([0-9a-fA-F]{2})\b", line.split(":", 1)[1])]
        if not address or len(data) != CHECK_BYTES:
            raise RuntimeError(f"Simbolo '{sym}' nao encontrado no ELF - nao da para verificar o firmware")
        checks.append((sym, int(address.group(1)), bytes(data)))
    cal = re.search(r"^\$\d+ = (\d+)", sections.get("cal", ""), flags=re.MULTILINE)
    return variables, missing, checks, (int(cal.group(1)) if cal else None)


class VcuLive:
    """Connection to the running VCU through the ST-LINK. read() returns {expr: value}."""

    def __init__(self, elf=ELF_FILE):
        self.variables, self.missing, self.checks, self.cal_address = resolve(elf)
        self.session = None

    def connect(self):
        try:
            from pyocd.core.helpers import ConnectHelper
        except ImportError:
            raise RuntimeError("Falta o pyocd: pip install pyocd") from None
        session = ConnectHelper.session_with_chosen_probe(
            blocking=False, target_override="cortex_m", connect_mode="attach",
            options={"frequency": SWD_FREQUENCY_HZ})
        if session is None:
            raise RuntimeError("ST-LINK nao encontrado.\n\n"
                               "- Liga o ST-LINK ao PC por USB.\n"
                               "- Termina o debug no CubeIDE (so um programa pode usar o ST-LINK).")
        try:
            session.open()
            if session.board is None:
                raise RuntimeError("sessao sem placa")
            target = session.board.target
            flash = [bytes(target.read_memory_block8(address, CHECK_BYTES)) for _, address, _ in self.checks]
        except Exception as err:  # noqa: BLE001 - pyocd raises ProbeError/TransferError/... when the VCU does not answer
            with contextlib.suppress(Exception):
                session.close()
            raise RuntimeError("VCU nao encontrada: o ST-LINK esta ligado ao PC, mas a VCU nao responde.\n\n"
                               "- Confirma que a VCU esta ligada ao ST-LINK (cabo SWD).\n"
                               "- Confirma que a VCU esta alimentada (LV ligada).\n"
                               "- Termina o debug no CubeIDE, se estiver aberto.\n\n"
                               f"(detalhe: {err})") from err
        for (sym, _, expected), actual in zip(self.checks, flash):
            if actual != expected:
                session.close()
                raise RuntimeError(f"O firmware na placa nao corresponde ao {ELF_FILE.name} ({sym} diferente). "
                                   "Grava na placa o firmware desta compilacao.")
        self.session = session

    def read(self):
        if self.session is None or self.session.board is None:
            raise RuntimeError("Nao ligado a VCU")
        target = self.session.board.target
        return {v.expr: v.decode(bytes(target.read_memory_block8(v.address, v.size))) for v in self.variables}

    def read_apps_cal(self):
        """APPS calibration record in flash, or None (erased, corrupted, or no block in this firmware)."""
        if self.cal_address is None or self.session is None or self.session.board is None:
            return None
        return parse_apps_cal(bytes(self.session.board.target.read_memory_block8(self.cal_address, CAL_SIZE)))

    def write_apps_cal(self, cal):
        """Write the APPS calibration record to flash with openocd, which erases the block,
        writes, verifies and resets the VCU (the firmware loads the record at boot).
        The pyocd session is closed meanwhile - one program per ST-LINK - and reopened after."""
        if self.cal_address is None:
            raise RuntimeError(f"O {ELF_FILE.name} nao tem '{CAL_SYMBOL}' - compila e grava o firmware novo")
        exe, scripts = find_openocd()
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
            f.write(pack_apps_cal(cal))
        self.close()
        try:
            result = subprocess.run(
                [exe, "-s", scripts, "-f", "interface/stlink-dap.cfg", "-c", "transport select dapdirect_swd",
                 "-f", "target/stm32f7x.cfg",
                 "-c", f"program {{{Path(f.name).as_posix()}}} 0x{self.cal_address:08x} verify reset exit"],
                capture_output=True, text=True, timeout=60)
        finally:
            Path(f.name).unlink(missing_ok=True)
        if result.returncode != 0:
            raise RuntimeError("openocd falhou - calibracao NAO gravada. Se a VCU ficou parada, desliga e liga "
                               "a LV.\n\n" + result.stderr[-1500:])
        try:
            self.connect()
        except RuntimeError as err:
            raise RuntimeError(f"Calibracao gravada, mas a VCU nao voltou a ligar:\n\n{err}") from err

    def close(self):
        if self.session is not None:
            self.session.close()
            self.session = None


if __name__ == "__main__":
    live = VcuLive()
    for v in live.variables:
        print(f"{v.expr:34s} 0x{v.address:08x}  {v.size} B  {v.kind}")
    for expr in live.missing:
        print(f"{expr:34s} NAO ENCONTRADO no ELF")
    print(f"{CAL_SYMBOL:34s} " + (f"0x{live.cal_address:08x}  bloco da calibracao APPS" if live.cal_address
                                  else "NAO ENCONTRADO no ELF"))
    if "--symbols" in sys.argv:
        sys.exit(0)
    live.connect()
    print("Ligado. Ctrl+C para sair.")
    try:
        while True:
            values = live.read()
            print(" | ".join(f"{v.label}: {v.text(values[v.expr])}" for v in live.variables))
            time.sleep(0.5)
    finally:
        live.close()
