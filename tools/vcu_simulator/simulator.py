"""VCU Powertrain Simulator — tkinter GUI + python-can.

Simulates CAN2 (Powertrain Bus) inputs for VCU firmware testing.
Hardware: WeActStudio USB2CANFD V1 (SLCAN interface).

Usage:
    python simulator.py
    python simulator.py --port COM8
    python simulator.py --port COM8 --bitrate 1000000
"""

import argparse
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime

try:
    import can
except ImportError:
    print("python-can not installed. Run: pip install python-can")
    sys.exit(1)

from can_messages import (
    # IDs
    R2D_AND_IGN_ID, APPS_ADC_RAW_ID, BMS_PRECHARGE_STATE_ID, IVT_RESULT_W_ID,
    VCU_3_ID, VCU_2_ID,
    # Encoders
    encode_r2d_and_ign, encode_apps_raw, encode_bms_precharge_state,
    encode_ivt_result_w,
    get_all_inv1_messages, get_all_inv2_messages,
    # Decoders
    decode_vcu_3, decode_vcu_2,
)


class VCUSimulator:
    """Main simulator class — CAN bus + GUI."""

    def __init__(self, root: tk.Tk, port: str = None, bitrate: int = 1000000):
        self.root = root
        self.root.title("VCU Powertrain Simulator")
        self.root.geometry("1100x820")
        self.root.resizable(True, True)

        self.port = port
        self.bitrate = bitrate
        self.bus = None
        self.running = False
        self.connected = False

        # Simulator state
        self.ignition = False
        self.r2d_button = False
        self.apps1_adc = 0
        self.apps2_adc = 0
        self.brake_pressure = 0
        self.precharge_state = 0
        self.pack_voltage = 600  # Default HV battery voltage

        # INV1 mock values
        self.inv1_erpm = 0
        self.inv1_duty = 0
        self.inv1_voltage = 600
        self.inv1_ac_current = 0
        self.inv1_dc_current = 0
        self.inv1_temp_controller = 250  # 25.0 C (scale 0.1)
        self.inv1_temp_motor = 250
        self.inv1_fault_code = 0
        self.inv1_drive_enable = False

        # INV2 mock values
        self.inv2_erpm = 0
        self.inv2_duty = 0
        self.inv2_voltage = 600
        self.inv2_ac_current = 0
        self.inv2_dc_current = 0
        self.inv2_temp_controller = 250
        self.inv2_temp_motor = 250
        self.inv2_fault_code = 0
        self.inv2_drive_enable = False

        # VCU telemetry (from CAN1 via VCU_3)
        self.vcu_state = "DISCONNECTED"
        self.vcu_rpm = 0
        self.vcu_voltage = 0

        # TX/RX counters
        self.tx_count = 0
        self.rx_count = 0

        # Log buffer
        self.log_lines = []
        self.max_log_lines = 200

        self._build_gui()
        self._detect_ports()

        if self.port:
            self.port_var.set(self.port)

    def _build_gui(self):
        """Build the tkinter GUI."""
        style = ttk.Style()
        style.configure("Status.TLabel", font=("Consolas", 10, "bold"))
        style.configure("Big.TButton", font=("Consolas", 10))

        # ──── Top bar: connection ────
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=5)
        conn_frame.pack(fill=tk.X, padx=5, pady=2)

        ttk.Label(conn_frame, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12)
        self.port_combo.pack(side=tk.LEFT, padx=3)
        self.refresh_btn = ttk.Button(conn_frame, text="Refresh", command=self._detect_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=2)

        ttk.Label(conn_frame, text="Bitrate:").pack(side=tk.LEFT, padx=(10, 0))
        self.bitrate_var = tk.StringVar(value=str(self.bitrate))
        bitrate_combo = ttk.Combobox(conn_frame, textvariable=self.bitrate_var, width=10,
                                      values=["500000", "1000000"], state="readonly")
        bitrate_combo.pack(side=tk.LEFT, padx=3)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(conn_frame, text="DISCONNECTED", style="Status.TLabel",
                                       foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=10)

        ttk.Label(conn_frame, text="TX:").pack(side=tk.LEFT, padx=(20, 0))
        self.tx_label = ttk.Label(conn_frame, text="0", width=8)
        self.tx_label.pack(side=tk.LEFT)
        ttk.Label(conn_frame, text="RX:").pack(side=tk.LEFT)
        self.rx_label = ttk.Label(conn_frame, text="0", width=8)
        self.rx_label.pack(side=tk.LEFT)

        # ──── Main content: left panel + right panel ────
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=2)

        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

        # ──── LEFT: Driver Inputs ────
        driver_frame = ttk.LabelFrame(left_panel, text="Driver Inputs", padding=5)
        driver_frame.pack(fill=tk.X, pady=2)

        # IGN toggle
        self.ign_var = tk.BooleanVar(value=False)
        self.ign_check = ttk.Checkbutton(driver_frame, text="IGNITION (0x740[0])",
                                           variable=self.ign_var, command=self._on_ign_change)
        self.ign_check.pack(anchor=tk.W, pady=1)

        # R2D button (momentary)
        r2d_frame = ttk.Frame(driver_frame)
        r2d_frame.pack(fill=tk.X, pady=1)
        self.r2d_btn = tk.Button(r2d_frame, text="R2D PRESS", bg="#4CAF50", fg="white",
                                  font=("Consolas", 10, "bold"), relief=tk.RAISED)
        self.r2d_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.r2d_btn.bind("<ButtonPress>", self._on_r2d_press)
        self.r2d_btn.bind("<ButtonRelease>", self._on_r2d_release)
        self.r2d_state_label = ttk.Label(r2d_frame, text="Released", width=12)
        self.r2d_state_label.pack(side=tk.LEFT, padx=5)

        # APPS sliders
        ttk.Label(driver_frame, text="APPS1 (0x710[0:1]) — Raw ADC").pack(anchor=tk.W, pady=(5, 0))
        self.apps1_var = tk.IntVar(value=0)
        self.apps1_slider = ttk.Scale(driver_frame, from_=0, to=4095, variable=self.apps1_var,
                                       orient=tk.HORIZONTAL, command=self._on_apps_change)
        self.apps1_slider.pack(fill=tk.X)
        self.apps1_label = ttk.Label(driver_frame, text="0 (0.0%)")
        self.apps1_label.pack(anchor=tk.W)

        ttk.Label(driver_frame, text="APPS2 (0x710[2:3]) — Raw ADC").pack(anchor=tk.W, pady=(5, 0))
        self.apps2_var = tk.IntVar(value=0)
        self.apps2_slider = ttk.Scale(driver_frame, from_=0, to=4095, variable=self.apps2_var,
                                       orient=tk.HORIZONTAL, command=self._on_apps_change)
        self.apps2_slider.pack(fill=tk.X)
        self.apps2_label = ttk.Label(driver_frame, text="0 (0.0%)")
        self.apps2_label.pack(anchor=tk.W)

        # ──── LEFT: BMS Mock ────
        bms_frame = ttk.LabelFrame(left_panel, text="BMS Mock", padding=5)
        bms_frame.pack(fill=tk.X, pady=2)

        precharge_row = ttk.Frame(bms_frame)
        precharge_row.pack(fill=tk.X)
        ttk.Label(precharge_row, text="Precharge State (0x702[1]):").pack(side=tk.LEFT)
        self.precharge_var = tk.IntVar(value=0)
        self.precharge_spin = ttk.Spinbox(precharge_row, from_=0, to=9, width=5,
                                           textvariable=self.precharge_var)
        self.precharge_spin.pack(side=tk.LEFT, padx=5)
        ttk.Label(precharge_row, text="9 = done").pack(side=tk.LEFT)

        voltage_row = ttk.Frame(bms_frame)
        voltage_row.pack(fill=tk.X, pady=(3, 0))
        ttk.Label(voltage_row, text="Pack Voltage:").pack(side=tk.LEFT)
        self.voltage_var = tk.IntVar(value=600)
        self.voltage_slider = ttk.Scale(voltage_row, from_=0, to=800, variable=self.voltage_var,
                                         orient=tk.HORIZONTAL)
        self.voltage_slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.voltage_label = ttk.Label(voltage_row, text="600 V", width=8)
        self.voltage_label.pack(side=tk.LEFT)

        # ──── LEFT: INV1 Mock ────
        inv1_frame = ttk.LabelFrame(left_panel, text="INV1 Mock (0x404-0x484)", padding=5)
        inv1_frame.pack(fill=tk.X, pady=2)

        self.inv1_erpm_var = tk.IntVar(value=0)
        self._build_inv_slider(inv1_frame, "ERPM:", self.inv1_erpm_var, -120000, 120000, "inv1_erpm")

        self.inv1_temp_var = tk.IntVar(value=250)
        self._build_inv_slider(inv1_frame, "Temp ctrl (0.1C):", self.inv1_temp_var, -550, 2000, "inv1_temp")

        self.inv1_drive_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(inv1_frame, text="Drive Enable", variable=self.inv1_drive_var).pack(anchor=tk.W)

        # ──── LEFT: INV2 Mock ────
        inv2_frame = ttk.LabelFrame(left_panel, text="INV2 Mock (0x405-0x485)", padding=5)
        inv2_frame.pack(fill=tk.X, pady=2)

        self.inv2_erpm_var = tk.IntVar(value=0)
        self._build_inv_slider(inv2_frame, "ERPM:", self.inv2_erpm_var, -120000, 120000, "inv2_erpm")

        self.inv2_temp_var = tk.IntVar(value=250)
        self._build_inv_slider(inv2_frame, "Temp ctrl (0.1C):", self.inv2_temp_var, -550, 2000, "inv2_temp")

        self.inv2_drive_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(inv2_frame, text="Drive Enable", variable=self.inv2_drive_var).pack(anchor=tk.W)

        # ──── RIGHT: VCU State Display ────
        vcu_frame = ttk.LabelFrame(right_panel, text="VCU State (from CAN1 Telemetry)", padding=5)
        vcu_frame.pack(fill=tk.X, pady=2)

        self.vcu_state_label = ttk.Label(vcu_frame, text="DISCONNECTED", font=("Consolas", 14, "bold"),
                                          foreground="gray")
        self.vcu_state_label.pack(anchor=tk.W)

        info_row = ttk.Frame(vcu_frame)
        info_row.pack(fill=tk.X)
        ttk.Label(info_row, text="IGN:").pack(side=tk.LEFT)
        self.vcu_ign_label = ttk.Label(info_row, text="OFF", width=6, font=("Consolas", 10))
        self.vcu_ign_label.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(info_row, text="R2D:").pack(side=tk.LEFT)
        self.vcu_r2d_label = ttk.Label(info_row, text="OFF", width=6, font=("Consolas", 10))
        self.vcu_r2d_label.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(info_row, text="RPM:").pack(side=tk.LEFT)
        self.vcu_rpm_label = ttk.Label(info_row, text="0", width=8, font=("Consolas", 10))
        self.vcu_rpm_label.pack(side=tk.LEFT)
        ttk.Label(info_row, text="Voltage:").pack(side=tk.LEFT)
        self.vcu_volt_label = ttk.Label(info_row, text="0V", width=8, font=("Consolas", 10))
        self.vcu_volt_label.pack(side=tk.LEFT)

        # ──── RIGHT: Cycle Time ────
        timing_frame = ttk.LabelFrame(right_panel, text="Transmission Timing", padding=5)
        timing_frame.pack(fill=tk.X, pady=2)

        timing_row = ttk.Frame(timing_frame)
        timing_row.pack(fill=tk.X)
        ttk.Label(timing_row, text="Driver inputs (ms):").pack(side=tk.LEFT)
        self.driver_interval_var = tk.IntVar(value=10)
        ttk.Spinbox(timing_row, from_=5, to=1000, width=6, textvariable=self.driver_interval_var).pack(side=tk.LEFT, padx=3)

        timing_row2 = ttk.Frame(timing_frame)
        timing_row2.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(timing_row2, text="Inverter status (ms):").pack(side=tk.LEFT)
        self.inv_interval_var = tk.IntVar(value=10)
        ttk.Spinbox(timing_row2, from_=5, to=1000, width=6, textvariable=self.inv_interval_var).pack(side=tk.LEFT, padx=3)

        timing_row3 = ttk.Frame(timing_frame)
        timing_row3.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(timing_row3, text="BMS mock (ms):").pack(side=tk.LEFT)
        self.bms_interval_var = tk.IntVar(value=100)
        ttk.Spinbox(timing_row3, from_=10, to=5000, width=6, textvariable=self.bms_interval_var).pack(side=tk.LEFT, padx=3)

        # ──── RIGHT: Quick Actions ────
        actions_frame = ttk.LabelFrame(right_panel, text="Quick Actions", padding=5)
        actions_frame.pack(fill=tk.X, pady=2)

        ttk.Button(actions_frame, text="Ignition ON + Precharge Done",
                    command=self._quick_ign_precharge).pack(fill=tk.X, pady=1)
        ttk.Button(actions_frame, text="Full Manual R2D Sequence",
                    command=self._quick_manual_r2d).pack(fill=tk.X, pady=1)
        ttk.Button(actions_frame, text="Reset All to Zero",
                    command=self._quick_reset).pack(fill=tk.X, pady=1)

        # ──── CAN Log ────
        log_frame = ttk.LabelFrame(right_panel, text="CAN Log", padding=3)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=2)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, font=("Consolas", 8),
                                                   state=tk.DISABLED, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        log_btn_frame = ttk.Frame(log_frame)
        log_btn_frame.pack(fill=tk.X)
        ttk.Button(log_btn_frame, text="Clear Log", command=self._clear_log).pack(side=tk.LEFT)
        self.logautoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(log_btn_frame, text="Auto-scroll", variable=self.logautoscroll_var).pack(side=tk.LEFT, padx=5)

    def _build_inv_slider(self, parent, label_text, var, from_val, to_val, prefix):
        """Helper to build an inverter slider row."""
        row = ttk.Frame(parent)
        row.pack(fill=tk.X)
        ttk.Label(row, text=label_text, width=16).pack(side=tk.LEFT)
        slider = ttk.Scale(row, from_=from_val, to=to_val, variable=var, orient=tk.HORIZONTAL)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        lbl = ttk.Label(row, text="0", width=8)
        lbl.pack(side=tk.LEFT)
        var.trace_add("write", lambda *a, v=var, l=lbl: l.configure(text=str(v.get())))

    def _detect_ports(self):
        """Detect available serial ports."""
        import serial.tools.list_ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        """Connect or disconnect from CAN bus."""
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        """Open SLCAN connection."""
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Select a COM port first")
            return

        bitrate = int(self.bitrate_var.get())
        try:
            self.bus = can.interface.Bus(
                channel=port,
                interface='slcan',
                bitrate=bitrate,
                tty_baudrate=115200,
            )
            self.connected = True
            self.running = True
            self.connect_btn.configure(text="Disconnect")
            self.status_label.configure(text="CONNECTED", foreground="green")
            self._log(f"Connected to {port} @ {bitrate/1e6:.1f} Mbps")

            # Start TX threads
            self._start_tx_threads()
            # Start RX listener
            self._start_rx_listener()

        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self._log(f"ERROR: {e}")

    def _disconnect(self):
        """Close CAN bus."""
        self.running = False
        self.connected = False
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self.connect_btn.configure(text="Connect")
        self.status_label.configure(text="DISCONNECTED", foreground="red")
        self._log("Disconnected")

    def _start_tx_threads(self):
        """Start periodic TX threads for each message group."""
        threading.Thread(target=self._tx_driver_loop, daemon=True).start()
        threading.Thread(target=self._tx_inv_loop, daemon=True).start()
        threading.Thread(target=self._tx_bms_loop, daemon=True).start()

    def _tx_driver_loop(self):
        """Send R2D/IGN + APPS at configured interval."""
        while self.running and self.connected:
            try:
                interval = self.driver_interval_var.get() / 1000.0
            except Exception:
                interval = 0.01

            # R2D + IGN
            msg = can.Message(
                arbitration_id=R2D_AND_IGN_ID,
                data=encode_r2d_and_ign(self.ignition, self.r2d_button),
                is_extended_id=False,
            )
            self._send(msg, "0x740 IGN+R2D")

            # APPS
            msg = can.Message(
                arbitration_id=APPS_ADC_RAW_ID,
                data=encode_apps_raw(self.apps1_adc, self.apps2_adc),
                is_extended_id=False,
            )
            self._send(msg, f"0x710 APPS [{self.apps1_adc}, {self.apps2_adc}]")

            time.sleep(interval)

    def _tx_inv_loop(self):
        """Send INV1 + INV2 status frames at configured interval."""
        while self.running and self.connected:
            try:
                interval = self.inv_interval_var.get() / 1000.0
            except Exception:
                interval = 0.01

            # INV1
            for cid, data in get_all_inv1_messages(
                erpm=self.inv1_erpm_var.get(),
                duty=0,
                voltage=600,
                ac_current=0, dc_current=0,
                temp_controller=self.inv1_temp_var.get(),
                temp_motor=250, fault_code=0,
                foc_id=0, foc_iq=0,
                throttle=0, brake=0,
                drive_enable=self.inv1_drive_var.get(),
            ):
                msg = can.Message(arbitration_id=cid, data=data, is_extended_id=False)
                self._send(msg, f"INV1 0x{cid:03X}")

            # INV2
            for cid, data in get_all_inv2_messages(
                erpm=self.inv2_erpm_var.get(),
                duty=0,
                voltage=600,
                ac_current=0, dc_current=0,
                temp_controller=self.inv2_temp_var.get(),
                temp_motor=250, fault_code=0,
                foc_id=0, foc_iq=0,
                throttle=0, brake=0,
                drive_enable=self.inv2_drive_var.get(),
            ):
                msg = can.Message(arbitration_id=cid, data=data, is_extended_id=False)
                self._send(msg, f"INV2 0x{cid:03X}")

            time.sleep(interval)

    def _tx_bms_loop(self):
        """Send BMS precharge + IVT at configured interval."""
        while self.running and self.connected:
            try:
                interval = self.bms_interval_var.get() / 1000.0
            except Exception:
                interval = 0.1

            # BMS precharge state
            msg = can.Message(
                arbitration_id=BMS_PRECHARGE_STATE_ID,
                data=encode_bms_precharge_state(self.precharge_var.get()),
                is_extended_id=False,
            )
            self._send(msg, f"0x702 Precharge={self.precharge_var.get()}")

            # IVT power (fake 0W)
            msg = can.Message(
                arbitration_id=IVT_RESULT_W_ID,
                data=encode_ivt_result_w(0),
                is_extended_id=False,
            )
            self._send(msg, "0x526 IVT P=0W")

            time.sleep(interval)

    def _send(self, msg: can.Message, label: str):
        """Send a CAN message and log it."""
        if not self.connected or not self.bus:
            return
        try:
            self.bus.send(msg)
            self.tx_count += 1
            self.root.after(0, self._update_tx_count)
            self.root.after(0, self._log, f"[TX] {label}: {msg.data.hex(' ')}")
        except Exception as e:
            self._log(f"[TX ERR] {label}: {e}")

    def _start_rx_listener(self):
        """Listen for VCU telemetry frames on CAN bus."""
        def _rx_thread():
            while self.running and self.connected:
                try:
                    msg = self.bus.recv(timeout=0.1)
                    if msg is None:
                        continue
                    self.rx_count += 1
                    self.root.after(0, self._update_rx_count)

                    if msg.arbitration_id == VCU_3_ID:
                        decoded = decode_vcu_3(msg.data)
                        if decoded:
                            self.root.after(0, self._update_vcu_display, decoded)
                    elif msg.arbitration_id == VCU_2_ID:
                        decoded = decode_vcu_2(msg.data)
                        if decoded:
                            self.root.after(0, self._log,
                                           f"[RX] VCU_2: faults=0x{decoded['faults']:04X} "
                                           f"lmt1={decoded['lmt1']} lmt2={decoded['lmt2']}")
                except Exception:
                    pass

        threading.Thread(target=_rx_thread, daemon=True).start()

    # ──── GUI callbacks ────

    def _on_ign_change(self):
        self.ignition = self.ign_var.get()

    def _on_r2d_press(self, event):
        self.r2d_button = True
        self.r2d_btn.configure(bg="#F44336", text="R2D ACTIVE")
        self.r2d_state_label.configure(text="Pressed")

    def _on_r2d_release(self, event):
        self.r2d_button = False
        self.r2d_btn.configure(bg="#4CAF50", text="R2D PRESS")
        self.r2d_state_label.configure(text="Released")

    def _on_apps_change(self, *args):
        v1 = self.apps1_var.get()
        v2 = self.apps2_var.get()
        pct1 = v1 / 4095 * 100
        pct2 = v2 / 4095 * 100
        self.apps1_label.configure(text=f"{v1} ({pct1:.1f}%)")
        self.apps2_label.configure(text=f"{v2} ({pct2:.1f}%)")
        self.apps1_adc = v1
        self.apps2_adc = v2

    # ──── Quick actions ────

    def _quick_ign_precharge(self):
        """Set IGN ON + precharge done."""
        self.ign_var.set(True)
        self.ignition = True
        self.precharge_var.set(9)
        self._log("[QUICK] IGN ON + Precharge=9")

    def _quick_manual_r2d(self):
        """Full manual R2D sequence: IGN + precharge + R2D press."""
        self._quick_ign_precharge()
        self.root.after(200, self._do_r2d_press)

    def _do_r2d_press(self):
        self.r2d_button = True
        self.r2d_btn.configure(bg="#F44336", text="R2D ACTIVE")
        self.r2d_state_label.configure(text="Pressed")
        self.root.after(300, self._do_r2d_release)

    def _do_r2d_release(self):
        self.r2d_button = False
        self.r2d_btn.configure(bg="#4CAF50", text="R2D PRESS")
        self.r2d_state_label.configure(text="Released")
        self._log("[QUICK] R2D button pressed + released")

    def _quick_reset(self):
        """Reset all inputs to zero."""
        self.ign_var.set(False)
        self.ignition = False
        self.r2d_button = False
        self.r2d_btn.configure(bg="#4CAF50", text="R2D PRESS")
        self.r2d_state_label.configure(text="Released")
        self.apps1_var.set(0)
        self.apps2_var.set(0)
        self.precharge_var.set(0)
        self.inv1_erpm_var.set(0)
        self.inv1_temp_var.set(250)
        self.inv1_drive_var.set(False)
        self.inv2_erpm_var.set(0)
        self.inv2_temp_var.set(250)
        self.inv2_drive_var.set(False)
        self._log("[QUICK] Reset all to zero")

    # ──── UI updates ────

    def _update_tx_count(self):
        self.tx_label.configure(text=str(self.tx_count))

    def _update_rx_count(self):
        self.rx_label.configure(text=str(self.rx_count))

    def _update_vcu_display(self, decoded: dict):
        ign = "ON" if decoded.get("ign") else "OFF"
        r2d = "ON" if decoded.get("r2d") else "OFF"
        rpm = decoded.get("rpm", 0)
        volt = decoded.get("voltage", 0)
        self.vcu_ign_label.configure(text=ign, foreground="green" if decoded.get("ign") else "red")
        self.vcu_r2d_label.configure(text=r2d, foreground="green" if decoded.get("r2d") else "red")
        self.vcu_rpm_label.configure(text=str(rpm))
        self.vcu_volt_label.configure(text=f"{volt}V")

        # Infer state from signals
        if decoded.get("r2d"):
            self.vcu_state = "READY"
            self.vcu_state_label.configure(text="READY", foreground="green")
        elif decoded.get("ign"):
            self.vcu_state = "IGN ON"
            self.vcu_state_label.configure(text="IGN ON", foreground="orange")
        else:
            self.vcu_state = "STANDBY"
            self.vcu_state_label.configure(text="STANDBY", foreground="gray")

    def _log(self, text: str):
        """Append to log widget."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{timestamp}] {text}\n"

        def _do():
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.insert(tk.END, line)
            # Trim old lines
            self.log_lines.append(line)
            if len(self.log_lines) > self.max_log_lines:
                self.log_lines = self.log_lines[-self.max_log_lines:]
                self.log_text.delete("1.0", f"{self.log_text.index('end-1c')}-{self.max_log_lines}l")
            if self.logautoscroll_var.get():
                self.log_text.see(tk.END)
            self.log_text.configure(state=tk.DISABLED)

        self.root.after(0, _do)

    def _clear_log(self):
        self.log_lines.clear()
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def on_close(self):
        """Clean shutdown."""
        self.running = False
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="VCU Powertrain Simulator")
    parser.add_argument("--port", "-p", help="Serial port (e.g. COM8)")
    parser.add_argument("--bitrate", "-b", type=int, default=1000000, help="CAN bitrate (default: 1000000)")
    args = parser.parse_args()

    root = tk.Tk()
    sim = VCUSimulator(root, port=args.port, bitrate=args.bitrate)
    root.protocol("WM_DELETE_WINDOW", sim.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
