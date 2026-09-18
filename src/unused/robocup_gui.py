import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import math
import time
import serial
from serial.tools import list_ports

BAUD_RATE = 115200
FLAG_COMMANDS = [
    ("Target identified", "tgt", "target_identified"),
    ("Reverse triggered", "rev", "reverse_triggered"),
    ("Home reached", "home", "home_reached"),
    ("Dummy identified", "dum", "dummy_identified"),
    ("Metal identified", "met", "metal_identified"),
    ("Weight in entrance", "in", "weight_in_entrance"),
    ("Magnet hit", "hit", "magnet_hit"),
    ("No vertical", "nov", "no_vertical"),
    ("No horizontal", "noh", "no_horizontal"),
]

class RoboCupGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboCup Robot Monitor")
        self.root.geometry("1400x850")
        self.root.minsize(1100, 700)

        self.ser = None
        self.reader_stop = threading.Event()
        self.rx_queue = queue.Queue()

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.last_telemetry_time = None
        self.path_points = []
        self.map_scale_mm_per_px = 10.0

        self.flag_counts = {}
        self.flag_indicator_labels = {}

        self._build_ui()
        self.refresh_ports()
        self.root.after(50, self.process_rx_queue)
        self.root.after(250, self.update_telemetry_age)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        self._build_connection_bar()

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))

        left = ttk.Frame(body)
        right = ttk.Frame(body)
        body.add(left, weight=3)
        body.add(right, weight=2)

        self._build_map(left)
        self._build_right_panel(right)
        self._build_serial_log()

    def _build_connection_bar(self):
        bar = ttk.Frame(self.root, padding=8)
        bar.grid(row=0, column=0, sticky="ew")
        bar.columnconfigure(6, weight=1)

        ttk.Label(bar, text="Port:").grid(row=0, column=0)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var, state="readonly", width=16)
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(bar, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)

        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=4)

        self.connection_var = tk.StringVar(value="DISCONNECTED")
        ttk.Label(bar, textvariable=self.connection_var).grid(row=0, column=4, padx=(12, 4))

        self.telemetry_age_var = tk.StringVar(value="Telemetry: --")
        ttk.Label(bar, textvariable=self.telemetry_age_var).grid(row=0, column=5, padx=8)

        ttk.Button(bar, text="STOP", command=lambda: self.send_command("stop")).grid(row=0, column=7, padx=(12, 0))

    def _build_map(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Map / Relative Pose", padding=6)
        frame.pack(fill="both", expand=True)

        self.map_canvas = tk.Canvas(frame, background="white", highlightthickness=1, highlightbackground="gray")
        self.map_canvas.pack(fill="both", expand=True)
        self.map_canvas.bind("<Configure>", lambda e: self.redraw_map())

        self.pose_var = tk.StringVar(value="x=0 mm   y=0 mm   heading=0°")
        ttk.Label(frame, textvariable=self.pose_var).pack(anchor="w", pady=(6, 0))

    def _build_right_panel(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(4, weight=1)

        state = ttk.LabelFrame(parent, text="Robot State", padding=8)
        state.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        self.nav_state_var = tk.StringVar(value="--")
        self.collect_state_var = tk.StringVar(value="--")
        ttk.Label(state, text="Nav state:").grid(row=0, column=0, sticky="w")
        ttk.Label(state, textvariable=self.nav_state_var).grid(row=0, column=1, sticky="w", padx=8)
        ttk.Label(state, text="Collect state:").grid(row=1, column=0, sticky="w")
        ttk.Label(state, textvariable=self.collect_state_var).grid(row=1, column=1, sticky="w", padx=8)

        tune = ttk.LabelFrame(parent, text="Controller Tuning", padding=8)
        tune.grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        tune.columnconfigure(1, weight=1)

        self.turn_kp_var = tk.DoubleVar(value=7.0)
        self.drive_kp_var = tk.DoubleVar(value=10.0)
        self.drive_power_var = tk.IntVar(value=300)

        self._make_slider(tune, 0, "Turn Kp", self.turn_kp_var, 0, 20,
                          lambda: self.send_command(f"kp {self.turn_kp_var.get():.2f}"))
        self._make_slider(tune, 1, "Drive Kp", self.drive_kp_var, 0, 25,
                          lambda: self.send_command(f"drivekp {self.drive_kp_var.get():.2f}"))
        self._make_slider(tune, 2, "Drive power", self.drive_power_var, 0, 450,
                          lambda: self.send_command(f"drivepower {int(self.drive_power_var.get())}"))

        buttons = ttk.Frame(tune)
        buttons.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        ttk.Button(buttons, text="Read gains", command=lambda: self.send_command("gains")).pack(side="left")
        ttk.Button(buttons, text="Open gate", command=lambda: self.send_command("open")).pack(side="left", padx=6)
        ttk.Button(buttons, text="Close gate", command=lambda: self.send_command("close")).pack(side="left")

        flags = ttk.LabelFrame(parent, text="Test Flags", padding=8)
        flags.grid(row=2, column=0, sticky="ew", padx=4, pady=4)
        for row, (label, abbrev, full_name) in enumerate(FLAG_COMMANDS):
            ttk.Button(flags, text=label, command=lambda a=abbrev: self.send_command(f"flag {a}")).grid(
                row=row, column=0, sticky="ew", pady=2
            )
            ttk.Label(flags, text=f"flag {abbrev}").grid(row=row, column=1, sticky="w", padx=8)
            indicator = ttk.Label(flags, text="--", width=8)
            indicator.grid(row=row, column=2, sticky="e")
            self.flag_indicator_labels[full_name] = indicator
        ttk.Button(flags, text="Clear flag history", command=self.clear_flag_history).grid(
            row=len(FLAG_COMMANDS), column=0, columnspan=3, sticky="ew", pady=(8, 0)
        )

        tof = ttk.LabelFrame(parent, text="ToF Readings (mm)", padding=8)
        tof.grid(row=3, column=0, sticky="ew", padx=4, pady=4)
        names = [
            "Nav outer L", "Nav inner L", "Nav inner R", "Nav outer R",
            "Weight L top", "Weight L bottom", "Weight R top", "Weight R bottom", "Weight middle",
        ]
        self.tof_vars = {}
        for i, name in enumerate(names):
            var = tk.StringVar(value="--")
            self.tof_vars[name] = var
            r = i // 3
            c = (i % 3) * 2
            ttk.Label(tof, text=name + ":").grid(row=r, column=c, sticky="e")
            ttk.Label(tof, textvariable=var, width=6).grid(row=r, column=c+1, sticky="w", padx=(2, 8))

        hist = ttk.LabelFrame(parent, text="Raised Flag History", padding=6)
        hist.grid(row=4, column=0, sticky="nsew", padx=4, pady=4)
        hist.rowconfigure(0, weight=1)
        hist.columnconfigure(0, weight=1)
        self.flag_history = tk.Listbox(hist, height=6)
        self.flag_history.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(hist, orient="vertical", command=self.flag_history.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.flag_history.configure(yscrollcommand=sb.set)

    def _make_slider(self, parent, row, text, var, minimum, maximum, command):
        ttk.Label(parent, text=text).grid(row=row, column=0, sticky="w")
        scale = ttk.Scale(parent, from_=minimum, to=maximum, variable=var)
        scale.grid(row=row, column=1, sticky="ew", padx=6)

        value_label = ttk.Label(parent, width=7)
        value_label.grid(row=row, column=2)

        def update_label(_=None):
            if isinstance(var, tk.IntVar):
                value_label.configure(text=str(int(var.get())))
            else:
                value_label.configure(text=f"{var.get():.2f}")

        scale.configure(command=update_label)
        scale.bind("<ButtonRelease-1>", lambda e: command())
        update_label()

    def _build_serial_log(self):
        frame = ttk.LabelFrame(self.root, text="Serial Log", padding=6)
        frame.grid(row=2, column=0, sticky="ew", padx=8, pady=(0, 8))
        frame.columnconfigure(0, weight=1)

        self.serial_log = tk.Text(frame, height=10, wrap="none", state="disabled")
        self.serial_log.grid(row=0, column=0, columnspan=3, sticky="ew")

        self.command_var = tk.StringVar()
        entry = ttk.Entry(frame, textvariable=self.command_var)
        entry.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        entry.bind("<Return>", lambda e: self.send_manual_command())

        ttk.Button(frame, text="Send", command=self.send_manual_command).grid(row=1, column=1, padx=6, pady=(6, 0))
        ttk.Button(frame, text="Clear log", command=self.clear_log).grid(row=1, column=2, pady=(6, 0))

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Serial", "Select a COM port first.")
            return
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1, write_timeout=0.5)
        except serial.SerialException as exc:
            messagebox.showerror("Serial connection failed", str(exc))
            return

        self.reader_stop.clear()
        threading.Thread(target=self._reader_loop, daemon=True).start()
        self.connection_var.set(f"CONNECTED: {port}")
        self.connect_btn.configure(text="Disconnect")
        self.append_log(f"[GUI] Connected to {port}")
        self.root.after(250, lambda: self.send_command("gains"))

    def disconnect(self):
        self.reader_stop.set()
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.connection_var.set("DISCONNECTED")
        self.connect_btn.configure(text="Connect")

    def _reader_loop(self):
        while not self.reader_stop.is_set():
            try:
                raw = self.ser.readline()
            except (serial.SerialException, AttributeError) as exc:
                self.rx_queue.put(("ERROR", str(exc)))
                break
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.rx_queue.put(("LINE", line))

    def send_command(self, command):
        command = command.strip()
        if not command:
            return
        if not self.ser or not self.ser.is_open:
            self.append_log(f"[GUI] Not connected: {command}")
            return
        try:
            self.ser.write((command + "\n").encode("utf-8"))
            self.append_log(f"> {command}")
        except serial.SerialException as exc:
            self.append_log(f"[GUI] Write error: {exc}")

    def send_manual_command(self):
        cmd = self.command_var.get().strip()
        self.command_var.set("")
        self.send_command(cmd)

    def process_rx_queue(self):
        while True:
            try:
                kind, payload = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "ERROR":
                self.append_log(f"[GUI] Serial error: {payload}")
                self.disconnect()
            else:
                self.handle_serial_line(payload)

        self.root.after(50, self.process_rx_queue)

    def handle_serial_line(self, line):
        if line.startswith("ROBOT,"):
            self.parse_telemetry(line)
            return

        if line.startswith("FLAG,"):
            self.handle_flag_event(line)
            self.append_log(line)
            return

        self.parse_tuning_response(line)
        self.append_log(line)

    def parse_telemetry(self, line):
        # ROBOT,x,y,heading,navState,collectState,9 ToF values
        parts = line.split(",")
        if len(parts) < 15:
            self.append_log(f"[GUI] Bad telemetry packet: {line}")
            return

        try:
            self.robot_x = float(parts[1])
            self.robot_y = float(parts[2])
            self.robot_heading = float(parts[3])
            self.nav_state_var.set(parts[4])
            self.collect_state_var.set(parts[5])
            tof_values = [int(float(v)) for v in parts[6:15]]
        except ValueError:
            self.append_log(f"[GUI] Could not parse telemetry: {line}")
            return

        self.last_telemetry_time = time.monotonic()
        self.pose_var.set(
            f"x={self.robot_x:.0f} mm   y={self.robot_y:.0f} mm   heading={self.robot_heading:.1f}°"
        )

        for name, value in zip(self.tof_vars.keys(), tof_values):
            self.tof_vars[name].set(str(value))

        self.path_points.append((self.robot_x, self.robot_y))
        if len(self.path_points) > 500:
            self.path_points.pop(0)

        self.redraw_map()

    def handle_flag_event(self, line):
        flag_name = line.split(",", 1)[1].strip()
        self.flag_counts[flag_name] = self.flag_counts.get(flag_name, 0) + 1
        count = self.flag_counts[flag_name]

        self.flag_history.insert(tk.END, f"{time.strftime('%H:%M:%S')}  {flag_name}  ({count})")
        self.flag_history.yview_moveto(1.0)

        if flag_name in self.flag_indicator_labels:
            self.flag_indicator_labels[flag_name].configure(text=f"Seen {count}")

    def parse_tuning_response(self, line):
        try:
            if line.startswith("Turn KP ="):
                self.turn_kp_var.set(float(line.split("=", 1)[1].strip()))
            elif line.startswith("Drive KP ="):
                self.drive_kp_var.set(float(line.split("=", 1)[1].strip()))
            elif line.startswith("Drive Power ="):
                self.drive_power_var.set(int(float(line.split("=", 1)[1].strip())))
        except ValueError:
            pass

    def redraw_map(self):
        c = self.map_canvas
        c.delete("all")
        w = max(c.winfo_width(), 10)
        h = max(c.winfo_height(), 10)
        cx, cy = w / 2, h / 2

        grid_px = 500 / self.map_scale_mm_per_px
        x = cx
        while x <= w:
            c.create_line(x, 0, x, h, fill="#eeeeee")
            x += grid_px
        x = cx - grid_px
        while x >= 0:
            c.create_line(x, 0, x, h, fill="#eeeeee")
            x -= grid_px
        y = cy
        while y <= h:
            c.create_line(0, y, w, y, fill="#eeeeee")
            y += grid_px
        y = cy - grid_px
        while y >= 0:
            c.create_line(0, y, w, y, fill="#eeeeee")
            y -= grid_px

        c.create_line(0, cy, w, cy, fill="#bbbbbb")
        c.create_line(cx, 0, cx, h, fill="#bbbbbb")
        c.create_text(cx + 8, cy + 8, text="start", anchor="nw")

        if len(self.path_points) >= 2:
            pts = []
            for x_mm, y_mm in self.path_points:
                pts.extend((cx + x_mm / self.map_scale_mm_per_px,
                            cy - y_mm / self.map_scale_mm_per_px))
            c.create_line(*pts, fill="#999999")

        rx = cx + self.robot_x / self.map_scale_mm_per_px
        ry = cy - self.robot_y / self.map_scale_mm_per_px
        a = math.radians(self.robot_heading)
        r = 16

        nose = (rx + r * math.cos(a), ry - r * math.sin(a))
        left = (rx + 0.75*r * math.cos(a + 2.5), ry - 0.75*r * math.sin(a + 2.5))
        right = (rx + 0.75*r * math.cos(a - 2.5), ry - 0.75*r * math.sin(a - 2.5))

        c.create_polygon(*nose, *left, *right, outline="black", fill="", width=2)
        c.create_oval(rx-2, ry-2, rx+2, ry+2, fill="black")

    def update_telemetry_age(self):
        if self.last_telemetry_time is None:
            self.telemetry_age_var.set("Telemetry: --")
        else:
            age = time.monotonic() - self.last_telemetry_time
            self.telemetry_age_var.set(f"Telemetry age: {age:.1f}s")
        self.root.after(250, self.update_telemetry_age)

    def append_log(self, text):
        self.serial_log.configure(state="normal")
        self.serial_log.insert(tk.END, text + "\n")
        self.serial_log.see(tk.END)
        self.serial_log.configure(state="disabled")

    def clear_log(self):
        self.serial_log.configure(state="normal")
        self.serial_log.delete("1.0", tk.END)
        self.serial_log.configure(state="disabled")

    def clear_flag_history(self):
        self.flag_counts.clear()
        self.flag_history.delete(0, tk.END)
        for label in self.flag_indicator_labels.values():
            label.configure(text="--")

    def on_close(self):
        self.disconnect()
        self.root.destroy()

def main():
    root = tk.Tk()
    RoboCupGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
