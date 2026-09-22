import math
import queue
import threading
import time
import tkinter as tk

from tkinter import ttk, messagebox, filedialog
from datetime import datetime
from pathlib import Path

import serial

from robocup_gui import RoboCupGUI, BAUD_RATE
from session_tools import (
    SessionRecorder,
    parse_config,
    command_allowed,
    CORNERS,
    MODULES,
)


class TestGUI(RoboCupGUI):
    def __init__(self, root):
        self.recorder = SessionRecorder()
        self.config = None
        self.mode = None
        self.locked = True
        self.lock_latched = False
        self.run_latched = False
        self.load_next = False

        self.debug_state = {}
        self.gains = {}
        self.pending = None
        self.generation = 0
        self.reader = None
        self.setup_widgets = []

        super().__init__(root)

        self.root.title("RoboCup setup and test recorder")
        self.root.after(1000, self.poll_status)
        self.refresh_setup()

    def _build_right_panel(self, parent):
        tabs = ttk.Notebook(parent)
        tabs.pack(fill="both", expand=True)

        setup = ttk.Frame(tabs, padding=8)
        tests = ttk.Frame(tabs, padding=8)
        controls = ttk.Frame(tabs)

        tabs.add(setup, text="PRE-RUN setup")
        tabs.add(tests, text="Unit / Debug Test")
        tabs.add(controls, text="Existing controls")

        self.build_setup(setup)
        self.build_tests(tests)

        # Keep the original controls, with scrolling for smaller screens.
        canvas = tk.Canvas(controls, highlightthickness=0)
        scroll = ttk.Scrollbar(controls, command=canvas.yview)
        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        canvas.configure(yscrollcommand=scroll.set)

        inner = ttk.Frame(canvas)
        window = canvas.create_window(0, 0, window=inner, anchor="nw")

        inner.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.bind(
            "<Configure>",
            lambda e: canvas.itemconfigure(window, width=e.width),
        )

        super()._build_right_panel(inner)
        self.turn_kp_var.set(22.0)

    def _make_slider(
        self, parent, row, text, var, minimum, maximum, command
    ):
        if "Kp" in text:
            maximum = 100

        super()._make_slider(
            parent, row, text, var, minimum, maximum, command
        )

    def _build_map(self, parent):
        frame = ttk.LabelFrame(
            parent,
            text="Arena: click to draft start position",
            padding=6,
        )
        frame.pack(fill="both", expand=True)

        self.map_canvas = tk.Canvas(frame, background="white")
        self.map_canvas.pack(fill="both", expand=True)
        self.map_canvas.bind(
            "<Configure>", lambda e: self.redraw_map()
        )
        self.map_canvas.bind("<Button-1>", self.map_click)

        self.pose_var = tk.StringVar(
            value="Waiting for ROBOT telemetry"
        )
        ttk.Label(frame, textvariable=self.pose_var).pack(anchor="w")

    def build_setup(self, parent):
        self.base_var = tk.StringVar(value="blue")
        self.blue_var = tk.StringVar(value="sw")
        self.green_var = tk.StringVar(value="ne")

        self.x_var = tk.StringVar(value="300")
        self.y_var = tk.StringVar(value="300")
        self.heading_var = tk.StringVar(value="0")
        self.pickup_var = tk.BooleanVar(value=True)

        self.setup_status = tk.StringVar(
            value="Connect and read firmware configuration."
        )

        ttk.Label(
            parent,
            text="Draft settings — Apply sends them to the Teensy.",
        ).pack(anchor="w")

        choices = [
            ("Our base colour", self.base_var, ("blue", "green")),
            ("Blue corner", self.blue_var, tuple(CORNERS)),
            ("Green corner", self.green_var, tuple(CORNERS)),
        ]

        for label, var, values in choices:
            row = ttk.Frame(parent)
            row.pack(fill="x", pady=4)

            ttk.Label(row, text=label, width=20).pack(side="left")

            widget = ttk.Combobox(
                row,
                textvariable=var,
                values=values,
                state="readonly",
                width=12,
            )
            widget.pack(side="left")
            widget.bind(
                "<<ComboboxSelected>>",
                lambda e: self.draft_changed(),
            )

            self.setup_widgets.append((widget, "readonly"))

        fields = [
            ("Start X (mm)", self.x_var),
            ("Start Y (mm)", self.y_var),
            ("Heading (degrees)", self.heading_var),
        ]

        for label, var in fields:
            row = ttk.Frame(parent)
            row.pack(fill="x", pady=4)

            ttk.Label(row, text=label, width=20).pack(side="left")

            widget = ttk.Entry(row, textvariable=var, width=14)
            widget.pack(side="left")
            widget.bind("<KeyRelease>", lambda e: self.draft_changed())

            self.setup_widgets.append((widget, "normal"))

        buttons = [
            ("Use our base centre as start", self.use_centre),
            ("Apply to Teensy", self.apply_setup),
            (
                "Lock configuration",
                lambda: self.send_command("config lock"),
            ),
        ]

        for text, action in buttons:
            widget = ttk.Button(parent, text=text, command=action)
            widget.pack(fill="x", pady=4)
            self.setup_widgets.append((widget, "normal"))

        widget = ttk.Checkbutton(
            parent,
            text="Collect weights (auto); unchecked = roam only",
            variable=self.pickup_var,
            command=self.draft_changed,
        )
        widget.pack(anchor="w", pady=4)
        self.setup_widgets.append((widget, "normal"))

        ttk.Button(
            parent,
            text="Read / load firmware configuration",
            command=self.load_config,
        ).pack(fill="x")

        ttk.Label(
            parent,
            textvariable=self.setup_status,
            wraplength=410,
        ).pack(anchor="w", pady=10)

        self.confirmed_text = tk.StringVar(
            value="Firmware configuration: unknown"
        )

        ttk.Label(
            parent,
            textvariable=self.confirmed_text,
            wraplength=410,
        ).pack(anchor="w")

        ttk.Label(
            parent,
            wraplength=410,
            text=(
                "0° faces +X (right); 90° faces +Y (up). "
                "Press physical GO to start.\n"
                "The other base corner is a drawing reference only; "
                "only our base is sent.\n"
                "Blue/green regions and orange start marker show the draft. "
                "The black home marker shows the firmware-confirmed target."
            ),
        ).pack(anchor="w", pady=12)

    def build_tests(self, parent):
        self.record_status = tk.StringVar(value="Not recording")

        ttk.Label(
            parent,
            textvariable=self.record_status,
            font=("", 14, "bold"),
        ).pack(pady=8)

        for text, action in [
            ("Start Test", self.start_test),
            ("Stop Test", self.stop_test),
            ("Export Test (.txt)", self.export_test),
        ]:
            ttk.Button(
                parent, text=text, command=action
            ).pack(fill="x", pady=5)

        tk.Button(
            parent,
            text="FAULT",
            command=self.fault,
            bg="#b00020",
            fg="white",
            font=("", 24, "bold"),
            height=2,
        ).pack(fill="x", pady=15)

        ttk.Label(
            parent,
            wraplength=410,
            text=(
                "FAULT records a local marker and requests a harmless "
                "robot-time echo.\n"
                "The recorder does not enable or disable debug modules. "
                "Set them before GO, for example: debug on nav. "
                "Their reported state is included in the log.\n"
                "Stop Test only stops recording. It does not stop the robot.\n"
                "The existing firmware STOP is not yet a global "
                "collection pause."
            ),
        ).pack(anchor="w")

    def editable(self):
        return bool(
            self.ser
            and self.config
            and not self.locked
            and not self.config["locked"]
            and not self.config["started"]
        )

    def refresh_setup(self):
        for widget, normal in self.setup_widgets:
            state = (
                normal
                if self.editable() and not self.pending
                else "disabled"
            )
            widget.configure(state=state)

        if self.config:
            c = self.config

            self.confirmed_text.set(
                f"Firmware: {self.mode or 'mode unknown'} / {c['colour']}\n"
                f"Home ({c['home_x']:g}, {c['home_y']:g}) mm\n"
                f"Start ({c['x']:g}, {c['y']:g}) mm, {c['heading']:g}°\n"
                f"{'AUTO' if c['pickup'] else 'ROAM'}; "
                f"{'RUNNING' if c['started'] else 'PRE-RUN'}; "
                f"{'LOCKED' if self.locked else 'unlocked'}"
            )

        if (
            self.config
            and not self.locked
            and self.setup_status.get().startswith(("Connect", "Read-only"))
        ):
            self.setup_status.set(
                "Ready: edit draft, then Apply to Teensy."
            )

        if self.locked:
            self.setup_status.set(
                "Read-only: running, locked, disconnected "
                "or awaiting firmware."
            )

        self.redraw_map()

    def draft_changed(self):
        self.setup_status.set("Draft changed — not yet applied.")
        self.recorder.add("GUI", "Setup draft edited")
        self.redraw_map()

    def use_centre(self):
        if not self.editable():
            return

        corner = (
            self.blue_var.get()
            if self.base_var.get() == "blue"
            else self.green_var.get()
        )

        x, y = CORNERS[corner]
        self.x_var.set(str(x))
        self.y_var.set(str(y))
        self.draft_changed()

    def copy_config_to_draft(self):
        if self.config:
            c = self.config

            self.base_var.set(c["colour"])
            self.x_var.set(f"{c['x']:g}")
            self.y_var.set(f"{c['y']:g}")
            self.heading_var.set(f"{c['heading']:g}")
            self.pickup_var.set(c["pickup"])

            for name, xy in CORNERS.items():
                if xy == (c["home_x"], c["home_y"]):
                    var = (
                        self.blue_var
                        if c["colour"] == "blue"
                        else self.green_var
                    )
                    var.set(name)

            self.redraw_map()

    def load_config(self):
        self.load_next = True
        self.send_command("config")

    def apply_setup(self):
        if not self.editable() or self.pending:
            return

        try:
            x, y, h = (
                float(v.get())
                for v in (self.x_var, self.y_var, self.heading_var)
            )

            if (
                not all(math.isfinite(v) for v in (x, y, h))
                or not (0 <= x <= 4900 and 0 <= y <= 2400)
            ):
                raise ValueError(
                    "Position must be inside the arena and heading finite."
                )

            if self.blue_var.get() == self.green_var.get():
                raise ValueError(
                    "Choose different corners for the two base drawings."
                )

        except ValueError as exc:
            messagebox.showerror("Setup", str(exc))
            return

        colour = self.base_var.get()
        corner = (
            self.blue_var.get()
            if colour == "blue"
            else self.green_var.get()
        )

        hx, hy = CORNERS[corner]

        self.pending = dict(
            colour=colour,
            home_x=hx,
            home_y=hy,
            x=x,
            y=y,
            heading=h % 360,
            pickup=self.pickup_var.get(),
        )

        # Match the precision sent to firmware and its two-decimal reply.
        for key in ("x", "y", "heading"):
            self.pending[key] = round(self.pending[key], 2)

        self.pending["heading"] %= 360
        p = self.pending

        commands = [
            f"base {colour}",
            f"home {corner}",
            f"startpose {p['x']:.2f} {p['y']:.2f} {p['heading']:.2f}",
            "auto" if p["pickup"] else "roam",
            "config",
        ]

        for command in commands:
            if not self.send_command(command):
                self.pending = None
                break

        self.setup_status.set("Waiting for firmware readback…")
        self.refresh_setup()

        token = self.pending

        def timeout():
            if token is not None and self.pending is token:
                self.pending = None
                self.refresh_setup()
                self.setup_status.set(
                    "No matching readback. Read configuration before retrying."
                )
                self.append_log("[GUI] Setup confirmation timed out")

        self.root.after(6000, timeout)

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            return

        try:
            connection = serial.Serial(
                port,
                BAUD_RATE,
                timeout=0.1,
                write_timeout=0.5,
            )
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("Serial", str(exc))
            return

        self.drain_rx(limit=None)
        self.generation += 1
        generation = self.generation

        self.ser = connection
        self.reader_stop = threading.Event()

        self.config = None
        self.mode = None
        self.locked = True
        self.pending = None
        self.lock_latched = False
        self.run_latched = False
        self.load_next = True

        self.debug_state = {}
        self.gains = {}
        self.last_telemetry_time = None
        self.path_points.clear()

        self.connection_var.set(f"CONNECTED: {port}")
        self.connect_btn.configure(text="Disconnect")

        self.reader = threading.Thread(
            target=self.read_connection,
            args=(connection, self.reader_stop, generation),
            daemon=True,
        )
        self.reader.start()

        self.append_log(f"[GUI] Connected to {port}")
        self.refresh_setup()

        for command in ("mode", "config", "debug status", "gains"):
            self.send_command(command)

    def read_connection(self, connection, stop, generation):
        pending = bytearray()

        while not stop.is_set():
            try:
                raw = connection.readline()

                if not raw:
                    continue

                pending.extend(raw)

                # readline() can return partial lines on timeout.
                while b"\n" in pending:
                    line, _, remainder = pending.partition(b"\n")
                    pending = bytearray(remainder)

                    text = line.rstrip(b"\r").decode(
                        "utf-8", errors="replace"
                    )

                    self.rx_queue.put(
                        (
                            generation,
                            time.monotonic_ns(),
                            "LINE",
                            text,
                        )
                    )

            except (serial.SerialException, OSError) as exc:
                if not stop.is_set():
                    self.rx_queue.put(
                        (
                            generation,
                            time.monotonic_ns(),
                            "ERROR",
                            str(exc),
                        )
                    )
                break

    def disconnect(self):
        if self.ser:
            self.append_log("[GUI] Disconnected")

        self.reader_stop.set()

        if self.ser:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass

        if self.reader:
            self.reader.join(timeout=0.3)

        self.ser = None
        self.config = None
        self.mode = None
        self.locked = True
        self.pending = None
        self.lock_latched = False
        self.run_latched = False
        self.load_next = True

        self.connection_var.set("DISCONNECTED")
        self.connect_btn.configure(text="Connect")
        self.confirmed_text.set("Firmware configuration: unknown")
        self.refresh_setup()

    def send_command(self, command):
        command = command.strip()

        if not command:
            return False

        if not self.ser or not self.ser.is_open:
            self.append_log(f"[GUI] Not connected: {command}")
            return False

        if not command_allowed(
            command, self.config, self.mode, self.locked
        ):
            self.append_log(
                f"[GUI] Blocked by run/configuration lock: {command}"
            )
            return False

        try:
            data = (command + "\n").encode("utf-8")

            if self.ser.write(data) != len(data):
                raise serial.SerialException(
                    "Incomplete serial write; disconnect and reconnect."
                )

            self.recorder.add("TX", command)
            if command != "config":
                super().append_log("> " + command)
            return True

        except (serial.SerialException, OSError) as exc:
            self.append_log(f"[GUI] Write error: {exc}")
            self.disconnect()
            return False

    def process_rx_queue(self):
        self.drain_rx()
        self.root.after(25, self.process_rx_queue)

    def drain_rx(self, limit=500):
        count = self.rx_queue.qsize() if limit is None else limit

        for _ in range(count):
            try:
                generation, ns, kind, text = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            if generation != self.generation:
                continue

            self.recorder.add(
                "RX" if kind == "LINE" else "GUI",
                text,
                ns,
            )

            if kind == "ERROR":
                self.append_log("[GUI] Serial error: " + text)
                self.disconnect()
            elif self.ser:
                self.handle_serial_line(text)

    def handle_serial_line(self, line):
        c = parse_config(line)

        if c:
            # A late pre-GO reply must never undo a RUN,STARTED latch.
            self.run_latched = self.run_latched or c["started"]
            self.lock_latched = (
                self.lock_latched or c["locked"] or self.run_latched
            )

            c["started"] = self.run_latched
            self.locked = self.lock_latched
            c["locked"] = self.locked
            self.config = c

            if self.load_next:
                self.load_next = False
                self.copy_config_to_draft()

            if self.pending:
                if c["locked"]:
                    self.pending = None
                    self.append_log(
                        "[GUI] Setup interrupted by configuration lock"
                    )
                elif all(
                    c[k] == v for k, v in self.pending.items()
                ):
                    self.pending = None
                    self.setup_status.set("Confirmed by Teensy.")
                    self.append_log(
                        "[GUI] Setup confirmed by firmware readback"
                    )

            self.refresh_setup()
            return

        elif line in ("MODE,BENCH", "MODE,COMPETITION"):
            self.mode = line.split(",")[1]
            self.refresh_setup()

        elif line == "RUN,WAITING":
            self.config = None
            self.locked = True
            self.pending = None
            self.lock_latched = False
            self.run_latched = False
            self.load_next = True

            self.debug_state = {}
            self.gains = {}
            self.path_points.clear()

            self.append_log(
                "[GUI] Firmware waiting/reset; awaiting fresh configuration"
            )
            self.refresh_setup()
            self.send_command("config")

        elif line in ("RUN,STARTED", "CONFIG,LOCKED"):
            self.locked = True
            self.lock_latched = True

            if line == "RUN,STARTED":
                self.run_latched = True

            if self.config:
                self.config["locked"] = True

                if line == "RUN,STARTED":
                    self.config["started"] = True

            self.pending = None
            self.refresh_setup()
            self.send_command("config")

        elif line.startswith("ERR,config,"):
            self.pending = None
            self.refresh_setup()
            self.setup_status.set("Firmware rejected setup: " + line)

        elif line.startswith("DEBUG,"):
            p = line.split(",")

            if (
                len(p) == 3
                and p[1] in MODULES
                and p[2] in ("0", "1")
            ):
                self.debug_state[p[1]] = p[2] == "1"

        for prefix in ("Turn KP =", "Drive KP =", "Drive Power ="):
            if line.startswith(prefix):
                self.gains[prefix[:-2].strip()] = (
                    line.split("=", 1)[1].strip()
                )

        if self.recorder.active:
            self.recorder.metadata["latest_reported_config"] = (
                dict(self.config) if self.config else None
            )
            self.recorder.metadata["reported_debug_modules"] = (
                dict(self.debug_state)
            )
            self.recorder.metadata["reported_gains"] = dict(self.gains)

        super().handle_serial_line(line)

    def parse_telemetry(self, line):
        try:
            p = line.split(",")

            if len(p) != 15 or not all(
                math.isfinite(float(p[i]))
                for i in (1, 2, 3, *range(6, 15))
            ):
                raise ValueError("Invalid telemetry")

        except (ValueError, IndexError):
            self.append_log("[GUI] Invalid telemetry: " + line)
            return

        super().parse_telemetry(line)

    def poll_status(self):
        if self.ser:
            # Only query until initial state is known.
            if self.config is None:
                self.send_command("config")

            if self.mode is None:
                self.send_command("mode")

        self.root.after(1000, self.poll_status)

    def append_log(self, text):
        if text.startswith("[GUI]"):
            self.recorder.add("GUI", text)

        super().append_log(text)

        # Bound the visible widget; the recorder keeps the full session.
        if int(self.serial_log.index("end-1c").split(".")[0]) > 2500:
            self.serial_log.configure(state="normal")
            self.serial_log.delete("1.0", "501.0")
            self.serial_log.configure(state="disabled")

    def preserve_session(self):
        if self.recorder.start_ns is None or self.recorder.saved:
            return True

        answer = messagebox.askyesnocancel(
            "Recorded session",
            "Export the previous recording before continuing?",
        )

        if answer is None:
            return False

        return self.export_test() if answer else True

    def start_test(self):
        if self.recorder.active:
            return

        if not self.ser:
            messagebox.showerror(
                "Test", "Connect to a serial port first."
            )
            return

        self.drain_rx()

        if not self.preserve_session():
            return

        if not self.ser or not self.ser.is_open:
            messagebox.showerror(
                "Test", "Serial connection was lost. Reconnect first."
            )
            return

        self.recorder.start(
            dict(
                port=self.ser.port,
                baud=BAUD_RATE,
                firmware_mode=self.mode,
                initial_config=(
                    dict(self.config) if self.config else None
                ),
                initial_debug_modules=dict(self.debug_state),
                initial_gains=dict(self.gains),
                other_base_drawing=dict(
                    blue=self.blue_var.get(),
                    green=self.green_var.get(),
                ),
                debug_policy="Recorder does not change module enables",
            )
        )

        self.record_status.set("● RECORDING")

        for command in ("config", "debug status", "gains"):
            self.send_command(command)

    def stop_test(self):
        if not self.recorder.active:
            return

        self.recorder.stop()
        self.drain_rx(limit=None)

        self.record_status.set(
            f"Stopped — {len(self.recorder.records)} records retained"
        )

    def fault(self):
        marker = self.recorder.fault()

        if marker:
            super().append_log(marker)
            self.send_command("mark fault")

    def export_test(self):
        if self.recorder.start_ns is None:
            messagebox.showinfo("Export", "No test recorded.")
            return False

        if self.recorder.active:
            messagebox.showinfo("Export", "Stop Test before exporting.")
            return False

        self.drain_rx(limit=None)

        name = filedialog.asksaveasfilename(
            defaultextension=".txt",
            initialfile=datetime.now().strftime(
                "robocup_test_%Y%m%d_%H%M%S.txt"
            ),
            filetypes=[("Text log", "*.txt")],
        )

        if not name:
            return False

        try:
            Path(name).write_text(
                self.recorder.export_text(),
                encoding="utf-8",
            )
        except OSError as exc:
            messagebox.showerror("Export failed", str(exc))
            return False

        self.recorder.saved = True
        self.record_status.set("Exported — session remains in memory")
        return True

    def transform(self):
        w = self.map_canvas.winfo_width()
        h = self.map_canvas.winfo_height()

        scale = max(
            0.001,
            min((w - 60) / 4900, (h - 70) / 2400),
        )

        return (
            (w - 4900 * scale) / 2,
            (h + 2400 * scale) / 2,
            scale,
        )

    def map_click(self, event):
        if not self.editable() or self.pending:
            return

        ox, oy, scale = self.transform()
        x = (event.x - ox) / scale
        y = (oy - event.y) / scale

        if 0 <= x <= 4900 and 0 <= y <= 2400:
            self.x_var.set(str(round(x)))
            self.y_var.set(str(round(y)))
            self.draft_changed()

    def redraw_map(self):
        if not hasattr(self, "blue_var"):
            return

        canvas = self.map_canvas
        canvas.delete("all")
        ox, oy, scale = self.transform()

        def xy(x, y):
            return ox + x * scale, oy - y * scale

        canvas.create_rectangle(
            *xy(0, 2400),
            *xy(4900, 0),
            fill="#f5f5f5",
            outline="#333",
        )

        for colour, var in [
            ("blue", self.blue_var),
            ("green", self.green_var),
        ]:
            x, y = CORNERS[var.get()]

            canvas.create_rectangle(
                *xy(x - 300, y + 300),
                *xy(x + 300, y - 300),
                fill=colour,
                stipple="gray50",
                outline=colour,
            )

            canvas.create_text(
                *xy(x, y),
                text=colour.upper(),
                fill="black",
            )

        canvas.create_text(
            *xy(2450, -160),
            text=(
                "X →   |   Y ↑   |   orange: draft start; "
                "black cross: confirmed home"
            ),
        )

        if self.config:
            px, py = xy(
                self.config["home_x"],
                self.config["home_y"],
            )
            canvas.create_line(px - 8, py, px + 8, py, width=3)
            canvas.create_line(px, py - 8, px, py + 8, width=3)

        if len(self.path_points) > 1:
            points = [
                v
                for point in self.path_points
                for v in xy(*point)
            ]
            canvas.create_line(*points, fill="#888")

        def arrow(x, y, heading, colour):
            px, py = xy(x, y)
            a = math.radians(heading)

            canvas.create_oval(
                px - 4, py - 4, px + 4, py + 4,
                fill=colour,
            )
            canvas.create_line(
                px,
                py,
                px + 22 * math.cos(a),
                py - 22 * math.sin(a),
                fill=colour,
                width=3,
                arrow=tk.LAST,
            )

        try:
            values = [
                float(v.get())
                for v in (self.x_var, self.y_var, self.heading_var)
            ]

            if all(math.isfinite(v) for v in values):
                arrow(*values, "#dd7900")

        except ValueError:
            pass

        if self.last_telemetry_time is not None:
            arrow(
                self.robot_x,
                self.robot_y,
                self.robot_heading,
                "#111",
            )

    def clear_log(self):
        self.recorder.add(
            "GUI",
            "Visible serial log cleared; recording retained",
        )
        super().clear_log()

    def clear_flag_history(self):
        self.recorder.add("GUI", "Visible flag history cleared")
        super().clear_flag_history()

    def on_close(self):
        self.stop_test()

        if not self.preserve_session():
            return

        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    TestGUI(root)
    root.mainloop()