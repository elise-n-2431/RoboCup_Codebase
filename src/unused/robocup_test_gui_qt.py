import math
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal, QPointF
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from session_tools import (
    CORNERS,
    MODULES,
    SessionRecorder,
    command_allowed,
    parse_config,
)


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

NAV_STATES = {
    "0": "STATIONARY",
    "1": "ROAMING",
    "2": "PURSUIT",
    "3": "SORTING",
    "4": "COLLECTING",
    "5": "HOMING",
    "6": "OPENING",
    "7": "CLOSING",
    "8": "REVERSING",
}

COLLECT_STATES = {
    "0": "IDLE",
    "1": "LOWERING_VERT",
    "2": "VERT_REACHED",
    "3": "LOWERING_HORI",
    "4": "HORI_REACHED",
    "5": "PICKING_UP",
    "6": "RETURNING",
    "7": "DECIDING",
}

TOF_NAMES = [
    "Nav outer L",
    "Nav inner L",
    "Nav inner R",
    "Nav outer R",
    "Weight L top",
    "Weight L bottom",
    "Weight R top",
    "Weight R bottom",
    "Weight middle",
]

INTERNAL_COMMANDS = {"config", "mode", "debug status"}


class SerialReader(QThread):
    line_received = pyqtSignal(str, object)
    serial_error = pyqtSignal(str)

    def __init__(self, connection):
        super().__init__()
        self.connection = connection
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def run(self):
        pending = bytearray()

        while not self.stop_event.is_set():
            try:
                raw = self.connection.readline()
            except (serial.SerialException, OSError) as exc:
                if not self.stop_event.is_set():
                    self.serial_error.emit(str(exc))
                return

            if not raw:
                continue

            pending.extend(raw)

            while b"\n" in pending:
                line, _, remainder = pending.partition(b"\n")
                pending = bytearray(remainder)
                text = line.rstrip(b"\r").decode("utf-8", errors="replace")
                self.line_received.emit(text, time.monotonic_ns())


class ArenaWidget(QWidget):
    start_clicked = pyqtSignal(float, float)
    weight_clicked = pyqtSignal(float, float)
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(360)
        self.setMouseTracking(True)

        self.editable = False
        self.base_colour = "blue"
        self.blue_corner = "sw"
        self.green_corner = "ne"
        self.start_x = 300.0
        self.start_y = 300.0
        self.start_heading = 0.0

        self.confirmed_home = None
        self.robot = None
        self.path_points = []
        self.place_weight_mode = False
        self.planned_weights = []

    def set_draft(self, colour, blue_corner, green_corner, x, y, heading):
        self.base_colour = colour
        self.blue_corner = blue_corner
        self.green_corner = green_corner
        self.start_x = x
        self.start_y = y
        self.start_heading = heading
        self.update()
        
    def set_planned_weights(self, weights):
        self.planned_weights = list(weights)
        self.update()


    def begin_weight_placement(self):
        self.place_weight_mode = True
        self.update()

    def set_confirmed_home(self, x, y):
        self.confirmed_home = (x, y)
        self.update()

    def set_robot(self, x, y, heading):
        self.robot = (x, y, heading)
        self.path_points.append((x, y))
        if len(self.path_points) > 500:
            self.path_points.pop(0)
        self.update()

    def clear_robot(self):
        self.robot = None
        self.path_points.clear()
        self.update()

    def _transform(self):
        margin = 28.0
        w = max(1.0, self.width() - 2 * margin)
        h = max(1.0, self.height() - 2 * margin)
        scale = min(w / 4900.0, h / 2400.0)
        ox = (self.width() - 4900.0 * scale) / 2.0
        oy = (self.height() + 2400.0 * scale) / 2.0
        return ox, oy, scale

    def _xy(self, x, y):
        ox, oy, scale = self._transform()
        return QPointF(ox + x * scale, oy - y * scale)

    def mousePressEvent(self, event):
        if not self.editable or event.button() != Qt.MouseButton.LeftButton:
            return

        ox, oy, scale = self._transform()
        x = (event.position().x() - ox) / scale
        y = (oy - event.position().y()) / scale

        if 0 <= x <= 4900 and 0 <= y <= 2400:
            if self.place_weight_mode:
                self.place_weight_mode = False
                self.weight_clicked.emit(x, y)
            else:
                self.start_clicked.emit(x, y)

    def _draw_arrow(self, painter, x, y, heading, colour, radius=16):
        p = self._xy(x, y)
        a = math.radians(heading)

        nose = QPointF(
            p.x() + radius * math.cos(a),
            p.y() - radius * math.sin(a),
        )
        left = QPointF(
            p.x() + 0.75 * radius * math.cos(a + 2.5),
            p.y() - 0.75 * radius * math.sin(a + 2.5),
        )
        right = QPointF(
            p.x() + 0.75 * radius * math.cos(a - 2.5),
            p.y() - 0.75 * radius * math.sin(a - 2.5),
        )

        painter.setPen(QPen(colour, 2))
        painter.setBrush(QBrush(colour))
        painter.drawPolygon(QPolygonF([nose, left, right]))
        painter.drawEllipse(p, 3, 3)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f7f8fa"))

        ox, oy, scale = self._transform()
        arena_left = ox
        arena_top = oy - 2400 * scale
        arena_width = 4900 * scale
        arena_height = 2400 * scale

        painter.setPen(QPen(QColor("#dfe3e8"), 1))
        for x in range(500, 4900, 500):
            p1 = self._xy(x, 0)
            p2 = self._xy(x, 2400)
            painter.drawLine(p1, p2)
        for y in range(500, 2400, 500):
            p1 = self._xy(0, y)
            p2 = self._xy(4900, y)
            painter.drawLine(p1, p2)

        painter.setPen(QPen(QColor("#24292f"), 2))
        painter.setBrush(QBrush(QColor("#ffffff")))
        painter.drawRect(int(arena_left), int(arena_top), int(arena_width), int(arena_height))

        for colour_name, corner_name in (
            ("blue", self.blue_corner),
            ("green", self.green_corner),
        ):
            cx, cy = CORNERS[corner_name]
            top_left = self._xy(cx - 300, cy + 300)
            bottom_right = self._xy(cx + 300, cy - 300)
            fill = QColor("#3787ff" if colour_name == "blue" else "#37b56b")
            fill.setAlpha(95)
            painter.setBrush(QBrush(fill))
            painter.setPen(QPen(QColor("#2767c8" if colour_name == "blue" else "#24884d"), 2))
            painter.drawRect(
                int(top_left.x()),
                int(top_left.y()),
                int(bottom_right.x() - top_left.x()),
                int(bottom_right.y() - top_left.y()),
            )

            centre = self._xy(cx, cy)
            painter.setPen(QPen(QColor("#111827"), 1))
            painter.drawText(
                int(centre.x() - 25),
                int(centre.y() + 5),
                colour_name.upper(),
            )

        if self.confirmed_home:
            p = self._xy(*self.confirmed_home)
            painter.setPen(QPen(QColor("#111827"), 3))
            painter.drawLine(QPointF(p.x() - 9, p.y()), QPointF(p.x() + 9, p.y()))
            painter.drawLine(QPointF(p.x(), p.y() - 9), QPointF(p.x(), p.y() + 9))

        if len(self.path_points) > 1:
            painter.setPen(QPen(QColor("#8c959f"), 2))
            last = self._xy(*self.path_points[0])
            for x, y in self.path_points[1:]:
                current = self._xy(x, y)
                painter.drawLine(last, current)
                last = current

        # ----------------------------------------------------------
        # Planned / known starting weight positions
        # ----------------------------------------------------------

        for i, (x, y) in enumerate(
            self.planned_weights
        ):
            point = self._xy(x, y)

            painter.setPen(
                QPen(
                    QColor("#7c3aed"),
                    2
                )
            )

            painter.setBrush(
                QBrush(
                    QColor("#c4b5fd")
                )
            )

            painter.drawEllipse(
                point,
                8,
                8
            )

            painter.setPen(
                QPen(
                    QColor("#5b21b6"),
                    1
                )
            )

            painter.drawText(
                int(point.x() + 11),
                int(point.y() - 8),
                f"W{i + 1}"
            )
        self._draw_arrow(
            painter,
            self.start_x,
            self.start_y,
            self.start_heading,
            QColor("#dd7900"),
        )

        if self.robot:
            self._draw_arrow(painter, *self.robot, QColor("#111827"), radius=18)

        painter.setPen(QPen(QColor("#57606a"), 1))
        painter.drawText(
            int(arena_left),
            int(arena_top + arena_height + 20),
            "X →   Y ↑    orange = draft start    black cross = confirmed home",
        )


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.ser = None
        self.reader = None
        self.recorder = SessionRecorder()

        self.config = None
        self.mode = None
        self.locked = True
        self.lock_latched = False
        self.run_latched = False
        self.load_next = False
        self.pending = None

        self.debug_state = {name: False for name in MODULES}
        self.gains = {}
        self.flag_counts = {}
        self.last_telemetry_time = None

        self.map_block_end = None
        self.skip_map_value_lines = 0

        self.setWindowTitle("RoboCup Control & Test")
        self.resize(1500, 900)
        self.setMinimumSize(1150, 720)
        self.planned_weights = []
        self._build_ui()
        self._apply_style()
        self.refresh_ports()
        self.refresh_setup()
        self.refresh_debug_buttons()
        self.refresh_status_chips()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.poll_status)
        self.status_timer.start(1000)

        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.update_telemetry_age)
        self.telemetry_timer.start(250)
        
        

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(10)

        outer.addWidget(self._build_top_bar())

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.arena = ArenaWidget()

        self.arena.start_clicked.connect(
            self.on_arena_click
        )

        self.arena.weight_clicked.connect(
            self.on_weight_click
        )
        left_layout.addWidget(self.arena, 3)
        left_layout.addWidget(self._build_robot_summary(), 0)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_setup_tab(), "Pre-run setup")
        self.tabs.addTab(self._build_test_tab(), "Unit / Debug Test")
        self.tabs.addTab(self._build_controls_tab(), "Existing controls")

        splitter.addWidget(left)
        splitter.addWidget(self.tabs)
        splitter.setSizes([900, 560])
        outer.addWidget(splitter, 1)
        outer.addWidget(self._build_serial_log(), 0)

    def _build_top_bar(self):
        frame = QFrame()
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(QLabel("Port"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        layout.addWidget(self.port_combo)

        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)

        layout.addSpacing(12)
        self.connection_chip = QLabel("DISCONNECTED")
        self.mode_chip = QLabel("MODE --")
        self.run_chip = QLabel("NO CONFIG")
        for chip in (self.connection_chip, self.mode_chip, self.run_chip):
            chip.setProperty("chip", True)
            layout.addWidget(chip)

        layout.addStretch(1)

        self.telemetry_label = QLabel("Telemetry: --")
        layout.addWidget(self.telemetry_label)

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("stopButton")
        self.stop_btn.clicked.connect(lambda: self.send_command("stop"))
        layout.addWidget(self.stop_btn)

        return frame

    def _build_robot_summary(self):
        group = QGroupBox("Robot")
        layout = QGridLayout(group)

        self.pose_label = QLabel("x=--   y=--   heading=--")
        self.nav_label = QLabel("Nav: --")
        self.collect_label = QLabel("Collect: --")

        layout.addWidget(self.pose_label, 0, 0, 1, 2)
        layout.addWidget(self.nav_label, 1, 0)
        layout.addWidget(self.collect_label, 1, 1)
        return group

    def _build_setup_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        intro = QLabel(
            "Configure the robot before physical GO. The arena here is only for "
            "home/start setup; unfinished mapping is intentionally kept separate."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        form_box = QGroupBox("Arena configuration")
        form = QFormLayout(form_box)

        self.base_combo = QComboBox()
        self.base_combo.addItems(["blue", "green"])
        self.blue_corner_combo = self._corner_combo("sw")
        self.green_corner_combo = self._corner_combo("ne")

        self.x_spin = self._double_spin(0, 4900, 300, 0)
        self.y_spin = self._double_spin(0, 2400, 300, 0)
        self.heading_spin = self._double_spin(0, 359.99, 0, 2)
        self.pickup_check = QCheckBox("Collect weights (AUTO)")
        self.pickup_check.setChecked(True)

        form.addRow("Our base colour", self.base_combo)
        form.addRow("Blue corner", self.blue_corner_combo)
        form.addRow("Green corner", self.green_corner_combo)
        form.addRow("Start X (mm)", self.x_spin)
        form.addRow("Start Y (mm)", self.y_spin)
        form.addRow("Heading (°)", self.heading_spin)
        form.addRow("Run mode", self.pickup_check)
        layout.addWidget(form_box)
        targets_box = QGroupBox("Priority weight positions")

        targets_layout = QVBoxLayout(
            targets_box
        )


        self.target_summary = QLabel(
            "No priority weights placed."
        )

        self.target_summary.setWordWrap(
            True
        )

        targets_layout.addWidget(
            self.target_summary
        )


        target_buttons = QHBoxLayout()


        self.add_target_btn = QPushButton(
            "+ Weight"
        )

        self.add_target_btn.clicked.connect(
            self.begin_weight_placement
        )


        self.remove_target_btn = QPushButton(
            "Remove last"
        )

        self.remove_target_btn.clicked.connect(
            self.remove_last_weight
        )


        self.clear_targets_btn = QPushButton(
            "Clear weights"
        )

        self.clear_targets_btn.clicked.connect(
            self.clear_weights
        )


        target_buttons.addWidget(
            self.add_target_btn
        )

        target_buttons.addWidget(
            self.remove_target_btn
        )

        target_buttons.addWidget(
            self.clear_targets_btn
        )


        targets_layout.addLayout(
            target_buttons
        )

        layout.addWidget(
            targets_box
        )
        

        self.setup_widgets = [
            self.base_combo,
            self.blue_corner_combo,
            self.green_corner_combo,
            self.x_spin,
            self.y_spin,
            self.heading_spin,
            self.pickup_check,
        ]

        for widget in self.setup_widgets:
            if isinstance(widget, QComboBox):
                widget.currentIndexChanged.connect(self.draft_changed)
            elif isinstance(widget, QCheckBox):
                widget.toggled.connect(self.draft_changed)
            else:
                widget.valueChanged.connect(self.draft_changed)

        use_centre = QPushButton("Use our base centre as start")
        use_centre.clicked.connect(self.use_base_centre)
        self.apply_btn = QPushButton("Apply to Teensy")
        self.apply_btn.setObjectName("primaryButton")
        self.apply_btn.clicked.connect(self.apply_setup)
        self.lock_btn = QPushButton("Lock configuration")
        self.lock_btn.clicked.connect(lambda: self.send_command("config lock"))
        read_btn = QPushButton("Read / load firmware configuration")
        read_btn.clicked.connect(self.load_config)

        self.setup_action_widgets = [
            use_centre,
            self.apply_btn,
            self.lock_btn,
            self.add_target_btn,
            self.remove_target_btn,
            self.clear_targets_btn,
        ]
        for button in self.setup_action_widgets:
            layout.addWidget(button)
        layout.addWidget(read_btn)

        self.setup_status = QLabel("Connect and read firmware configuration.")
        self.setup_status.setWordWrap(True)
        self.setup_status.setObjectName("mutedLabel")
        layout.addWidget(self.setup_status)

        self.confirmed_label = QLabel("Firmware configuration: unknown")
        self.confirmed_label.setWordWrap(True)
        self.confirmed_label.setObjectName("configCard")
        layout.addWidget(self.confirmed_label)
        layout.addStretch(1)
        return tab

    def _build_test_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.record_status = QLabel("Not recording")
        self.record_status.setObjectName("recordStatus")
        layout.addWidget(self.record_status)

        row = QHBoxLayout()
        start = QPushButton("Start Test")
        start.clicked.connect(self.start_test)
        stop = QPushButton("Stop Test")
        stop.clicked.connect(self.stop_test)
        export = QPushButton("Export Test (.txt)")
        export.clicked.connect(self.export_test)
        row.addWidget(start)
        row.addWidget(stop)
        row.addWidget(export)
        layout.addLayout(row)

        fault = QPushButton("FAULT")
        fault.setObjectName("faultButton")
        fault.setMinimumHeight(85)
        fault.clicked.connect(self.fault)
        layout.addWidget(fault)

        debug_box = QGroupBox("Debug streams")
        debug_layout = QGridLayout(debug_box)
        self.debug_buttons = {}

        for i, module in enumerate(MODULES):
            button = QPushButton()
            button.setCheckable(True)
            button.clicked.connect(
                lambda checked, name=module: self.set_debug_module(name, checked)
            )
            self.debug_buttons[module] = button
            debug_layout.addWidget(button, i // 2, i % 2)

        controls = QHBoxLayout()
        all_on = QPushButton("All on")
        all_on.clicked.connect(lambda: self.set_debug_all(True))
        all_off = QPushButton("All off")
        all_off.clicked.connect(lambda: self.set_debug_all(False))
        refresh = QPushButton("Refresh status")
        refresh.clicked.connect(lambda: self.send_command("debug status", visible=False))
        controls.addWidget(all_on)
        controls.addWidget(all_off)
        controls.addWidget(refresh)
        debug_layout.addLayout(controls, (len(MODULES) + 1) // 2, 0, 1, 2)

        layout.addWidget(debug_box)

        note = QLabel(
            "Debug buttons use the firmware's debug on/off commands. In competition "
            "mode they become read-only once GO starts. FAULT only adds a test marker; "
            "it does not alter the robot state machine."
        )
        note.setWordWrap(True)
        note.setObjectName("mutedLabel")
        layout.addWidget(note)
        layout.addStretch(1)
        return tab

    def _build_controls_tab(self):
        content = QWidget()
        layout = QVBoxLayout(content)

        tuning = QGroupBox("Controller tuning")
        tune_form = QFormLayout(tuning)
        self.turn_kp_spin = self._double_spin(0, 100, 22, 2)
        self.drive_kp_spin = self._double_spin(0, 100, 10, 2)
        self.drive_power_spin = self._double_spin(0, 450, 280, 0)
        tune_form.addRow("Turn Kp", self.turn_kp_spin)
        tune_form.addRow("Drive Kp", self.drive_kp_spin)
        tune_form.addRow("Drive power", self.drive_power_spin)

        tune_buttons = QHBoxLayout()
        for text, command in (
            ("Apply Turn Kp", lambda: f"kp {self.turn_kp_spin.value():.2f}"),
            ("Apply Drive Kp", lambda: f"drivekp {self.drive_kp_spin.value():.2f}"),
            ("Apply Power", lambda: f"drivepower {int(self.drive_power_spin.value())}"),
        ):
            button = QPushButton(text)
            button.clicked.connect(lambda _, fn=command: self.send_command(fn()))
            tune_buttons.addWidget(button)
        tune_form.addRow(tune_buttons)

        read_gains = QPushButton("Read gains")
        read_gains.clicked.connect(lambda: self.send_command("gains"))
        tune_form.addRow(read_gains)
        layout.addWidget(tuning)

        actions = QGroupBox("Bench actions")
        action_grid = QGridLayout(actions)

        for i, (text, command) in enumerate(
            [
                ("AUTO", "auto"),
                ("ROAM", "roam"),
                ("Open gate", "open"),
                ("Close gate", "close"),
            ]
        ):
            button = QPushButton(text)
            button.clicked.connect(lambda _, cmd=command: self.send_command(cmd))
            action_grid.addWidget(button, i // 2, i % 2)

        self.turn_spin = self._double_spin(-360, 360, 90, 1)
        turn_btn = QPushButton("Turn")
        turn_btn.clicked.connect(lambda: self.send_command(f"turn {self.turn_spin.value():.1f}"))
        action_grid.addWidget(self.turn_spin, 2, 0)
        action_grid.addWidget(turn_btn, 2, 1)

        self.manual_drive_spin = self._double_spin(0, 450, 280, 0)
        drive_btn = QPushButton("Drive")
        drive_btn.clicked.connect(
            lambda: self.send_command(f"drive {int(self.manual_drive_spin.value())}")
        )
        action_grid.addWidget(self.manual_drive_spin, 3, 0)
        action_grid.addWidget(drive_btn, 3, 1)
        layout.addWidget(actions)

        flags_box = QGroupBox("Test flags")
        flags_layout = QGridLayout(flags_box)
        self.flag_labels = {}
        for row, (label, abbrev, full_name) in enumerate(FLAG_COMMANDS):
            button = QPushButton(label)
            button.clicked.connect(
                lambda _, a=abbrev: self.send_command(f"flag {a}")
            )
            seen = QLabel("--")
            self.flag_labels[full_name] = seen
            flags_layout.addWidget(button, row, 0)
            flags_layout.addWidget(seen, row, 1)
        clear_flags = QPushButton("Clear flag history")
        clear_flags.clicked.connect(self.clear_flag_history)
        flags_layout.addWidget(clear_flags, len(FLAG_COMMANDS), 0, 1, 2)
        layout.addWidget(flags_box)

        tof_box = QGroupBox("ToF readings (mm)")
        tof_grid = QGridLayout(tof_box)
        self.tof_labels = {}
        for i, name in enumerate(TOF_NAMES):
            value = QLabel("--")
            self.tof_labels[name] = value
            tof_grid.addWidget(QLabel(name), i // 3 * 2, i % 3)
            tof_grid.addWidget(value, i // 3 * 2 + 1, i % 3)
        layout.addWidget(tof_box)

        history_box = QGroupBox("Raised flag history")
        history_layout = QVBoxLayout(history_box)
        self.flag_history = QListWidget()
        history_layout.addWidget(self.flag_history)
        layout.addWidget(history_box)
        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        return scroll

    def _build_serial_log(self):
        group = QGroupBox("Serial / event log")
        layout = QVBoxLayout(group)
        self.serial_log = QTextEdit()
        self.serial_log.setReadOnly(True)
        self.serial_log.setMaximumHeight(220)
        layout.addWidget(self.serial_log)

        row = QHBoxLayout()
        self.command_edit = QLineEdit()
        self.command_edit.setPlaceholderText("Manual command")
        self.command_edit.returnPressed.connect(self.send_manual_command)
        send = QPushButton("Send")
        send.clicked.connect(self.send_manual_command)
        clear = QPushButton("Clear visible log")
        clear.clicked.connect(self.clear_log)
        row.addWidget(self.command_edit, 1)
        row.addWidget(send)
        row.addWidget(clear)
        layout.addLayout(row)
        return group

    def _corner_combo(self, default):
        combo = QComboBox()
        for name in ("sw", "se", "nw", "ne"):
            combo.addItem(name.upper(), name)
        combo.setCurrentIndex(combo.findData(default))
        return combo

    def _double_spin(self, minimum, maximum, value, decimals):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setValue(value)
        spin.setKeyboardTracking(False)
        return spin

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #f6f8fa; color: #1f2328; }
            QGroupBox {
                background: white;
                border: 1px solid #d0d7de;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 8px;
                font-weight: 600;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton {
                background: white;
                border: 1px solid #d0d7de;
                border-radius: 6px;
                padding: 7px 10px;
            }
            QPushButton:hover { background: #f3f4f6; }
            QPushButton:disabled { color: #8c959f; background: #f6f8fa; }
            QPushButton#primaryButton { background: #0969da; color: white; border: none; font-weight: 600; }
            QPushButton#faultButton { background: #cf222e; color: white; border: none; font-size: 22px; font-weight: 700; }
            QPushButton#stopButton { background: #cf222e; color: white; border: none; font-weight: 700; }
            QPushButton[debugOn="true"] { background: #1a7f37; color: white; border-color: #1a7f37; font-weight: 600; }
            QLabel[chip="true"] { background: #eaeef2; border-radius: 9px; padding: 5px 9px; font-weight: 600; }
            QLabel#mutedLabel { color: #57606a; }
            QLabel#configCard { background: white; border: 1px solid #d0d7de; border-radius: 6px; padding: 8px; }
            QLabel#recordStatus { font-size: 16px; font-weight: 700; }
            QLineEdit, QComboBox, QDoubleSpinBox, QTextEdit, QListWidget {
                background: white;
                border: 1px solid #d0d7de;
                border-radius: 5px;
                padding: 5px;
            }
            QTabWidget::pane { border: 1px solid #d0d7de; background: white; border-radius: 7px; }
            QTabBar::tab { padding: 8px 12px; }
            QTabBar::tab:selected { font-weight: 700; }
            """
        )

    # ------------------------------------------------------------ connection
    def refresh_ports(self):
        current = self.port_combo.currentText()
        ports = [p.device for p in list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "Serial", "Select a COM port first.")
            return

        try:
            connection = serial.Serial(
                port,
                BAUD_RATE,
                timeout=0.1,
                write_timeout=0.5,
            )
        except (serial.SerialException, OSError) as exc:
            QMessageBox.critical(self, "Serial connection failed", str(exc))
            return

        self.ser = connection
        self.reader = SerialReader(connection)
        self.reader.line_received.connect(self.on_serial_line)
        self.reader.serial_error.connect(self.on_serial_error)
        self.reader.start()

        self.config = None
        self.mode = None
        self.locked = True
        self.lock_latched = False
        self.run_latched = False
        self.load_next = True
        self.pending = None
        self.debug_state = {name: False for name in MODULES}
        self.gains = {}
        self.last_telemetry_time = None
        self.arena.clear_robot()

        self.connect_btn.setText("Disconnect")
        self.append_log(f"[GUI] Connected to {port}")
        self.refresh_setup()
        self.refresh_debug_buttons()
        self.refresh_status_chips()

        self.send_command("mode", visible=False)
        self.send_command("config", visible=False)
        self.send_command("debug status", visible=False)
        self.send_command("gains", visible=False)

    def disconnect(self):
        if self.ser:
            self.append_log("[GUI] Disconnected")

        if self.reader:
            self.reader.stop()

        if self.ser:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass

        if self.reader:
            self.reader.wait(400)

        self.reader = None
        self.ser = None
        self.config = None
        self.mode = None
        self.locked = True
        self.pending = None
        self.lock_latched = False
        self.run_latched = False
        self.load_next = True

        self.connect_btn.setText("Connect")
        self.confirmed_label.setText("Firmware configuration: unknown")
        self.refresh_setup()
        self.refresh_debug_buttons()
        self.refresh_status_chips()

    def on_serial_error(self, text):
        self.recorder.add("GUI", "Serial error: " + text)
        self.append_log("[GUI] Serial error: " + text)
        self.disconnect()

    def on_serial_line(self, line, ns):
        self.recorder.add("RX", line, ns)
        self.handle_serial_line(line)

    # -------------------------------------------------------------- protocol
    def send_command(self, command, visible=True):
        command = command.strip()
        if not command:
            return False

        if not self.ser or not self.ser.is_open:
            self.append_log(f"[GUI] Not connected: {command}")
            return False

        if not command_allowed(command, self.config, self.mode, self.locked):
            self.append_log(f"[GUI] Blocked by run/configuration lock: {command}")
            return False

        try:
            data = (command + "\n").encode("utf-8")
            if self.ser.write(data) != len(data):
                raise serial.SerialException("Incomplete serial write")

            self.recorder.add("TX", command)
            if visible and command not in INTERNAL_COMMANDS:
                self.append_log("> " + command)
            return True
        except (serial.SerialException, OSError) as exc:
            self.append_log("[GUI] Write error: " + str(exc))
            self.disconnect()
            return False

    def _strip_known_prefix_noise(self, line):
        # Temporary robustness for the old map timing-print bug. The firmware
        # should still be fixed so structured replies begin at column 0.
        for token in ("CONFIG,", "MODE,", "RUN,", "DEBUG,", "FLAG,"):
            idx = line.find(token)
            if idx > 0:
                return line[idx:]
        return line

    def _consume_map_line(self, line):
        if self.map_block_end:
            if line == self.map_block_end:
                self.map_block_end = None
            return True

        starts = {
            "OBSTACLE_MAP_START": "OBSTACLE_MAP_END",
            "FRONTIER_MAP_START": "FRONTIER_MAP_END",
            "WEIGHT_MAP_START": "WEIGHT_MAP_END",
        }
        if line in starts:
            self.map_block_end = starts[line]
            return True

        if self.skip_map_value_lines:
            self.skip_map_value_lines -= 1
            return True

        if line in ("Current position", "TOF readings", "Heading", "Target"):
            self.skip_map_value_lines = 1
            return True

        return False

    def handle_serial_line(self, raw_line):
        line = self._strip_known_prefix_noise(raw_line.strip())

        if self._consume_map_line(line):
            return

        config = parse_config(line)
        if config:
            self.run_latched = self.run_latched or config["started"]
            self.lock_latched = (
                self.lock_latched or config["locked"] or self.run_latched
            )
            config["started"] = self.run_latched
            self.locked = self.lock_latched
            config["locked"] = self.locked
            self.config = config

            if self.load_next:
                self.load_next = False
                self.copy_config_to_draft()

            if self.pending:
                if config["locked"]:
                    self.pending = None
                    self.setup_status.setText("Setup interrupted by configuration lock.")
                elif all(config[k] == v for k, v in self.pending.items()):
                    self.pending = None
                    self.setup_status.setText("Confirmed by Teensy.")
                    self.append_log("[GUI] Setup confirmed by firmware readback")

            self.refresh_setup()
            self.refresh_debug_buttons()
            self.refresh_status_chips()
            self.update_recorder_metadata()
            return

        if line in ("MODE,BENCH", "MODE,COMPETITION"):
            self.mode = line.split(",", 1)[1]
            self.refresh_debug_buttons()
            self.refresh_status_chips()
            self.update_recorder_metadata()
            return

        if line == "RUN,WAITING":
            self.config = None
            self.locked = True
            self.pending = None
            self.lock_latched = False
            self.run_latched = False
            self.load_next = True
            self.debug_state = {name: False for name in MODULES}
            self.arena.clear_robot()
            self.append_log(line)
            self.send_command("config", visible=False)
            self.refresh_setup()
            self.refresh_debug_buttons()
            self.refresh_status_chips()
            return

        if line in ("RUN,STARTED", "CONFIG,LOCKED"):
            self.locked = True
            self.lock_latched = True
            if line == "RUN,STARTED":
                self.run_latched = True
            if self.config:
                self.config["locked"] = True
                if line == "RUN,STARTED":
                    self.config["started"] = True
            self.pending = None
            self.append_log(line)
            self.send_command("config", visible=False)
            self.refresh_setup()
            self.refresh_debug_buttons()
            self.refresh_status_chips()
            return

        if line.startswith("ERR,config,"):
            self.pending = None
            self.setup_status.setText("Firmware rejected setup: " + line)
            self.append_log(line)
            self.refresh_setup()
            return

        if line.startswith("DEBUG,"):
            parts = line.split(",")
            if len(parts) == 3 and parts[1] in MODULES and parts[2] in ("0", "1"):
                self.debug_state[parts[1]] = parts[2] == "1"
                self.refresh_debug_buttons()
                self.update_recorder_metadata()
                return

        if line.startswith("ROBOT,"):
            self.parse_telemetry(line)
            return

        if line.startswith("FLAG,"):
            self.handle_flag_event(line)
            self.append_log(line)
            return

        if line.startswith("Turn KP ="):
            self._set_gain("Turn KP", line, self.turn_kp_spin)
        elif line.startswith("Drive KP ="):
            self._set_gain("Drive KP", line, self.drive_kp_spin)
        elif line.startswith("Drive Power ="):
            self._set_gain("Drive Power", line, self.drive_power_spin)

        self.append_log(line)
        self.update_recorder_metadata()

    def poll_status(self):
        if not self.ser or not self.ser.is_open or self.pending:
            return
        if self.config is None:
            self.send_command("config", visible=False)
        if self.mode is None:
            self.send_command("mode", visible=False)

    # ------------------------------------------------------------- config UI
    def editable(self):
        return bool(
            self.ser
            and self.config
            and not self.locked
            and not self.config["locked"]
            and not self.config["started"]
        )

    def refresh_setup(self):
        enabled = self.editable() and self.pending is None
        for widget in self.setup_widgets + self.setup_action_widgets:
            widget.setEnabled(enabled)

        self.arena.editable = enabled

        if self.config:
            c = self.config
            self.confirmed_label.setText(
                f"{self.mode or 'mode unknown'}  •  {c['colour'].upper()} base\n"
                f"Home ({c['home_x']:g}, {c['home_y']:g}) mm\n"
                f"Start ({c['x']:g}, {c['y']:g}) mm @ {c['heading']:g}°\n"
                f"{'AUTO' if c['pickup'] else 'ROAM'}  •  "
                f"{'RUNNING' if c['started'] else 'PRE-RUN'}  •  "
                f"{'LOCKED' if self.locked else 'UNLOCKED'}"
            )
            self.arena.set_confirmed_home(c["home_x"], c["home_y"])
        else:
            self.confirmed_label.setText("Firmware configuration: unknown")
            self.arena.confirmed_home = None

        if self.pending:
            self.setup_status.setText("Waiting for firmware readback…")
        elif enabled and self.setup_status.text().startswith(("Connect", "Read-only")):
            self.setup_status.setText("Ready: edit draft, then Apply to Teensy.")
        elif not enabled and not self.pending:
            self.setup_status.setText(
                "Read-only: disconnected, awaiting firmware, locked or running."
            )

        self.update_arena_draft()
        self.refresh_planned_weights()

        self.add_target_btn.setEnabled(
            enabled and
            len(self.planned_weights) < 12
        )

        self.remove_target_btn.setEnabled(
            enabled and
            bool(self.planned_weights)
        )

        self.clear_targets_btn.setEnabled(
            enabled and
            bool(self.planned_weights)
        )

    def draft_changed(self, *args):
        if self.editable():
            self.setup_status.setText("Draft changed — not yet applied.")
            self.recorder.add("GUI", "Setup draft edited")
        self.update_arena_draft()

    def update_arena_draft(self):
        self.arena.set_draft(
            self.base_combo.currentText(),
            self.blue_corner_combo.currentData(),
            self.green_corner_combo.currentData(),
            self.x_spin.value(),
            self.y_spin.value(),
            self.heading_spin.value(),
        )
    def begin_weight_placement(self):
        if not self.editable():
            return

        if len(self.planned_weights) >= 12:
            QMessageBox.warning(
                self,
                "Priority weights",
                "Maximum of 12 priority weights."
            )
            return

        self.arena.begin_weight_placement()

        self.setup_status.setText(
            "Click the arena to place a priority weight."
        )


    def on_weight_click(self, x, y):
        if not self.editable():
            return

        if len(self.planned_weights) >= 12:
            return

        self.planned_weights.append(
            (
                round(x),
                round(y)
            )
        )

        self.refresh_planned_weights()

        self.setup_status.setText(
            "Priority weight added — Apply to Teensy when ready."
        )


    def remove_last_weight(self):
        if not self.editable():
            return

        if self.planned_weights:
            self.planned_weights.pop()

        self.refresh_planned_weights()


    def clear_weights(self):
        if not self.editable():
            return

        self.planned_weights.clear()

        self.refresh_planned_weights()


    def refresh_planned_weights(self):
        self.arena.set_planned_weights(
            self.planned_weights
        )

        if not self.planned_weights:
            self.target_summary.setText(
                "No priority weights placed."
            )

            return


        lines = []

        for i, (x, y) in enumerate(
            self.planned_weights
        ):
            lines.append(
                f"W{i + 1}: ({x:.0f}, {y:.0f}) mm"
            )


        self.target_summary.setText(
            "   ".join(lines)
        )

    def on_arena_click(self, x, y):
        if not self.editable() or self.pending:
            return
        self.x_spin.setValue(round(x))
        self.y_spin.setValue(round(y))
        self.draft_changed()

    def use_base_centre(self):
        corner = (
            self.blue_corner_combo.currentData()
            if self.base_combo.currentText() == "blue"
            else self.green_corner_combo.currentData()
        )
        x, y = CORNERS[corner]
        self.x_spin.setValue(x)
        self.y_spin.setValue(y)
        self.draft_changed()

    def copy_config_to_draft(self):
        if not self.config:
            return

        c = self.config
        self.base_combo.setCurrentText(c["colour"])
        self.x_spin.setValue(c["x"])
        self.y_spin.setValue(c["y"])
        self.heading_spin.setValue(c["heading"])
        self.pickup_check.setChecked(c["pickup"])

        for name, xy in CORNERS.items():
            if xy == (c["home_x"], c["home_y"]):
                combo = (
                    self.blue_corner_combo
                    if c["colour"] == "blue"
                    else self.green_corner_combo
                )
                combo.setCurrentIndex(combo.findData(name))
                break

        self.update_arena_draft()

    def load_config(self):
        self.load_next = True
        self.send_command("config", visible=False)

    def apply_setup(self):
        if not self.editable() or self.pending:
            return

        if (
            self.blue_corner_combo.currentData()
            == self.green_corner_combo.currentData()
        ):
            QMessageBox.warning(
                self,
                "Setup",
                "Choose different corners for the blue and green base drawings.",
            )
            return


        colour = self.base_combo.currentText()

        corner = (
            self.blue_corner_combo.currentData()
            if colour == "blue"
            else self.green_corner_combo.currentData()
        )

        hx, hy = CORNERS[corner]


        # ----------------------------------------------------------
        # What we expect the Teensy configuration to report back.
        # ----------------------------------------------------------

        self.pending = {
            "colour": colour,
            "home_x": float(hx),
            "home_y": float(hy),
            "x": round(
                self.x_spin.value(),
                2
            ),
            "y": round(
                self.y_spin.value(),
                2
            ),
            "heading": round(
                self.heading_spin.value() % 360,
                2
            ),
            "pickup": self.pickup_check.isChecked(),
        }


        p = self.pending


        # ----------------------------------------------------------
        # Build command sequence.
        #
        # IMPORTANT:
        # Send the priority targets FIRST.
        #
        # This means that by the time normal CONFIG replies can
        # make the GUI say "Confirmed by Teensy", the target list
        # has already been sent.
        # ----------------------------------------------------------

        commands = [
            "targets clear",
        ]


        for x, y in self.planned_weights:
            commands.append(
                f"target add {x:.0f} {y:.0f}"
            )


        # Ask the Teensy to print the list back.
        #
        # We should visibly see:
        #
        # TARGETS,4
        # TARGET,0,...
        # TARGET,1,...
        # etc.
        commands.append(
            "targets"
        )


        # Normal robot configuration comes afterwards.
        commands.extend(
            [
                f"base {colour}",
                f"home {corner}",
                (
                    f"startpose "
                    f"{p['x']:.2f} "
                    f"{p['y']:.2f} "
                    f"{p['heading']:.2f}"
                ),
                "auto"
                if p["pickup"]
                else "roam",

                # Final readbacks.
                "targets",
                "config",
            ]
        )


        # ----------------------------------------------------------
        # Send the commands ONE AT A TIME.
        #
        # Previously they were all written almost instantly.
        # Give the Teensy/main loop time to consume and respond
        # to each command.
        # ----------------------------------------------------------

        token = self.pending


        def send_next(index=0):
            if index >= len(commands):
                return


            command = commands[index]


            if not self.send_command(
                command,
                visible=(command != "config")
            ):
                self.pending = None

                self.refresh_setup()

                self.setup_status.setText(
                    "Setup upload failed."
                )

                return


            QTimer.singleShot(
                150,
                lambda: send_next(
                    index + 1
                )
            )


        send_next()


        self.refresh_setup()


        # ----------------------------------------------------------
        # Existing readback timeout.
        # ----------------------------------------------------------

        def timeout():
            if (
                token is not None
                and self.pending is token
            ):
                self.pending = None

                self.refresh_setup()

                self.setup_status.setText(
                    "No matching readback. "
                    "Read configuration before retrying."
                )

                self.append_log(
                    "[GUI] Setup confirmation timed out"
                )


        # Give the paced upload a little more time than before.
        QTimer.singleShot(
            8000,
            timeout
        )

    # ------------------------------------------------------------- debug UI
    def debug_changes_allowed(self):
        if not self.ser or not self.config:
            return False
        return (not self.config["started"]) or self.mode == "BENCH"

    def refresh_debug_buttons(self):
        can_change = self.debug_changes_allowed()
        for module, button in self.debug_buttons.items():
            state = bool(self.debug_state.get(module, False))
            button.blockSignals(True)
            button.setChecked(state)
            button.setText(f"{module.upper()}   {'ON' if state else 'OFF'}")
            button.setProperty("debugOn", state)
            button.style().unpolish(button)
            button.style().polish(button)
            button.setEnabled(can_change)
            button.blockSignals(False)

    def set_debug_module(self, module, enabled):
        if not self.send_command(
            f"debug {'on' if enabled else 'off'} {module}",
            visible=True,
        ):
            self.refresh_debug_buttons()
            return
        QTimer.singleShot(120, lambda: self.send_command("debug status", visible=False))

    def set_debug_all(self, enabled):
        if self.send_command(
            f"debug {'on' if enabled else 'off'} all",
            visible=True,
        ):
            QTimer.singleShot(120, lambda: self.send_command("debug status", visible=False))

    # -------------------------------------------------------------- recorder
    def preserve_session(self):
        if self.recorder.start_ns is None or self.recorder.saved:
            return True

        answer = QMessageBox.question(
            self,
            "Recorded session",
            "Export the previous recording before continuing?",
            QMessageBox.StandardButton.Yes
            | QMessageBox.StandardButton.No
            | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Yes,
        )

        if answer == QMessageBox.StandardButton.Cancel:
            return False
        if answer == QMessageBox.StandardButton.Yes:
            return self.export_test()
        return True

    def start_test(self):
        if self.recorder.active:
            return
        if not self.ser or not self.ser.is_open:
            QMessageBox.warning(self, "Test", "Connect to a serial port first.")
            return
        if not self.preserve_session():
            return

        self.recorder.start(
            {
                "port": self.ser.port,
                "baud": BAUD_RATE,
                "firmware_mode": self.mode,
                "initial_config": dict(self.config) if self.config else None,
                "initial_debug_modules": dict(self.debug_state),
                "initial_gains": dict(self.gains),
                "other_base_drawing": {
                    "blue": self.blue_corner_combo.currentData(),
                    "green": self.green_corner_combo.currentData(),
                },
                "debug_policy": "GUI buttons control explicit firmware debug modules",
            }
        )
        self.record_status.setText("● RECORDING")
        self.record_status.setStyleSheet("color: #cf222e;")

        self.send_command("config", visible=False)
        self.send_command("targets", visible=False)
        self.send_command("debug status", visible=False)
        self.send_command("gains", visible=False)
    def stop_test(self):
        if not self.recorder.active:
            return
        self.recorder.stop()
        self.record_status.setText(
            f"Stopped — {len(self.recorder.records)} records retained"
        )
        self.record_status.setStyleSheet("")

    def fault(self):
        marker = self.recorder.fault()
        if marker:
            self.append_log(marker)
            self.send_command("mark fault", visible=False)

    def export_test(self):
        if self.recorder.start_ns is None:
            QMessageBox.information(self, "Export", "No test recorded.")
            return False
        if self.recorder.active:
            QMessageBox.information(self, "Export", "Stop Test before exporting.")
            return False

        default = datetime.now().strftime("robocup_test_%Y%m%d_%H%M%S.txt")
        name, _ = QFileDialog.getSaveFileName(
            self,
            "Export RoboCup test",
            default,
            "Text log (*.txt)",
        )
        if not name:
            return False

        try:
            Path(name).write_text(self.recorder.export_text(), encoding="utf-8")
        except OSError as exc:
            QMessageBox.critical(self, "Export failed", str(exc))
            return False

        self.recorder.saved = True
        self.record_status.setText("Exported — session remains in memory")
        return True

    def update_recorder_metadata(self):
        if not self.recorder.active:
            return
        self.recorder.metadata["latest_reported_config"] = (
            dict(self.config) if self.config else None
        )
        self.recorder.metadata["reported_debug_modules"] = dict(self.debug_state)
        self.recorder.metadata["reported_gains"] = dict(self.gains)

    # ------------------------------------------------------------- telemetry
    def parse_telemetry(self, line):
        parts = line.split(",")
        if len(parts) != 15:
            self.append_log("[GUI] Bad telemetry packet: " + line)
            return

        try:
            x = float(parts[1])
            y = float(parts[2])
            heading = float(parts[3])
            values = [int(float(v)) for v in parts[6:15]]
            if not all(math.isfinite(v) for v in (x, y, heading)):
                raise ValueError
        except ValueError:
            self.append_log("[GUI] Could not parse telemetry: " + line)
            return

        self.last_telemetry_time = time.monotonic()
        self.pose_label.setText(f"x={x:.0f} mm   y={y:.0f} mm   heading={heading:.1f}°")
        self.nav_label.setText("Nav: " + NAV_STATES.get(parts[4], parts[4]))
        self.collect_label.setText("Collect: " + COLLECT_STATES.get(parts[5], parts[5]))

        for name, value in zip(TOF_NAMES, values):
            self.tof_labels[name].setText(str(value))

        self.arena.set_robot(x, y, heading)

    def update_telemetry_age(self):
        if self.last_telemetry_time is None:
            self.telemetry_label.setText("Telemetry: --")
        else:
            age = time.monotonic() - self.last_telemetry_time
            self.telemetry_label.setText(f"Telemetry age: {age:.1f}s")

    def handle_flag_event(self, line):
        flag_name = line.split(",", 1)[1].strip()
        self.flag_counts[flag_name] = self.flag_counts.get(flag_name, 0) + 1
        count = self.flag_counts[flag_name]
        self.flag_history.addItem(
            f"{time.strftime('%H:%M:%S')}  {flag_name}  ({count})"
        )
        self.flag_history.scrollToBottom()
        if flag_name in self.flag_labels:
            self.flag_labels[flag_name].setText(f"Seen {count}")

    def clear_flag_history(self):
        self.flag_counts.clear()
        self.flag_history.clear()
        for label in self.flag_labels.values():
            label.setText("--")
        self.recorder.add("GUI", "Visible flag history cleared")

    def _set_gain(self, name, line, widget):
        try:
            value = float(line.split("=", 1)[1].strip())
            widget.setValue(value)
            self.gains[name] = f"{value:g}"
        except ValueError:
            return

    # -------------------------------------------------------------- misc UI
    def refresh_status_chips(self):
        connected = bool(self.ser and self.ser.is_open)
        self.connection_chip.setText("CONNECTED" if connected else "DISCONNECTED")
        self.mode_chip.setText(self.mode or "MODE --")

        if not self.config:
            self.run_chip.setText("NO CONFIG")
        elif self.config["started"]:
            self.run_chip.setText("RUNNING")
        elif self.locked:
            self.run_chip.setText("PRE-RUN • LOCKED")
        else:
            self.run_chip.setText("PRE-RUN • READY")

    def append_log(self, text):
        self.serial_log.append(text)
        if self.serial_log.document().blockCount() > 2500:
            cursor = self.serial_log.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            for _ in range(500):
                cursor.select(cursor.SelectionType.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()

    def clear_log(self):
        self.recorder.add("GUI", "Visible serial log cleared; recording retained")
        self.serial_log.clear()

    def send_manual_command(self):
        command = self.command_edit.text().strip()
        self.command_edit.clear()
        if command:
            self.send_command(command)

    def closeEvent(self, event):
        self.stop_test()
        if not self.preserve_session():
            event.ignore()
            return
        self.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
