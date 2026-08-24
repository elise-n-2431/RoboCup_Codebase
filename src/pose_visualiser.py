import serial
import math
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox, Button


# ============================================================
# SETTINGS
# ============================================================

PORT = "COM26"
BAUD = 115200


# ============================================================
# SERIAL
# ============================================================

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0
)


# ============================================================
# POSE DATA
# ============================================================

import time

last_map_update = 0.0
last_map_plot = 0.0



x_history = []
y_history = []

cell_confidence = {}


MIN_CONFIDENCE = -3
MAX_CONFIDENCE = 5

FREE_THRESHOLD = -2
OBSTACLE_THRESHOLD = 2

FREE_DECREMENT = 1
OBSTACLE_INCREMENT = 3

last_map_plot = 0.0
PLOT_MAP_PERIOD = 0.20



current_tof = {
    "outer_left": 0.0,
    "inner_left": 0.0,
    "inner_right": 0.0,
    "outer_right": 0.0,
}

current_weight_tof = {
    "left_top": 0.0,
    "left_bottom": 0.0,
    "right_top": 0.0,
    "right_bottom": 0.0,
}


current_x = 0.0
current_y = 0.0
current_heading = 0.0

CELL_SIZE_MM = 50.0


VIEW_WIDTH_MM = 6000
VIEW_HEIGHT_MM = 4000

VIEW_HALF_WIDTH = VIEW_WIDTH_MM / 2
VIEW_HALF_HEIGHT = VIEW_HEIGHT_MM / 2



# ============================================================
# PLOT SETUP
# ============================================================

#convertingthe sensor readisn to icr based on CAD
SENSOR_CIRCLE_X = 125.0
SENSOR_RADIUS = 90.0


SENSOR_ANGLES = {
    "outer_left": 40.0,
    "inner_left": 15.0,
    "inner_right": -15.0,
    "outer_right": -40.0,
}


fig, ax = plt.subplots()

tof_lines = {
    "outer_left": ax.plot([], [])[0],
    "inner_left": ax.plot([], [])[0],
    "inner_right": ax.plot([], [])[0],
    "outer_right": ax.plot([], [])[0],
}


tof_points = {
    "outer_left": ax.plot([], [], marker="o")[0],
    "inner_left": ax.plot([], [], marker="o")[0],
    "inner_right": ax.plot([], [], marker="o")[0],
    "outer_right": ax.plot([], [], marker="o")[0],
}

# Leave room at bottom for serial command controls
fig.subplots_adjust(
    bottom=0.28
)


path_line, = ax.plot(
    [],
    [],
    marker="o",
    markersize=2
)


heading_line, = ax.plot(
    [],
    []
)

confidence_points = ax.scatter(
    [],
    [],
    c=[],
    cmap="coolwarm",
    vmin=MIN_CONFIDENCE,
    vmax=MAX_CONFIDENCE,
    marker="s",
    s=35
)

colour_bar = fig.colorbar(
    confidence_points,
    ax=ax
)

colour_bar.set_label(
    "Occupancy confidence"
)


ax.set_title("RoboCup Live Pose + ToF Map")

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")

ax.grid(True)

ax.set_aspect(
    "equal",
    adjustable="box"
)

ax.set_xlim(
    -VIEW_HALF_WIDTH,
    VIEW_HALF_WIDTH
)

ax.set_ylim(
    -VIEW_HALF_HEIGHT,
    VIEW_HALF_HEIGHT
)

# ============================================================
# ROBOT INFORMATION
# ============================================================

pose_text = fig.text(
    0.02,
    0.15,
    "X: 0 mm    Y: 0 mm    Heading: 0 deg"
)


command_status = fig.text(
    0.02,
    0.125,
    "Command: ready"
)

weight_text = fig.text(
    0.02,
    0.185,
    (
        "LEFT  Top: 0 mm   Bottom: 0 mm   Difference: 0 mm\n"
        "RIGHT Top: 0 mm   Bottom: 0 mm   Difference: 0 mm"
    )
)


# ============================================================
# COMMAND BOX
# ============================================================

command_axis = fig.add_axes(
    [0.15, 0.04, 0.55, 0.06]
)


command_box = TextBox(
    command_axis,
    "Command: "
)


# ============================================================
# SEND BUTTON
# ============================================================

button_axis = fig.add_axes(
    [0.73, 0.04, 0.15, 0.06]
)


send_button = Button(
    button_axis,
    "Send"
)


# ============================================================
# SEND SERIAL COMMAND
# ============================================================

def send_command(command):

    command = command.strip()

    if command == "":
        return


    message = command + "\n"

    ser.write(
        message.encode()
    )

    command_status.set_text(
        "Sent: " + command
    )

    print(
        "Sent:",
        command
    )


def send_button_clicked(event):

    send_command(
        command_box.text
    )


# Pressing Enter sends command
command_box.on_submit(
    send_command
)


# Clicking button sends command
send_button.on_clicked(
    send_button_clicked
)


# ============================================================
# UPDATE VISUALISER
# ============================================================

def sensor_position_robot_frame(angle_deg):

    angle = math.radians(angle_deg)

    sensor_x = (
        SENSOR_CIRCLE_X
        + SENSOR_RADIUS
        * math.cos(angle)
    )

    sensor_y = (
        SENSOR_RADIUS
        * math.sin(angle)
    )

    return sensor_x, sensor_y



def robot_to_world(
    local_x,
    local_y,
    robot_x,
    robot_y,
    heading_deg
):

    heading = math.radians(
        heading_deg
    )

    world_x = (
        robot_x
        + local_x * math.cos(heading)
        - local_y * math.sin(heading)
    )

    world_y = (
        robot_y
        + local_x * math.sin(heading)
        + local_y * math.cos(heading)
    )

    return world_x, world_y


def calculate_tof_beam(angle_deg,distance_mm,robot_x,robot_y,robot_heading):

    # Sensor mounting position relative to ICR
    sensor_local_x, sensor_local_y = (sensor_position_robot_frame(angle_deg))


    # Sensor origin in world coordinates
    sensor_world_x, sensor_world_y = (robot_to_world(sensor_local_x,sensor_local_y,robot_x,robot_y,robot_heading))


    # Beam direction relative to world
    beam_heading = (
        robot_heading
        + angle_deg
    )

    beam_heading_rad = math.radians(
        beam_heading
    )


    hit_world_x = (
        sensor_world_x
        + distance_mm
        * math.cos(beam_heading_rad)
    )

    hit_world_y = (
        sensor_world_y
        + distance_mm
        * math.sin(beam_heading_rad)
    )


    return (
        sensor_world_x,
        sensor_world_y,
        hit_world_x,
        hit_world_y
    )


def add_obstacle_evidence(
    hit_x,
    hit_y
):

    cell_x = round(
        hit_x / CELL_SIZE_MM
    )

    cell_y = round(
        hit_y / CELL_SIZE_MM
    )

    cell = (
        cell_x,
        cell_y
    )


    current_score = cell_confidence.get(
        cell,
        0
    )


    current_score += OBSTACLE_INCREMENT


    current_score = min(
        current_score,
        MAX_CONFIDENCE
    )


    cell_confidence[cell] = (
        current_score
    )


def add_free_evidence(
    x,
    y
):

    cell_x = round(
        x / CELL_SIZE_MM
    )

    cell_y = round(
        y / CELL_SIZE_MM
    )

    cell = (
        cell_x,
        cell_y
    )


    current_score = cell_confidence.get(
        cell,
        0
    )


    current_score -= FREE_DECREMENT


    current_score = max(
        current_score,
        MIN_CONFIDENCE
    )


    cell_confidence[cell] = (
        current_score
    )

def mark_free_along_ray(
    sensor_x,
    sensor_y,
    hit_x,
    hit_y
):

    dx = hit_x - sensor_x
    dy = hit_y - sensor_y

    distance = math.sqrt(
        dx * dx
        + dy * dy
    )

    if distance <= 0:
        return


    step_mm = 50.0

    number_steps = int(
        distance / step_mm
    )

    if number_steps <= 0:
        return


    visited_cells = set()


    for i in range(number_steps):

        fraction = (
            i / number_steps
        )

        remaining_distance = (
            distance
            * (1.0 - fraction)
        )

        # Leave endpoint cell for obstacle evidence
        if remaining_distance < CELL_SIZE_MM:
            break


        x = (
            sensor_x
            + dx * fraction
        )

        y = (
            sensor_y
            + dy * fraction
        )


        cell_x = round(
            x / CELL_SIZE_MM
        )

        cell_y = round(
            y / CELL_SIZE_MM
        )

        cell = (
            cell_x,
            cell_y
        )


        # Only alter each map cell once per sensor reading
        if cell in visited_cells:
            continue

        visited_cells.add(
            cell
        )

        add_free_evidence(
            x,
            y
        )

def update_confidence_plot():

    x_values = []
    y_values = []
    confidence_values = []


    for (cell_x, cell_y), score in cell_confidence.items():

        # Don't display completely unknown cells
        if score == 0:
            continue


        x_values.append(
            cell_x * CELL_SIZE_MM
        )

        y_values.append(
            cell_y * CELL_SIZE_MM
        )

        confidence_values.append(
            score
        )


    # No known cells yet
    if len(x_values) == 0:

        confidence_points.set_offsets(
            np.empty((0, 2))
        )

        confidence_points.set_array(
            np.array([])
        )

        return


    # Plot confidence cells
    confidence_points.set_offsets(
        np.column_stack(
            (
                x_values,
                y_values
            )
        )
    )

    confidence_points.set_array(
        np.array(
            confidence_values,
            dtype=float
        )
    )

def update_plot_view():

    ax.set_xlim(
        current_x - VIEW_HALF_WIDTH,
        current_x + VIEW_HALF_WIDTH
    )

    ax.set_ylim(
        current_y - VIEW_HALF_HEIGHT,
        current_y + VIEW_HALF_HEIGHT
    )


def update_weight_diagnostics():

    left_top = current_weight_tof[
        "left_top"
    ]

    left_bottom = current_weight_tof[
        "left_bottom"
    ]

    right_top = current_weight_tof[
        "right_top"
    ]

    right_bottom = current_weight_tof[
        "right_bottom"
    ]


    left_difference = (
        left_top
        - left_bottom
    )

    right_difference = (
        right_top
        - right_bottom
    )


    weight_text.set_text(
        f"LEFT   "
        f"Top: {left_top:.0f} mm   "
        f"Bottom: {left_bottom:.0f} mm   "
        f"Difference: {left_difference:.0f} mm\n"

        f"RIGHT  "
        f"Top: {right_top:.0f} mm   "
        f"Bottom: {right_bottom:.0f} mm   "
        f"Difference: {right_difference:.0f} mm"
    )


def update(frame):

    global current_x
    global current_y
    global current_heading
    global last_map_plot


    new_telemetry = False


    # ========================================================
    # READ SERIAL DATA
    # ========================================================

    for _ in range(20):

        if ser.in_waiting == 0:
            break


        line = (
            ser.readline()
            .decode(
                errors="ignore"
            )
            .strip()
        )


        # Print non-telemetry messages in terminal
        if not line.startswith("ROBOT,"):

            if line != "":
                print(
                    "Robot:",
                    line
                )

            continue


        parts = line.split(",")


        if len(parts) != 12:
            continue


        try:

            current_x = float(parts[1])
            current_y = float(parts[2])
            current_heading = float(parts[3])


            # Navigation ToFs
            current_tof[
                "outer_left"
            ] = float(parts[4])

            current_tof[
                "inner_left"
            ] = float(parts[5])

            current_tof[
                "inner_right"
            ] = float(parts[6])

            current_tof[
                "outer_right"
            ] = float(parts[7])


            # Weight ToFs
            current_weight_tof[
                "left_top"
            ] = float(parts[8])

            current_weight_tof[
                "left_bottom"
            ] = float(parts[9])

            current_weight_tof[
                "right_top"
            ] = float(parts[10])

            current_weight_tof[
                "right_bottom"
            ] = float(parts[11])


        except ValueError:

            continue


        # Store travelled path
        x_history.append(
            current_x
        )

        y_history.append(
            current_y
        )


        # We have a genuinely new sensor measurement
        new_telemetry = True


    # ========================================================
    # UPDATE ROBOT PATH
    # ========================================================

    path_line.set_data(
        x_history,
        y_history
    )


    # ========================================================
    # UPDATE HEADING LINE
    # ========================================================

    arrow_length = 120.0


    heading_rad = math.radians(
        current_heading
    )


    heading_x = (
        current_x
        + arrow_length
        * math.cos(
            heading_rad
        )
    )


    heading_y = (
        current_y
        + arrow_length
        * math.sin(
            heading_rad
        )
    )


    heading_line.set_data(
        [
            current_x,
            heading_x
        ],
        [
            current_y,
            heading_y
        ]
    )


    # ========================================================
    # UPDATE TOF BEAMS
    # ========================================================

    for (
        sensor_name,
        sensor_angle
    ) in SENSOR_ANGLES.items():


        distance = current_tof[
            sensor_name
        ]


        # Invalid measurement
        if distance <= 0:

            tof_lines[
                sensor_name
            ].set_data(
                [],
                []
            )

            tof_points[
                sensor_name
            ].set_data(
                [],
                []
            )

            continue


        (
            sensor_x,
            sensor_y,
            hit_x,
            hit_y

        ) = calculate_tof_beam(
            sensor_angle,
            distance,
            current_x,
            current_y,
            current_heading
        )


        # Live beam
        tof_lines[
            sensor_name
        ].set_data(
            [
                sensor_x,
                hit_x
            ],
            [
                sensor_y,
                hit_y
            ]
        )


        # Live endpoint
        tof_points[
            sensor_name
        ].set_data(
            [hit_x],
            [hit_y]
        )


        # ====================================================
        # UPDATE CONFIDENCE MAP
        # ====================================================

        if new_telemetry:

            mark_free_along_ray(
                sensor_x,
                sensor_y,
                hit_x,
                hit_y
            )

            add_obstacle_evidence(
                hit_x,
                hit_y
            )


    # ========================================================
    # REDRAW CONFIDENCE MAP AT LOWER RATE
    # ========================================================

    now = time.time()


    if (
        now - last_map_plot
        >= PLOT_MAP_PERIOD
    ):

        update_confidence_plot()

        last_map_plot = now


    # ========================================================
    # UPDATE TEXT
    # ========================================================

    pose_text.set_text(
        f"X: {current_x:.1f} mm    "
        f"Y: {current_y:.1f} mm    "
        f"Heading: {current_heading:.1f} deg"
    )

    update_weight_diagnostics()
    # Keep robot centred in plot
    update_plot_view()


    return (
        path_line,
        heading_line,
        confidence_points,
        *tof_lines.values(),
        *tof_points.values()
    )

def clear_map():

    cell_confidence.clear()

    x_history.clear()
    y_history.clear()


    confidence_points.set_offsets(
        np.empty((0, 2))
    )

    confidence_points.set_array(
        np.array([])
    )


    path_line.set_data(
        [],
        []
    )


    print(
        "Visualiser map cleared"
    )


# ============================================================
# START
# ============================================================

animation = FuncAnimation(
    fig,
    update,
    interval=50,
    blit=False,
    cache_frame_data=False
)


plt.show()


# ============================================================
# CLEAN UP
# ============================================================

ser.close()