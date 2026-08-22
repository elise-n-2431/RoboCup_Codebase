import serial
import math

import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox, Button


# ============================================================
# SETTINGS
# ============================================================

PORT = "COM21"
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

x_history = []
y_history = []

current_x = 0.0
current_y = 0.0
current_heading = 0.0


# ============================================================
# PLOT SETUP
# ============================================================

fig, ax = plt.subplots()

# Leave room at bottom for serial command controls
fig.subplots_adjust(
    bottom=0.22
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


ax.set_title(
    "RoboCup Live Pose"
)

ax.set_xlabel(
    "X (mm)"
)

ax.set_ylabel(
    "Y (mm)"
)

ax.grid(True)

ax.set_aspect(
    "equal",
    adjustable="box"
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
def update(frame):

    global current_x
    global current_y
    global current_heading


    # Only process a limited number each frame so
    # matplotlib always gets time to redraw.
    for _ in range(20):

        if ser.in_waiting == 0:
            break

        line = (
            ser.readline()
            .decode(errors="ignore")
            .strip()
        )


        if not line.startswith("POSE,"):

            if line != "":
                print("Robot:", line)

            continue


        parts = line.split(",")

        if len(parts) != 4:
            continue


        try:

            current_x = float(parts[1])
            current_y = float(parts[2])
            current_heading = float(parts[3])

        except ValueError:

            continue


        x_history.append(current_x)
        y_history.append(current_y)


    # Update path
    path_line.set_data(
        x_history,
        y_history
    )


    # Heading line
    arrow_length = 120.0

    heading_rad = math.radians(
        current_heading
    )

    heading_x = (
        current_x
        + arrow_length
        * math.cos(heading_rad)
    )

    heading_y = (
        current_y
        + arrow_length
        * math.sin(heading_rad)
    )


    heading_line.set_data(
        [current_x, heading_x],
        [current_y, heading_y]
    )


    pose_text.set_text(
        f"X: {current_x:.1f} mm    "
        f"Y: {current_y:.1f} mm    "
        f"Heading: {current_heading:.1f} deg"
    )


    # Update view
    if len(x_history) > 0:

        margin = 300

        x_min = min(
            min(x_history) - margin,
            -margin
        )

        x_max = max(
            max(x_history) + margin,
            margin
        )

        y_min = min(
            min(y_history) - margin,
            -margin
        )

        y_max = max(
            max(y_history) + margin,
            margin
        )

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)


    return path_line, heading_line


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