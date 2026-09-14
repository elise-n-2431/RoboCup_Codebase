import serial
import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------------------
# Serial
# --------------------------------------------------

SERIAL_PORT = "COM10"
BAUD_RATE = 115200

reading_weight = False

MAP_WIDTH = 97 + 4  # 2 cells at each extrema for walls
MAP_HEIGHT = 49 + 4

ser = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=1
)

self_x = 3
self_y = 3
tof_readings = [0] * 9

# --------------------------------------------------
# Maps
# --------------------------------------------------

weight_map = np.zeros((MAP_HEIGHT, MAP_WIDTH))
obstacle_map = np.zeros((MAP_HEIGHT, MAP_WIDTH))
frontier_map = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype=bool)


def read_map(start_marker, end_marker):
    """
    Read one complete map from Serial2.
    """

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if line == start_marker:
            break

    data = []

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if line == end_marker:
            break

        try:
            row = [int(value) for value in line.split(",")]

            if len(row) == MAP_WIDTH:
                data.append(row)

        except ValueError:
            pass

    if len(data) != MAP_HEIGHT:
        return None

    return np.array(data)


def read_position():
    """
    Read the robot's current map position.
    """

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if line == "Current position":
            position = ser.readline().decode(errors="ignore").strip()

            try:
                return map(int, position.split(","))

            except ValueError:
                return None

def read_tof():
    """
    Read the 9 ToF sensor distances.
    """

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if line == "TOF readings":
            readings = ser.readline().decode(errors="ignore").strip()

            try:
                return [int(value) for value in readings.split(",")]

            except ValueError:
                return None
            
def read_heading():
    """
    Read the robot's current heading.
    """

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if line == "Heading":
            heading = ser.readline().decode(errors="ignore").strip()

            try:
                return heading

            except ValueError:
                return None

def read_target():
    """
    Read the robot's current map position.
    """

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if line == "Target":
            position = ser.readline().decode(errors="ignore").strip()

            try:
                return map(int, position.split(","))

            except ValueError:
                return None

# --------------------------------------------------
# Plot
# --------------------------------------------------

plt.ion()


plt.ion()

if reading_weight:
    fig, (ax_weight, ax_obstacle) = plt.subplots(
        2,
        1,
        figsize=(14, 8)
    )

    weight_plot = ax_weight.imshow(
        weight_map,
        origin="upper",
        interpolation="nearest",
        vmin=0,
        vmax=1000
    )

    ax_weight.set_title("Weight Map")
    ax_weight.set_xlabel("X cell")
    ax_weight.set_ylabel("Y cell")

else:
    fig, ax_obstacle = plt.subplots(
        1,
        1,
        figsize=(14, 8)
    )


obstacle_plot = ax_obstacle.imshow(
    obstacle_map,
    origin="upper",
    interpolation="nearest",
    vmin=-7,
    vmax=7
)

# Overlay: only frontier==True cells are drawn, everything else transparent
from matplotlib.colors import ListedColormap

frontier_cmap = ListedColormap(["#00e5ff"])  # pick any color you like

frontier_plot = ax_obstacle.imshow(
    np.ma.masked_where(~frontier_map, frontier_map),
    origin="upper",
    interpolation="nearest",
    cmap=frontier_cmap,
    vmin=0,
    vmax=1,
    alpha=0.85  # tweak transparency so obstacle map still peeks through if you want
)

# weight_text = []
# obstacle_text = []

# for y in range(MAP_HEIGHT):
#     weight_row = []
#     obstacle_row = []

#     for x in range(MAP_WIDTH):
#         weight_row.append(
#             ax_weight.text(
#                 x, y, "",
#                 ha="center",
#                 va="center",
#                 fontsize=8
#             )
#         )

#         obstacle_row.append(
#             ax_obstacle.text(
#                 x, y, "",
#                 ha="center",
#                 va="center",
#                 fontsize=8
#             )
#         )

#     weight_text.append(weight_row)
#     obstacle_text.append(obstacle_row)

ax_obstacle.set_title("Obstacle Map")
ax_obstacle.set_xlabel("X cell")
ax_obstacle.set_ylabel("Y cell")

# Position statistic
position_text = fig.text(
    0.6,
    0.99,
    "Position: (0, 0)",
    ha="center",
    va="top",
    fontsize=12
)

heading_text = fig.text(
    0.7,
    0.99,
    "Heading: 0 degrees",
    ha="center",
    va="top",
    fontsize=12
)

target_text = fig.text(
    0.4,
    0.99,
    "Target: (0, 0)",
    ha="center",
    va="top",
    fontsize=12
)

tof_ax = fig.add_axes([0.72, 0.05, 0.25, 0.35])
tof_ax.set_xlim(0, 3)
tof_ax.set_ylim(0, 4)
tof_ax.set_aspect("equal")
tof_ax.axis("off")

tof_positions = {
    3:(0.5, 4.0),
    2:(1.5, 4.0),
    1:(2.5, 4.0),
    0:(3.5, 4.0),

    5:(1.5, 2.8),
    4:(2.5, 2.8),

    7:(1.5, 1.6),
    6:(2.5, 1.6),

    8:(2.0, 0.4)
}

tof_text = {}

for sensor, (x, y) in tof_positions.items():
    tof_text[sensor] = tof_ax.text(
        x,
        y,
        f"{sensor}\n--- mm",
        ha="center",
        va="center",
        fontsize=11
    )

plt.tight_layout()


# --------------------------------------------------
# Main loop
# --------------------------------------------------
def read_obstacle_map_quantized(height, width):
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "OBSTACLE_MAP_START":
            break

    hex_line = ser.readline().decode(errors="ignore").strip()
    end_line = ser.readline().decode(errors="ignore").strip()

    if end_line != "OBSTACLE_MAP_END":
        return None

    if len(hex_line) != height * width:
        return None

    try:
        values = []
        for c in hex_line:
            v = int(c, 16)
            if v > 7:       # undo 4-bit two's complement
                v -= 16
            values.append(v)
    except ValueError:
        return None

    return np.array(values, dtype=np.int32).reshape((height, width))

def read_frontier_bitmap(height, width):
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "FRONTIER_MAP_START":
            break

    hex_line = ser.readline().decode(errors="ignore").strip()
    end_line = ser.readline().decode(errors="ignore").strip()

    if end_line != "FRONTIER_MAP_END":
        return None

    try:
        raw = bytes.fromhex(hex_line)
    except ValueError:
        return None

    bits = np.unpackbits(np.frombuffer(raw, dtype=np.uint8))
    bits = bits[: height * width]
    return bits.reshape((height, width)).astype(bool)




try:
    while True:

        if reading_weight:
            new_weight_map = read_map("WEIGHT_MAP_START", "WEIGHT_MAP_END")
        new_obstacle_map = read_obstacle_map_quantized(MAP_HEIGHT, MAP_WIDTH)
        # new_obstacle_map = read_map("OBSTACLE_MAP_START", "OBSTACLE_MAP_END")
        new_frontier_map = read_frontier_bitmap(MAP_HEIGHT, MAP_WIDTH)


        # for y in range(MAP_HEIGHT):
        #     for x in range(MAP_WIDTH):
        #         weight_text[y][x].set_text(str(weight_map[y, x]//100))
        #         obstacle_text[y][x].set_text(str(obstacle_map[y, x]//100))

        position = read_position()
        readings = read_tof()
        heading = read_heading()
        target = read_target()

        if readings is not None and len(readings) == 9:
            tof_readings = readings

            for sensor in range(9):
                tof_text[sensor].set_text(
                    f"{sensor}\n{tof_readings[sensor]} mm"
                )

        if position is not None:
            self_x, self_y = position

        if target is not None:
            target_x, target_y = target

        if reading_weight:
            if new_weight_map is not None:
                weight_map = new_weight_map

        if new_obstacle_map is not None:
            obstacle_map = new_obstacle_map

        if new_frontier_map is not None:
            frontier_map = new_frontier_map.astype(bool)

        if reading_weight:
            weight_map[self_x][self_y] = 1000
            weight_plot.set_data(weight_map)
        obstacle_plot.set_data(obstacle_map)
        frontier_plot.set_data(np.ma.masked_where(~frontier_map, frontier_map))

        position_text.set_text(
            f"Position: ({self_x}, {self_y})"
        )
        heading_text.set_text(
            f"Heading: {heading} degrees"
        )
        target_text.set_text(
            f"Target: ({target_x}, {target_y})"
        )

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Stopped.")

finally:
    ser.close()