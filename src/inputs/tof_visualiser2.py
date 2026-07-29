import math
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

PORT = "COM25"
BAUD = 115200




import math
import random

def calc(index, depth):
    # print(index % 8)
    # print(index // 8)

    theta = math.radians(((index % 8) - 3.5) * 7.5)
    gamma = math.radians(((index // 8) - 3.5) * 7.5)
    # print(theta, gamma)

    x_prime = math.tan(theta)
    z_prime = math.tan(gamma)
    mag = math.sqrt(x_prime ** 2 + z_prime ** 2 + 1)
    x = depth * (x_prime / mag)
    y = depth * (1 / mag)
    z = depth * (z_prime / mag)

    print(x, y, z)


        
for i in range(64):
    depth = random.randint(1, 100)
    calc(i, depth)





FOV_DEG = 60.0
PIXEL_STEP = FOV_DEG / 8  # 7.5 degrees per pixel
MIN_VALID_MM = 20     # sensor floor, per datasheet (20mm-4000mm range)
MAX_VALID_MM = 4000
VMIN, VMAX = 100, 3000  # colour/axis scaling for display

# Set based on your sensor's actual row ordering (top row = index 0 or not).
# Verify once against a known scene (e.g. point it at a flat wall/floor)
# before trusting the 3D reconstruction.
FLIP_ROW = True


def build_unit_rays(flip_row=True):
    """
    Precompute the unit ray direction (dx, dy, dz) for each of the 64
    pixels once. These never change at runtime -- only the raw range r
    changes per frame, so we just scale these by r each update.
    Returns three (8,8) arrays: dx, dy, dz.
    """
    dx = np.zeros((8, 8))
    dy = np.zeros((8, 8))
    dz = np.zeros((8, 8))
    for i in range(8):
        for j in range(8):
            phi = math.radians((j - 3.5) * PIXEL_STEP)
            row_term = (3.5 - i) if flip_row else (i - 3.5)
            theta = math.radians(row_term * PIXEL_STEP)

            x_prime = math.tan(phi)
            z_prime = math.tan(theta)
            mag = math.sqrt(x_prime**2 + z_prime**2 + 1)

            dx[i, j] = x_prime / mag
            dy[i, j] = 1 / mag
            dz[i, j] = z_prime / mag
    return dx, dy, dz


DX, DY, DZ = build_unit_rays(flip_row=FLIP_ROW)

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

data = np.zeros((8, 8))  # raw range grid, mm

# ---------- figure layout: heatmap on the left, 3D scatter on the right ----------
fig = plt.figure(figsize=(13, 6))
ax1 = fig.add_subplot(1, 2, 1)
ax2 = fig.add_subplot(1, 2, 2, projection='3d')

cmap = plt.cm.inferno_r.copy()
cmap.set_bad(color="white")

img = ax1.imshow(
    np.ma.masked_where(data <= 0, data),
    cmap=cmap,
    norm=Normalize(vmin=VMIN, vmax=VMAX),
    interpolation="nearest",
)
plt.colorbar(img, ax=ax1, label="Distance (mm)")
ax1.set_title("DFRobot SEN0628 ToF heatmap")
ax1.set_xticks(np.arange(-0.5, 8, 1), minor=True)
ax1.set_yticks(np.arange(-0.5, 8, 1), minor=True)
ax1.grid(which="minor", color="white", linewidth=1)
ax1.set_xlabel("column j (x)")
ax1.set_ylabel("row i (z)")

# Pre-create the 64 text labels once; we only update their content/color
# each frame rather than recreating them (much cheaper).
text_objs = [[None] * 8 for _ in range(8)]
for i in range(8):
    for j in range(8):
        text_objs[i][j] = ax1.text(
            j, i, "", ha="center", va="center", fontsize=7, color="white"
        )

# ---------- 3D scatter setup ----------
sc = ax2.scatter([], [], [], c=[], cmap="viridis", vmin=VMIN, vmax=VMAX, s=30)
ax2.set_xlabel("x (left/right, mm)")
ax2.set_ylabel("y (forward/depth, mm)")
ax2.set_zlabel("z (up/down, mm)")
ax2.set_title("Live 3D point cloud")

# Fixed axis limits so the plot doesn't jump/rescale every frame.
half_span = VMAX * math.tan(math.radians(FOV_DEG / 2))
ax2.set_xlim(-half_span, half_span)
ax2.set_ylim(0, VMAX)
ax2.set_zlim(-half_span, half_span)
ax2.view_init(elev=15, azim=-60)

fig.colorbar(sc, ax=ax2, shrink=0.6, label="Distance (mm)")


def update(frame):
    global data

    try:
        line = ser.readline().decode("utf-8").strip()
        values = line.split(",")

        if len(values) == 64:
            distances = np.array([int(x) for x in values])
            data = distances.reshape((8, 8))

            # --- heatmap ---
            valid_mask = (data >= MIN_VALID_MM) & (data <= MAX_VALID_MM)
            display = np.ma.masked_where(~valid_mask, data)
            img.set_data(display)

            for i in range(8):
                for j in range(8):
                    if valid_mask[i, j]:
                        text_objs[i][j].set_text(f"{int(data[i, j])}")
                        # flip label colour on bright cells so it stays readable
                        text_objs[i][j].set_color(
                            "black" if data[i, j] > (VMIN + VMAX) / 2 else "white"
                        )
                    else:
                        text_objs[i][j].set_text("")

            # --- 3D point cloud ---
            r = np.where(valid_mask, data, np.nan)
            x = r * DX
            y = r * DY
            z = r * DZ

            xf = x[valid_mask]
            yf = y[valid_mask]
            zf = z[valid_mask]
            cf = data[valid_mask]

            sc._offsets3d = (xf, yf, zf)
            sc.set_array(cf)

    except Exception as e:
        print(e)

    return [img, sc, *[t for row in text_objs for t in row]]


# blit=False: mixing a 3D axes and per-cell text objects doesn't play
# well with blitting, and at 50ms/frame a full redraw is cheap enough.
ani = FuncAnimation(fig, update, interval=50, blit=False)

plt.tight_layout()
plt.show()

ser.close()