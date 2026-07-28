import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from matplotlib.colors import Normalize


PORT = "COM25"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

time.sleep(2)


# Start with invalid values
data = np.zeros((8,8))


fig, ax = plt.subplots()

cmap = plt.cm.inferno_r.copy()
cmap.set_bad(color="white")

img = ax.imshow(
    np.ma.masked_where(data <= 0, data),
    cmap=cmap,
    norm=Normalize(vmin=100, vmax=3000),
    interpolation="nearest"
)

plt.colorbar(img, label="Distance (mm)")

ax.set_title("DFRobot SEN0628 ToF Heatmap")


ax.set_xticks(np.arange(-0.5,8,1), minor=True)
ax.set_yticks(np.arange(-0.5,8,1), minor=True)
ax.grid(which="minor", color="white", linewidth=1)


def update(frame):

    global data

    try:
        line = ser.readline().decode("utf-8").strip()

        values = line.split(",")

        if len(values) == 64:

            distances = np.array(
                [int(x) for x in values]
            )

            data = distances.reshape((8,8))

            display = np.ma.masked_where(
                data <= 0,
                data
            )

            img.set_data(display)


    except Exception as e:
        print(e)

    return [img]


ani = FuncAnimation(
    fig,
    update,
    interval=50,
    blit=True
)


plt.show()