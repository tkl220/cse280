import csv
import sys
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import collections as mc
from matplotlib import animation

sensor_log = []
grid_log = []

with open('sensor_segments.csv', 'r') as sensor_csv:
    csv_reader = csv.reader(sensor_csv, delimiter=',')
    sensors = []
    i = 0
    for row in csv_reader:
        segment = [(row[0], row[1]), (row[2], row[3])]
        sensors.append(segment)
        i += 1
        if i%16 == 0:
            sensor_log.append(sensors)
            sensors= []
with open('grid.csv', 'r') as grid_csv:
    csv_reader = csv.reader(grid_csv, delimiter=',')
    grid = []
    i = 0
    for row in csv_reader:
        grid.append([int(x) for x in row[:-1]])
        i += 1
        if i%50 == 0:
            grid_log.append([grid])
            grid = []
grid_log = np.array(grid_log, dtype=np.int8)

if __name__ == '__main__':
    fig, ax = plt.subplots()

    def animate(frame):
        ax.clear()

        # Sensor segments
        sc = mc.LineCollection(sensor_log[frame], linewidths=0.5, colors='#ff4545')
        ax.add_collection(sc)

        # Grid
        ax.imshow(grid_log[frame].T)

        fig.gca().invert_yaxis()
    
    anim = animation.FuncAnimation(fig, animate, frames=len(sensor_log), interval = 100)

    if (len(sys.argv) > 1):
        anim.save(sys.argv[1])
    else:
        plt.show()
