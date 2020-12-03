# This is a sample Python script.

# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.
import matplotlib
import numpy as np
import pylab as pl
from matplotlib import collections as mc
from matplotlib import animation
from matplotlib.animation import FuncAnimation
import csv


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press ⌘F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    walls = []
    sensors = []
    sanim = []
    grid = []
    ganim = []
    i = 0
    with open('track.csv', 'r') as track_csv:
        csv_reader = csv.reader(track_csv, delimiter=',')
        for row in csv_reader:
            segment = [(row[0], row[1]), (row[2], row[3])]
            walls.append(segment)
    with open('sensor_segments.csv', 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            segment = [(row[0], row[1]), (row[2], row[3])]
            sensors.append(segment)
            i += 1
            if i%16 == 0:
                sanim.append(sensors)
                sensors = []
    i = 0
    with open('grid.csv', 'r') as grid_csv:
        csv_reader = csv.reader(grid_csv, delimiter=',')
        for row in csv_reader:
            grid.append([int(x) for x in row[:-1]])
            i += 1
            if i%50 == 0:
                ganim.append([grid])
                grid = []
    ganim = np.array(ganim)
    wc = mc.LineCollection(walls, linewidths=2)
    sc = mc.LineCollection(sensors, linewidths=0.5, colors='#ff4545')

    # Init
    fig, ax = pl.subplots()
    # ax.imshow(grid.T)
    ax.add_collection(wc)
    # ax.add_collection(sc)
    ax.autoscale()
    ax.margins(0.1)
    ax.grid(True)
    fig.gca().invert_yaxis()

    # Manual animation
    for i in range(len(sanim)):
        sc = mc.LineCollection(sanim[i], linewidths=0.5, colors='#ff4545')
        ax.clear()
        ax.imshow(ganim[i].T)
        fig.gca().invert_yaxis()
        ax.add_collection(sc)
        pl.pause(0.001)

    fig.show()
    matplotlib.pyplot.show()
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
