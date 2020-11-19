# This is a sample Python script.

# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.
import matplotlib
import numpy as np
import pylab as pl
from matplotlib import collections as mc
import csv


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press ⌘F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    walls = []
    sensors = []
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
        wc = mc.LineCollection(walls, linewidths=2)
        sc = mc.LineCollection(sensors, linewidths=0.5, colors='#ff4545')
        fig, ax = pl.subplots()
        ax.add_collection(wc)
        ax.add_collection(sc)
        ax.autoscale()
        ax.margins(0.1)
        ax.grid(True)
        fig.show()
        matplotlib.pyplot.show()
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
