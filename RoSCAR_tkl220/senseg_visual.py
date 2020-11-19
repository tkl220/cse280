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
    lines = []
    with open('track.csv', 'r') as track_csv:
        csv_reader = csv.reader(track_csv, delimiter=',')
        for row in csv_reader:
            segment = [(row[0], row[1]), (row[2], row[3])]
            lines.append(segment)
    with open('senseg_old.csv', 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            segment = [(row[0], row[1]), (row[2], row[3])]
            lines.append(segment)
        lc = mc.LineCollection(lines, linewidths=2)
        fig, ax = pl.subplots()
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        fig.show()
        matplotlib.pyplot.show()
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
