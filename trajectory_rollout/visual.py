import csv
import matplotlib.pyplot as plt
import numpy as np


#https://scriptverse.academy/tutorials/python-matplotlib-plot-function.html

# plot path for reference (circle in this case)
figure, axes = plt.subplots()
draw_circle = plt.Circle((0, 5), 5, fill = False)

axes.set_aspect(1)
axes.add_artist(draw_circle)
plt.title('Circle')

# load the text from the trajectory.txt file
x, y = np.loadtxt('trajectory.txt', delimiter=',', unpack=True)
plt.plot(x, y, 'ro', label = 'trajectory')

# naming the x axis 
plt.xlabel('x axis') 
# naming the y axis 
plt.ylabel('y axis') 
  
# giving a title to my graph 
plt.title('Plot of Trajectory') 

# show the legend
plt.legend()

# set axes: [(lowest x, highest x, lowest y, highest y)]
plt.axis([-10, 10, -10, 10])

# shows the plot
plt.show()
