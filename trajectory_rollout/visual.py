import csv
import matplotlib.pyplot as plt

x = []
y = []

with open('trajectory.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    first = True
    for row in csv_reader:
        x.append(row[0])
        y.append(row[1])



# plot the x and y points
plt.plot(x, y, label = 'trajectory') 

  
# naming the x axis 
plt.xlabel('x axis') 
# naming the y axis 
plt.ylabel('y axis') 
  
# giving a title to my graph 
plt.title('Plot of Trajectory') 

plt.axis([-2, 10, -1, 15])
# function to show the plot 
plt.legend()
plt.show() 
