# README to explain the files in this folder


## The Different Files

### TrajectoryRollout.cpp (older work, don't look at this file)
    -- my other files have extended from this one, but scoring is not being done correctly in this file
    See the main method to understand the flow of the model, basically:
        each for loop ends up moving the car 1 second forward
        in each for loop you:
            1. calculate points representing possible trajectories of the car from the current position it's at 1 second into the future
            2. scores each possible trajectory and returns the best one
            3. moves 0.1 second in that direction, and continues from that point
        once you exit, the points will be passed to the trajectory.txt file, and also printed to the terminal

### ** ScoreTesting.cpp (this file shows the car approaching a line)
    -- more recent file I have been working on - this file adds the concept of a goal point which we are always chasing and is always moving further away, so this file is the same as TrajectoryRollout.cpp, but the scoring method is different, and working better. The other files I am explaining may bring clarity to how it's working as a whole, but this file is the most important to look at.

### **** Simulation.cpp (BEST working file -- car runs in a circle)
    --  most recent file I have been working on -- this file simulates the robot running on a circular path
    Similar to the previoud two files:
    See the main method to understand the flow of the model, basically:
        each complete for loop ends up moving the car 1 second forward, each iteration moves it 0.1 second forward
        in each for loop you:
            1. calculate points representing possible trajectories of the car from the current position it's at 1 second into the future
            2. scores each possible trajectory and determine the best one
            3. move 0.1 second in that direction, and continues from that point
        once you choose to exit, the points will be passed to the trajectory.txt file, and also printed to the terminal, where you can then visualize the trajectory by running the visual.py file (see below)

### trajectory.txt
    -- the text file that is populated in csv format with the coordinates forming the trajectory of the car after you finish running ScoreTesting.cpp or 

### visual.py -- the script to read in the trajectory.txt file, and create a graph of the trajectory
            NOTE: currently, it graphs a circle representing the path because I was running Simulation.cpp to get the trajectory


## To run it:

```bash
g++ -o Sim Simulation.cpp
./ScoreTesting.cpp
```

You have to install the module included in the python file to use the graph visualization
Once you've installed the module, then after running ScoreTesting as much as you want and then end, run this:

```bash
python3 visual.py
```

and a graph should appear showing the trajectory
