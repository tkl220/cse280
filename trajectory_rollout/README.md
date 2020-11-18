# README to explain the files in this folder


## The Different Files

TrajectoryRollout.cpp
    -- the main file I have been working on. Scoring is not being done correctly right now.
    See the main method to understand the flow of the model, basically:
        each for loop ends up moving the car 1 second forward
        in each for loop you:
            1. calculate points representing possible trajectories of the car from the current position it's at 1 second into the future
            2. scores each possible trajectory and returns the best one
            3. moves 0.1 second in that direction, and continues from that point
        once you exit, the points will be passed to the trajectory.txt file, and also printed to the terminal

** ScoreTesting.cpp
    -- LOOK AT THIS FILE!!!!!- more recent file I have been working on - this file adds the concept of a goal point which we are always chasing and is always moving further away, so this file is the same as TrajectoryROllout.cpp, but the scoring method is different, and working better. The other files I am explaining may bring clarity to how it's working as a whole, but this file is the most important to look at.

trajectory.txt
    -- the text file that is populated in csv format with the coordinates forming the trajectory of the car after you finish running ScoreTesting.cpp

visual.py -- the script to read in the trajectory.txt file, and create a graph of the coordinates


## To run it:

```bash
g++ -o ScoreTesting ScoreTesting.cpp
./ScoreTesting.cpp
```

You have to install the module included in the python file to use the graph visualization thing, but the plot isnt fully working anyway, so instead to visualize the data I reccommend copy-pasting all the points from the terminal into Desmos.com and then adding the line "y = -x + 7" (which is the current line hard-coded into the main method) to see the behavior of the trajectory and how it approaches the line

If you did install the module, then after running ScoreTesting as much as you want and then end, run this:

```bash
python3 visual.py
```

and a graph should appear (doesn't properly work yet)


## Main issues I'm having

- the convergence point of when the car should enter the path-- it approaches it infinitely it seems like but never actually reaches it
- right now it works for moving left or right(+x or -x), but only moves in the +y direction -- not sure if I should account for this somehow
- getting the visualize.py graphs to work
