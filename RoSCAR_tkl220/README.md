# Q-Learning

## Usage
### Building
A (one line) Makefile is included.  
### Executing: `roboq [S]`
The executable is called `roboq` by default (this can be changed); 
1. If it is run with no arguments, it builds the Q matrix and outputs a log of the first 100 steps with maximum Q-values. This is enough to make it just over 1 lap around the track (depending on the length of the training).
2. If the first argument is a number `S`, it outputs a log of the first `S` steps in the training process and a log of what it knows about the map via the sensor model for each step. This is mostly random movements and it is used to demonstrate the sensor model building the map.  

Executing in either manner will clear the previous logs.
### Visualization: `python visual.py [file.mp4]`
`visual.py` simply reads the logs output by `roboq` and animates them. It depends on matplotlib and numpy. It will save a video of the animation in the optional command line argument if one is passed. Otherwise, it will display the animation in a new window.  
tested on python 3.7.9
