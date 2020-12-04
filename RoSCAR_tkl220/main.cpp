#include <iostream>
#include <string>
#include <stdlib.h>
#include <random>
#include "Segment.h"
#include "sensors.h"
#include <fstream>
#include <algorithm>

using namespace std;
// Credit to Jin Fagang from https://github.com/jinfagang/Q-Learning
// for basis of this code.

#define DEBUG false     // Print debug information (Q values) and use consistent rand() seed

// map relevant definitions
#define NUM_ACTIONS 8 // N {0,1} E {1,0} S {0,-1} W {-1,0} 
                      // NE {1,1} SE {1,-1} SW {-1,-1} NW {-1,1}
#define N 50 //size of map NxN
#define MAP_SIZE 2500 //total size of map, MAP_SIZE = NxN

// Q-learning relevant definitions
#define ALPHA .1 //weight of newly learned behavior
#define GAMMA .7 //weight of next action
#define EPSILON .3 //chance next action is random
#define EPISODES 1000 //number of times to find goal

#define TRANSITIONS_TO_PRINT 10 //number of actions from state to print in traversing map using converged Q-matrix

int anim_steps = 200;
bool ATRAIN = false;    // Animate 
int gridworld[N][N] = {0};
double Q[MAP_SIZE][NUM_ACTIONS] = {0};
double R[MAP_SIZE][NUM_ACTIONS] = {0};
// Forward declarations
vector<Segment> track;
void run_sensors(int init_state);
void update_senseg_csv(pair<double, double> agent, vector<Segment> sensor_segments);
void write_out_gridworld(int (&grid)[N][N]);
void write_out_grid_update(int (&grid)[N][N]);

int action_inverse(int action) {
    if(action == 0) {           // N
        return 2;
    } else if(action == 1) {    // E
        return 3;
    } else if(action == 2) {    // S
        return 0;
    } else if(action == 3) {    // W
        return 1;
    } else if(action == 4) {    // NE
        return 6;
    } else if(action == 5) {    // SE
        return 7;
    } else if(action == 6) {    // SW
        return 4;
    } else if(action == 7) {    // NW
        return 5;
    }
}

/**
 * Make a checkpoint with rewards for parameter actions
 * in the principal direction parameter direction
 */ 
void draw_line(Segment seg, vector<int> actions, int direction) {
    pair<double, double> p1(seg.p1.first + 0.5, seg.p1.second + 0.5);
    pair<double, double> p2(seg.p2.first + 0.5, seg.p2.second + 0.5);
    if (seg.p2.first < seg.p1.first) {
        pair<double,double> temp = p1;
        p1 = p2;
        p2 = temp;
    }

    double dx = p2.first - p1.first;
    double dy = p2.second - p1.second;
    double slope = abs(dy/dx);
    if(slope <= .5) {
       for (int x = (int)(p1.first); x < (int)(p2.first); x++) {
           int ny  = (int)(p1.second + dy * (x - p1.first) / dx);
      	   int sy = ny - direction;
           for (int i = 0; i < actions.size(); i++) {
	       int action = actions[i];
	       int r_action = action_inverse(action);
               if(i < 3) {
	       	   R[N*sy+x][action] = 100.0;
	       }
               R[N*ny+x][r_action] = -500.0;
	       R[N*sy+x][r_action] = -500.0;
	       if((N*sy+x)%50 != 0) {
	   	   R[N*sy+x-1][r_action] = -500.0;
	       }
	       if((N*ny+x)%50 != 0) {
		   R[N*ny+x-1][r_action] = -500.0;
	       }
	       if((N*sy+x)%50 != 49) {
	    	   R[N*sy+x+1][r_action] = -500.0;
	       }
	       if((N*ny+x)%50 != 49) {
	    	   R[N*ny+x+1][r_action] = -500.0;
	       }
           }
       }	
    } else if(slope > .5) {
        if(seg.p2.second < seg.p1.second) {
            pair<double,double> temp = p1;
            p1 = p2;
            p2 = temp;
        }
        for (int y = (int)(p1.second); y < (int)(p2.second); y++) {
            int nx  = (int)(p1.first + dx * (y - p1.second) / dy);
            int sx = nx - direction;
            for (int i = 0; i < actions.size(); i++) {
                int action = actions[i];
                int r_action = action_inverse(action);
                    if(i < 3) {
                    R[N*y+sx][action] = 100.0;
                }
                    R[N*y+nx][r_action] = -500.0;
                R[N*y+sx][r_action] = -500.0;
                if((N*y+sx)%50 != 0) {
                R[N*y+sx-1][r_action] = -500.0;
                }
                if((N*y+nx)%50 != 0) {
                R[N*y+nx-1][r_action] = -500.0;
                }
                if((N*y+sx)%50 != 49) {
                    R[N*y+sx+1][r_action] = -500.0;
                }
                if((N*y+nx)%50 != 49) {
                    R[N*y+nx+1][r_action] = -500.0;
                }
            }
        }	
    }
}

/*
 * Check if action is valid from given state
 */
bool check_action(int state, int action) {
    if(action == 0 && gridworld[state%N][state/N + 1] == 1) {               // N
        return false;
    } else if(action == 1 && gridworld[state%N + 1][state/N] == 1) {        // E
        return false;
    } else if(action == 2 && gridworld[state%N][state/N - 1] == 1) {        // S
        return false;
    } else if(action == 3 && gridworld[state%N - 1][state/N] == 1) {        // W
        return false;
    } else if((action == 4 && ((gridworld[state%N + 1][state/N + 1] == 1)
        || (gridworld[state%N][state/N + 1] == 1 && gridworld[state%N + 1][state/N] == 1)))) {    // NE
        return false;
    } else if((action == 5 && ((gridworld[state%N + 1][state/N - 1] == 1)
        || (gridworld[state%N][state/N - 1] == 1 && gridworld[state%N + 1][state/N] == 1)))) {    // SE
        return false;
    } else if((action == 6 && ((gridworld[state%N - 1][state/N - 1] == 1)
        || (gridworld[state%N - 1][state/N] == 1 && gridworld[state%N][state/N - 1] == 1)))) {    // SW
        return false;
    } else if((action == 7 && ((gridworld[state%N - 1][state/N + 1] == 1)
        || (gridworld[state%N - 1][state/N] == 1 && gridworld[state%N][state/N + 1] == 1)))) {    // NW
        return false;
    }
    return true;
}

/*
 * get max value in Q matrix from given state
 */
int max_q_action(int state) {
    double max = -999;
    int action = -1;
    int i = 0;
    vector<int> equal_actions = {};
    for (i = 0; i < NUM_ACTIONS; i++) {
        if (Q[state][i] >= max && check_action(state, i)) {
            if (Q[state][i] > max) {
                equal_actions.clear();
            }
            equal_actions.push_back(i);
            max = Q[state][i];
            action = i;
        }
    }
    int idx = rand()%equal_actions.size();
    action = equal_actions[idx];
    if (DEBUG) cout << "Q[" << state << "][" << action << "], R[" << state << "][" << action << "], = " << Q[state][action] << ", " << R[state][action] << endl;
    return action;
}

/*
* old state + action = new state
*/
int update_state(int state, int action) {
    int new_state = -1;
    if(action == 0) new_state = state + N;          // North
    else if(action == 1) new_state = state + 1;     // East
    else if(action == 2) new_state = state - N;     // South
    else if(action == 3) new_state = state - 1;     // West
    else if(action == 4) new_state = state + N + 1; // NE
    else if(action == 5) new_state = state - N + 1; // SE
    else if(action == 6) new_state = state - N - 1; // SW
    else if(action == 7) new_state = state + N - 1; // NW
    return  new_state;
}

/*
 * Runs one iteration of finding the goal state.
 */
int episode_iterator(int init_state){
    int next_state = -1;
    int action = -1;
    double old_q;

    int step=0;
    if (ATRAIN) step = 10000 - anim_steps;
    while (step < 10000) {

        run_sensors(init_state);
        if (ATRAIN) write_out_grid_update(gridworld);

        // get next action
        double r = (float) rand()/RAND_MAX ;
        action = rand() % NUM_ACTIONS;
        //random chance to select random action
        if (r < EPSILON) {
            while(!check_action(init_state, action)) action = rand() % NUM_ACTIONS;
        } else {
            action = max_q_action(init_state);
        }

        next_state = update_state(init_state, action);
        old_q = Q[init_state][action];
        Q[init_state][action] = old_q+ALPHA*(R[init_state][action]+GAMMA*Q[next_state][max_q_action(next_state)]-old_q);

        init_state = next_state;
        step++;
    }

    return init_state;
}

/*
 * Runs EPISODES number of iterations of finding the goal state to create a converged Q-matrix.
 */
void train(int init_state){
    // start random
    srand((unsigned)time(NULL));
    if (DEBUG) srand(1);
    cout << "[INFO] start training..." << endl;
    for (int i = 0; i < EPISODES; ++i) {
        if (ATRAIN && i) break;
        episode_iterator(init_state);
        cout <<  '\r' << i << '/' << EPISODES << '\t' << std::flush;
    }
    cout << "[INFO] end training..." << endl;
}

void run_Qlearing(int state) {
    for (int j = 0; j < 100; j++) {
        int action = max_q_action(state);
        state = update_state(state, action);    
        pair<double, double> agent(state%N, state/N);    
        update_senseg_csv(agent,get_sensor_segments(track, agent, sensor_16(agent)));
	if (!ATRAIN) write_out_grid_update(gridworld);
    }
}

void write_out_gridworld(int (&grid)[N][N]) {
    ofstream out_file("grid.csv");
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < N; j++) {
            out_file << grid[i][j] << ",";
        }
        out_file << endl;
    }
    return;
}

void write_out_grid_update(int (&grid)[N][N]) {
    ofstream out_file("grid.csv", std::ios_base::app);
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < N; j++) {
            out_file << grid[i][j] << ",";
        }
        out_file << endl;
    }
    return;
}

void update_track_csv(vector<Segment> track) {
    ofstream out_file("track.csv");
    for(auto segment : track) {
        out_file << segment.p1.first << "," << segment.p1.second << "," << segment.p2.first 
            << "," << segment.p2.second << endl;
    }
}

void update_senseg_csv(pair<double, double> agent, vector<Segment> sensor_segments) {
    ofstream out_file("sensor_segments.csv", std::ios_base::app);
    for(auto segment : sensor_segments) {
        out_file << agent.first << "," << agent.second << "," << segment.p2.first << "," << segment.p2.second << endl;
    }

}

void run_sensors(int init_state) {
    // NB: This is the only place we should be dealing with decimals - keep casting to a minimum overall
    pair<int, int> agent = make_pair(init_state%N, init_state/N);

    //gets segments representing sensors, functions and such found in sensors.cpp and Segment.cpp
    vector<pair<double, pair<double, double>>> sensors = sensor_16(agent);
    vector<Segment> sensor_segments = get_sensor_segments(track, agent, sensors);
    for(auto segment : sensor_segments) {
        gridworld[(int)(segment.p2.first + 0.5)][(int)(segment.p2.second + 0.5)] = 1;
    }
    // update_track_csv(track);
    if (ATRAIN) update_senseg_csv(agent, sensor_segments);
}

void build_checkpoints() {
    int old_n = 8;

    vector<Segment> checkpoints;
    // Turns -------------------------
    checkpoints.push_back(Segment(make_pair(-2,6), make_pair(-1.25,4.75)));
    checkpoints.push_back(Segment(make_pair(2,6), make_pair(1.25,4.75)));
    checkpoints.push_back(Segment(make_pair(2.25,3.75), make_pair(4,4)));
    checkpoints.push_back(Segment(make_pair(1.25,1.75), make_pair(2,0)));
    checkpoints.push_back(Segment(make_pair(-1.25,1.75), make_pair(-2,0)));
    checkpoints.push_back(Segment(make_pair(-2.25,3.75), make_pair(-4,4)));
    transform(checkpoints.begin(), checkpoints.end(), checkpoints.begin(), 
        [old_n](Segment &seg){return Segment(make_pair(seg.p1.first+old_n/2, seg.p1.second), 
                                            make_pair(seg.p2.first+old_n/2, seg.p2.second));}
    );
    transform(checkpoints.begin(), checkpoints.end(), checkpoints.begin(), 
        [old_n](Segment &seg){return Segment(make_pair(seg.p1.first*N/old_n, seg.p1.second*N/old_n),
                                            make_pair(seg.p2.first*N/old_n, seg.p2.second*N/old_n));}
    );
    draw_line(checkpoints[0], {0, 4, 1, 5}, 1);
    draw_line(checkpoints[1], {2, 5, 1, 4}, 1);
    draw_line(checkpoints[2], {6, 2, 5, 1}, -1);
    draw_line(checkpoints[3], {3, 6, 2, 7}, -1);
    draw_line(checkpoints[4], {0, 7, 3, 6}, 1);
    draw_line(checkpoints[5], {0, 7, 4, 1}, 1);
}

int main(int argc, char* argv[]) {
    // INPUTS -------------------------
    if (argc > 1) {
        ATRAIN = true;
        anim_steps = atoi(argv[1]);
    }
    track = track1();
    build_checkpoints();
    double old_n = 8;
    double x = -3;
    double y = 4;
    // Shift & scale inputs -----------
    // [(-4,-1), (4,7)] => [(0,0), (8,8)] => [(0,0), (N,N)]
    transform(track.begin(), track.end(), track.begin(), 
        [old_n](Segment &seg){return Segment(make_pair(seg.p1.first+old_n/2, seg.p1.second), 
                                            make_pair(seg.p2.first+old_n/2, seg.p2.second));}
    );
    transform(track.begin(), track.end(), track.begin(), 
        [old_n](Segment &seg){return Segment(make_pair(seg.p1.first*N/old_n, seg.p1.second*N/old_n),
                                            make_pair(seg.p2.first*N/old_n, seg.p2.second*N/old_n));}
    );
    x += old_n/2.0;
    x *= N/old_n;
    y *= N/old_n;
    // Do the Q -----------------------
    ofstream out_file("sensor_segments.csv");
    out_file.close(); // Clear it
    ofstream out_file1("grid.csv");
    out_file.close(); // Clear it
    train(N*y+x);
    if (!ATRAIN) run_Qlearing(N*y+x);
    return 0;
}
