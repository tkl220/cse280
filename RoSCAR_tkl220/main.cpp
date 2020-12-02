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

// map relevant definitions
#define NUM_ACTIONS 8 // N {0,1} E {1,0} S {0,-1} W {-1,0} 
                      // NE {1,1} SE {1,-1} SW {-1,-1} NW {-1,1}
#define N 50 //size of map NxN
#define MAP_SIZE 2500 //total size of map, MAP_SIZE = NxN

// Q-learning relevant definitions
#define ALPHA .1 //weight of newly learned behavior
#define GAMMA .7 //weight of next action
#define EPSILON .3 //chance next action is random
#define DESTINATION 8 //state which has positive reward (cheese)
#define EPISODES 3000 //number of times to find goal

#define TRANSITIONS_TO_PRINT 10 //number of actions from state to print in traversing map using converged Q-matrix

int gridworld[N][N] = {0};
double Q[MAP_SIZE][NUM_ACTIONS] = {0};
double R[MAP_SIZE][NUM_ACTIONS] = {0};
// 20,000 doubles = 160kB?
vector<Segment> track;
void run_sensors(int init_state);
std::pair<int, int> actions[8] = { // TODO: Maybe use this instead?
    make_pair(0,1),     // N
    make_pair(1,0),     // E
    make_pair(0,-1),    // S
    make_pair(-1,0),    // W
    make_pair(1,1),     // NE
    make_pair(1,-1),    // SE
    make_pair(-1,-1),   // SW
    make_pair(-1,1)     // NW
};

// TODO: set up checkpoints in R matrix
// Make them floats for my laptop doesn't have much memory please
// positive reward for going clockwise
// negative reward for going counter-clockwise
// y = int(y1 + dy*(x - x1) / dx) doesn't work for vertical lines! - just transpose it?

/*
 * Check if action is valid from given state
 */
// bool check_action(int state, std::pair<int,int> action) {
bool check_action(int state, int action) {
    // TODO: use gridworld to check if action is valid
    // int x = state%N;
    // int y = state/N;
    // return !gridworld[x + action.first][y + action.second];
    return true;
}

/*
 * get max value in Q matrix from given state
 */
int max_q_action(int state){
    double max = -999999;
    int action = -1;
    int i = 0;
    for (i = 0; i < NUM_ACTIONS; i++) {
        if (Q[state][i] >= max && check_action(state, i)) {
            max = Q[state][i];
            action = i;
        }
    }
    return action;
}

/*
* old state + action = new state
*/
// int update_state(int state, std::pair<int,int> action) {
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
    // return state + N % action.first + N / action.second
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
    while (init_state != DESTINATION){

        run_sensors(init_state);

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
    srand((unsigned)time(NULL)); // TODO: What does this do?
    cout << "[INFO] start training..." << endl;
    for (int i = 0; i < EPISODES; ++i) {
        episode_iterator(init_state);
    }
    cout << "[INFO] end training..." << endl;
}

void run_Qlearing(int init_state, vector<Segment> *track) {
    train(init_state);

    int state = 0;
    while(state != DESTINATION) { //while not at goal state continue asking for new start state...
        cout << "input initial state: " << endl;
        cin >> state;
        cout << "Sequence of actions and states in the format: previous_state-action_taken->new_state" << endl;
        //cout << state << "->";
        if(state == DESTINATION) {
            cout << "cheese!" << endl;
            break;
        }
        int i = 0;
        while(i++ < TRANSITIONS_TO_PRINT) { //make TRANSITIONS_TO_PRINT number of actions or until goal state found
            int action = max_q_action(state);
            cout << state << "-";
            state = update_state(state, action);
            cout << action << "->" << state << endl;
            if(state == DESTINATION) { //if at goal state stop loop
                cout << "cheese!" << endl;
                break;
            }
            //cout << action << "->" << state << "::";
        }
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

void update_track_csv(vector<Segment> track) {
    ofstream out_file("track.csv");
    for(auto segment : track) {
        out_file << segment.p1.first << "," << segment.p1.second << "," << segment.p2.first 
            << "," << segment.p2.second << endl;
    }
}

void update_senseg_csv(pair<double, double> agent, vector<Segment> sensor_segments) {
    ofstream out_file("sensor_segments.csv");
    for(auto segment : sensor_segments) {
        out_file << agent.first << "," << agent.second << "," << segment.p2.first << "," << segment.p2.second << endl;
    }

}

void run_sensors(int init_state) {
    // NB: This is the only place we should be dealing with decimals - keep casting to a minimum overall
    pair<int, int> agent = make_pair(init_state/N, init_state%N);

    //gets segments representing sensors, functions and such found in sensors.cpp and Segment.cpp
    vector<pair<double, pair<double, double>>> sensors = sensor_16(agent);
    vector<Segment> sensor_segments = get_sensor_segments(track, agent, sensors);
    for(auto segment : sensor_segments) {
        gridworld[(int)(segment.p2.first+0.5)][(int)(segment.p2.second+0.5)] = 1;
    }
    update_track_csv(track);
    update_senseg_csv(agent, sensor_segments);
}

int main() {
    // INPUTS -------------------------
    track = track1();
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
    /* ######## Just messin ######## */
    run_sensors(N*(int)(x+0.5)+y);
    write_out_gridworld(gridworld);
    /* ############################# */
    // Do the Q -----------------------
    // run_Qlearing();
    return 0;
}
