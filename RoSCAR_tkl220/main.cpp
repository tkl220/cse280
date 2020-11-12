#include <iostream>
#include <string>
#include <stdlib.h>
#include <random>
#include "Segment.h"
#include "sensors.h"
/*#include "/Applications/MATLAB_R2020b.app/extern/include/engine.h"
#include "/Applications/MATLAB_R2020b.app/extern/include/configure.h"
#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"
#include "mex.hpp"*/

//void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

using namespace std;
// Credit to Jin Fagang from https://github.com/jinfagang/Q-Learning
// for basis of this code.

// map relevant definitions
#define NUM_ACTIONS 4 //down, up, left, right: {-1, 0}, {1, 0}, {0, -1}, {0, 1}
#define N 3 //size of map NxN
#define MAP_SIZE 9 //total size of map, MAP_SIZE = NxN

// Q-learning relevant definitions
#define ALPHA .1 //weight of newly learned behavior
#define GAMMA .9 //weight of next action
#define EPSILON .1 //chance next action is random
#define DESTINATION 8 //state which has positive reward (cheese)
#define EPISODES 3000 //number of times to find goal

#define TRANSITIONS_TO_PRINT 10 //number of actions from state to print in traversing map using converged Q-matrix

int map7[7][7] = {{-1, -1, -1, -1, -1, -1, -1},
                 {-1,  0,-25,  0,-25,  0, -1},
                 {-100,  0,  0,  0,  0,  0, -1},
                 {-1,-200,-50,-50,  0,-25, -1},
                 {-1,-90,-90,  0,  0,  0, -1},
                 {-1,  0,  0,  0,-25,  0, -1},
                 {-1,100, -1, -1, -1, -1, -1}};

int map[3][3] = {{   0,   0,   0},
                 {-100,   0,-100},
                 {-100,   0,  50}};

double Q[MAP_SIZE][NUM_ACTIONS];
double R[MAP_SIZE][NUM_ACTIONS];

//initialize all values in Q to zero
void buildQ() {
    for(int i = 0; i < MAP_SIZE; i++) {
        for(int j = 0; j < NUM_ACTIONS; j++) {
            Q[i][j] = 0;
        }
    }
}
//Build reward table from map
//-999999 used to signify invalid action from current state
void buildR() {
    for(int i = 0; i < MAP_SIZE; i++) {
        if((i/N)-1 < 0 && (i%N)-1 < 0) { //if in top left corner
            R[i][0] = map[(i/N)+1][i%N];
            R[i][1] = -999999;
            R[i][2] = -999999;
            R[i][3] = map[i/N][(i%N)+1];
        } else if((i/N)+1 > N-1 && (i%N)-1 < 0) { //if in bottom left corner
            R[i][0] = -999999;;
            R[i][1] = map[(i/N)-1][i%N];
            R[i][2] = -999999;
            R[i][3] = map[i/N][(i%N)+1];
        } else if((i/N)-1 < 0 && (i%N)+1 > N-1) { //if in top right corner
            R[i][0] = map[(i/N)+1][i%N];
            R[i][1] = -999999;;
            R[i][2] = map[i/N][(i%N)-1];
            R[i][3] = -999999;
        } else if((i/N)+1 > N-1 && (i%N)+1 > N-1) { //if in bottom right corner
            R[i][0] = -999999;
            R[i][1] = map[(i/N)-1][i%N];
            R[i][2] = map[i/N][(i%N)-1];
            R[i][3] = -999999;
        } else if((i/N)-1 < 0) { //if on top edge
            R[i][0] = map[(i/N)+1][i%N];
            R[i][1] = -999999;
            R[i][2] = map[i/N][(i%N)-1];
            R[i][3] = map[i/N][(i%N)+1];
        } else if ((i/N)+1 > N-1) { //if on bottom edge
            R[i][0] = -999999;
            R[i][1] = map[(i/N)-1][i%N];
            R[i][2] = map[i/N][(i%N)-1];
            R[i][3] = map[i/N][(i%N)+1];
        } else if ((i%N)-1 < 0) { //if on left edge
            R[i][0] = map[(i/N)+1][i%N];
            R[i][1] = map[(i/N)-1][i%N];
            R[i][2] = -999999;
            R[i][3] = map[i/N][(i%N)+1];
        } else if ((i%N)+1 > N-1) { //if on right edge
            R[i][0] = map[(i/N)+1][i%N];
            R[i][1] = map[(i/N)-1][i%N];
            R[i][2] = map[i/N][(i%N)-1];
            R[i][3] = -999999;
        } else { //if not on any edge
            R[i][0] = map[(i / N) + 1][i % N];
            R[i][1] = map[(i / N) - 1][i % N];
            R[i][2] = map[i / N][(i % N) - 1];
            R[i][3] = map[i / N][(i % N) + 1];
        }
    }
}

/*
 * print Q-matrix
 */
void print_q() {
    for(int i = 0; i < MAP_SIZE; i++) {
        cout << i <<": {";
        for(int j = 0; j < NUM_ACTIONS - 1; ++j) {
            cout << Q[i][j] << ", ";
        }
        cout << Q[i][NUM_ACTIONS-1];
        cout << "}," << endl;
    }
}

/*
 * print Reward-matrix
 */
void print_r() {
    for(int i = 0; i < MAP_SIZE; i++) {
        cout << i <<": {";
        for(int j = 0; j < NUM_ACTIONS - 1; ++j) {
            cout << R[i][j] << ", ";
        }
        cout << R[i][NUM_ACTIONS-1];
        cout << "}," << endl;
    }
}

/*
 * Check if action is valid from given state
 */
bool check_action(int state, int action) {
    if(action == 0 && (state/N + 1) > N-1) {
        return false;
    } else if(action == 1 && (state/N - 1) < 0) {
        return false;
    } else if(action == 2 && (state%N - 1) < 0) {
        return false;
    } else if(action == 3 && (state%N + 1) > N-1) {
        return false;
    }
    return true;
}

/*
 * get max value in Q matrix from given state
 */
int max_q(int state){
    double max = -999999;
    int action = -1;
    for(int i = 0; i < NUM_ACTIONS; i++){
        //cout << "max, i, Q[state][i], check_action::: " << max << ", " << i << ", " << Q[state][i] << ", " << << ", " << endl;
        if (Q[state][i] >= max && check_action(state, i)) {
            max = Q[state][i];
            action = i;
        }
    }
    return action;
}

/*
* get max value in Q matrix from given state
*/
int update_state(int state, int action) {
    int new_state = -1;
    if(action == 0) new_state = state + N;
    else if(action == 1) new_state = state - N;
    else if(action == 2) new_state = state - 1;
    else if(action == 3) new_state = state + 1;
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
        //cout << "-- step " << step << ": initial state: " << init_state << endl;

        // get next action
        std::uniform_real_distribution<double> urd(0,1);
        std::default_random_engine re;
        double r = urd(re);
        action = rand() % NUM_ACTIONS;
        //random chance to select random action
        if (r < EPSILON) {
            //cout << "-- step " << step << ": EPSILON" << endl;
            while(!check_action(init_state, action)) action = rand() % NUM_ACTIONS;
        } else {
            //cout << "-- step " << step << ": MAX_Q" << endl;
            action = max_q(init_state);
        }
        //cout << "-- step " << step << ": action: " << action << endl;

        next_state = update_state(init_state, action);
        old_q = Q[init_state][action];
        Q[init_state][action] = old_q + ALPHA * (R[init_state][action] + GAMMA * max_q(next_state) - old_q);

        //cout << "-- step " << step << ": next state: " << next_state << endl;

        init_state = next_state;
        step++;
    }

    return init_state;
}

/*
 * Runs EPISODES number of iterations of finding the goal state to create a converged Q-matrix.
 */
void train(int init_state){
    int initial_state = init_state;

    // start random
    srand((unsigned)time(NULL));
    //srand(1);
    cout << "[INFO] start training..." << endl;
    for (int i = 0; i < EPISODES; ++i) {
        //cout << "[INFO] Episode: " << i << endl;
        episode_iterator(0);
        //cout << "-- updated Q matrix: " << endl;
        //print_q();
    }
    cout << "[INFO] end training..." << endl;
}

void run_Qlearing() {
    buildQ();
    buildR();

    cout << "Q matrix:" << endl;
    print_q();

    train(0);
    cout << "Q convergence matrix:" << endl;
    print_q();
    cout << "R matrix:" << endl;
    print_r();

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
            int action = max_q(state);
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

void run_sensors() {
    pair<double, double> agent = make_pair(-3, 4);
    //gets segments representing sensors, functions and such found in sensors.cpp and Segment.cpp
    vector<Segment> sensor_segments = get_sensor_segments(track1(), agent, sensor_16(agent));
    cout << "agent coordinates: " << agent.first << ", " << agent.second << endl;
    cout << "All segments are from agent -> intersection, since agent it always the same it is not repeated,\nonly the intersection points are displayed" << endl;
    for(auto segment : sensor_segments) {
        //minimum formatting for ease of copy and pasting into
        cout << segment.p2.first << ", " << segment.p2.second << endl;
    }
}

int main() {
    int run;
    cout << "[1] sensor simulation" << endl << "[2] Q-learning" << endl <<"choose from above to run: " << endl;
    cin >> run;
    if(run == 1) run_sensors();
    else if(run == 2) run_Qlearing();
    return 0;
}

/*
void plotlines(int x, int y) {
    unique_ptr<matlab::engine::MATLABEngine> matlabPtr = matlab::engine::startMATLAB();
    matlab::data::ArrayFactory factory;
    vector<matlab::data::Array> args({factory.createScalar<int16_t>(30), factory.createScalar<int16_t>(56) });
    matlab::data::TypedArray<int16_t> result = matlabPtr->feval(u"gcd", args);
    int16_t v = result[0];
    std::cout << "Result: " << v << std::endl;
}
*/


