#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>

using namespace std;

// the (x,y) position of the point
struct position {
    float x;
    float y;
    float ang;
};

// the (x,y) position and other state info of a possible point
struct statePosTraj {
    position pos;
    float newAng;
    float w;
    float v;
};

// Ax + By + C = 0;
//
// slope: (-a/b)
// y-intercept: (-c/b)
//
struct path {
    float a;
    float b;
    float c;
};

// function for the equation of calculating a possible next position
position eqn(position &oldPos, float angle, float v, float w, float deltTime) {
    float newX = 0;
    float newY = 0;
    float newAngle = 0;
    position newPos;

    const float pi = 3.14159265358979323846;

    // "90 -" to make the angles how we want to input them
    float correctAng = 90 - oldPos.ang - angle;
    
    // get it in radians (to use cos and sin functions)
    float radians = (correctAng * pi) / 180;

    if (angle == 0 || w == 0) {
        // just going straight
        // calculate the velocity components
        float x_vel = v * cos(radians);
        float y_vel = v * sin(radians);

        // calculate x and y changes in position, new angle
        newX = oldPos.x + x_vel * deltTime;
        newY = oldPos.y + y_vel * deltTime;
        newAngle = angle + oldPos.ang;
        
        // add to newPos struct & return
        newPos.x = newX;
        newPos.y = newY;
        newPos.ang = newAngle;
        return newPos;
    }

    // if not going straight
    newX = oldPos.x - (v/w) * sin(radians) + (v/w) * sin(radians + w*deltTime);
    newY = oldPos.y + (v/w) * cos(radians) - (v/w) * cos(radians + w*deltTime);
    float temp = (oldPos.ang * pi) / 180;
    newAngle = temp + (w * deltTime);

    // is this how I want it for the plane doe?
    float newAngDeg = newAngle * 180 / pi;
    
    newPos.x = newX;
    newPos.y = newY;
    newPos.ang = newAngDeg;
    return newPos;
}

// sorting function used in scoring 
bool sortbysec(const pair<int,int> &a, const pair<int,int> &b) { 
    return (a.second < b.second);
} 

statePosTraj scoreTrajByGoal(std::vector<statePosTraj> &posTraj, path &thePath, position &goal) {
    
    int index = -1;

    // check distance to the line, if it's < some amount, make the goal even closer
    printf("(%f, %f)", goal.x, goal.y);
    float dist;
    float shortestDist = 9999999;
    for (int i = 0; i < posTraj.size(); i++) {
        dist = sqrt(pow(goal.x - posTraj[i].pos.x, 2) + pow(goal.y - posTraj[i].pos.y, 2));
        if (dist < shortestDist) {
            shortestDist = dist;
            index = i;
        }
    }

    float angPath = abs(atan(-(thePath.a / thePath.b)));
    printf("the angle is %f \n", angPath);

    float slope = (-thePath.a/thePath.b);
    // how to know if path tilted left or right? --> +x if right, -x if left
    if (slope > 0) {
        goal.x = goal.x + cos(angPath);
    } else {
        goal.x = goal.x - cos(angPath);
    }

    // TODO: add something to determine which y-way we wanna go?
    goal.y = goal.y + sin(angPath);

    //printf("the goal is (%f, %f) \n", goal.x, goal.y);


    printf("the chosen best path is x = %.2f, y = %.2f, ang = %.2f \n", posTraj.at(index).pos.x, posTraj.at(index).pos.y, posTraj.at(index).pos.ang);
    // return state & position at the index of the lowest score
    return posTraj.at(index);
}

/*
* method to generate trajectories for the car
* pass info in a for loop to eqn() to receive a new positions and add to array of possble trajectories
* passes raw angle to eqn() method, converted there
*
* params -> traj: vector of positions containing the path taken so far by the car
*           path: the path the car is trying to get to
* 
* returns: an array containing (x,y, ang) structs representing possible locations the car could go
*/
std::vector<statePosTraj> genTraj(std::vector<position> traj, path &thePath) {
    // get the current position of the car
    position currPos = traj.back();
    printf("traj.back is: x = %.2f, y = %.2f, ang = %.2f \n", currPos.x, currPos.y, currPos.ang);

    // vector of possible trajectories
    std::vector<statePosTraj> posTraj;
    
    float dist = abs((thePath.a * currPos.x) + (thePath.b * currPos.y) + thePath.c) / sqrt(pow(thePath.a, 2) + pow(thePath.b, 2));
    
    // just added this, to follow the path if v close, indicated by -5 (see check in scoreTraj if it == -5)
    if (dist < .3) { // ??? || currPos.x == NULL
        statePosTraj theState;
        theState.pos = currPos;
        theState.w = -5;
        theState.v = -5;
        theState.newAng = 0;
        posTraj.push_back(theState);
        return posTraj;
    }

    // center, extremes: (1, -1, -angle?), (1, 0, 0?), (1, 1, +angle?)

    float angle = 0;
    float max = 7; // can change this, but keep it odd so we always have a direct middle point (to go straight)
    float v = 1;
    float w = 0;
    int decider = ((max - 1) / 2); // determines the middle index for going straight
    float deltTime = 1; // 1 second
    for (int i = 0; i < max; i++) {
        // middle point, angle = 0
        if (i == decider) {
            angle = 0;
            w = 0;
        }
        // "negative" or "positive" angle
        else {
            // first: sections off 90 degrees into portions based on decider
            // second: multiplies by (i - decider) bc angle increases as i increases
            // if "negative" angle, (i - decider) will make it neg
            angle = (90.0 / (decider + 1)) * (i - decider); // degrees
            // compute ang velocity as a proportion
            w = (float)(i - decider) / decider; // rad/s
        }
        position posPos = eqn(currPos, angle, v, w, deltTime);
        statePosTraj theState;
        theState.pos = posPos;
        theState.w = w;
        theState.v = v;
        theState.newAng = angle;
        posTraj.push_back(theState);
        printf("(%.2f, %.2f)\n", theState.pos.x, theState.pos.y);
        //printf("the possible state: w = %.2f, v = %.2f, x = %.2f, y = %.2f, ang = %.2f \n", w, v, theState.pos.x, theState.pos.y, theState.pos.ang);
    }
    printf("\n");
    return posTraj;
}

/*
* velocity: [0, 1] m/s
* angular velocity: [-1, 1] rad/s
*
* NOTE: assumption: angle == 0 => car going straight up y axis (of car's plane)
*                   angle < 0 => car turning left, v < 1, w < 0
*                   angle > 0 => car turning right, v < 1, w > 0
*
*/
int main() {
    // receive state --> curr position, and the path
    
    // incorporate a path --> a line, slope and intercept
    //      simple geometry for intercepting lines, finding closest perpendicular
    //      cross product for direction of path

    std::vector<position> traj;
    traj.push_back(position()); // instead, recieve the state somehow and push_back that
    traj.at(0).x = 0;
    traj.at(0).y = 0;
    traj.at(0).ang = 0;

    int exit = 1;

    
    
    // for now, create a path:
    //      ax + by + c = 0;
    //      slope: (-a/b) -1
    //      y-intercept: (-c/b)
    path thePath;
    thePath.a = 1;
    thePath.b = 1;
    thePath.c = -7;

    // position representing the starting goal position for the car (a point on the path, in the future)
    // goal = {x, y, angle}
    position goal = {3.5, 3.5, abs(atan(-(thePath.a / thePath.b)))};


    // NOTE: thePath and goal are passed in, and goal is a point on thePath


    // get initial distance from origin to the line
    float distFromOrigin = abs(thePath.c / sqrt(pow(thePath.a, 2) + pow(thePath.b, 2)));

    do {
        // moves the car 1 second towards the path
        for (int i = 0; i < 10; i++) {
            // generate possible trajectories for the car
            std::vector<statePosTraj> posTraj = genTraj(traj, thePath);
            
            statePosTraj bestPosState = scoreTrajByGoal(posTraj, thePath, goal);
            //printf("the x pos is %.2f \n", bestPosState.pos.x);

            // calculate moving just 0.1 seconds in that direction
            position actualMove = eqn(traj.back(), bestPosState.newAng, bestPosState.v, bestPosState.w, 0.1);
            traj.push_back(actualMove);
            //printf("Position is (%.2f, %.2f, %.2f) after %.1f  second(s) \n", actualMove.x, actualMove.y, actualMove.ang, (i+1)*0.1);
        }
        
        // exit or continue the program
        printf("EXIT: type 0, CONTINUE: type 1 \n");
        scanf("%d", &exit);
    } while(exit != 0);
    printf("\n\n\n The Trajectory: \n\n");

    std::ofstream myfile;
    myfile.open("trajectory.txt");
    for (int i = 0; i < traj.size(); i++) {
        string theString = to_string(traj[i].x) + "," +  to_string(traj[i].y) + "\n";
        myfile << theString;
        printf("(%f, %f)\n", traj[i].x, traj[i].y);
    }
    myfile.close();
}