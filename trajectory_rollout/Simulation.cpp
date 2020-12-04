#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>

using namespace std;

const float pi = 3.14159265358979323846;
float startAng = 3 * pi / 2;
float add = pi/80;
float angGoal = (3 * pi / 2) - pi/10;


// the (x,y) position of the point
struct position {
    float x;
    float y;
    float ang;
};

// r^2 = (x - x_center)^2 + (y - y_center)^2
// where r is the radius, and (x_center, y_center) is the center of the circle
struct circle {
    float r;
    float x_center;
    float y_center;
};

// the (x,y) position and other state info of a possible point
struct statePosTraj {
    position pos;
    float newAng;
    float w;
    float v;
};

// ang in radians
float getArcLength(position p1, position p2, circle theCircle) {
    // find angle between 2 positions
    float ang = atan2(p2.y - theCircle.y_center, p2.x - theCircle.x_center) - atan2(p1.y - theCircle.y_center, p1.x - theCircle.x_center);
    return ang;
}

// ang in radians
position getPointOnCircle(float ang, circle theCircle) {
    float x = theCircle.r * cos(ang) + theCircle.x_center;
    float y = theCircle.r * sin(ang) + theCircle.y_center;
    position pos = {x, y, 0};
    return pos;
}

// function for the equation of calculating a possible next position
position eqn(position &oldPos, float angle, float v, float w, float deltTime) {
    float newX = 0;
    float newY = 0;
    float newAngle = 0;
    position newPos;

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

statePosTraj scoreTrajByGoal(std::vector<statePosTraj> &posTraj, circle theCircle, position &goal, position lastPos) {
    
    int index = -1;

    // check distance to the line, if it's < some amount, make the goal even closer
    //printf("(%f, %f)\n", goal.x, goal.y);
    float dist;
    float shortestDist = 9999999;
    for (int i = 0; i < posTraj.size(); i++) {
        dist = sqrt(pow(goal.x - posTraj[i].pos.x, 2) + pow(goal.y - posTraj[i].pos.y, 2));
        if (dist < shortestDist) {
            shortestDist = dist;
            index = i;
        }
    }

    position bestPos = eqn(lastPos, posTraj.at(index).newAng, posTraj.at(index).v, posTraj.at(index).w, 0.1);

    // find arc dist between lastPos and bestPos
    float arcAng = abs(getArcLength(lastPos, bestPos, theCircle));
    angGoal -= arcAng;
    // change the goal
    goal = getPointOnCircle(angGoal, theCircle);
    //printf("(%f, %f)\n", goal.x, goal.y);

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
std::vector<statePosTraj> genTraj(std::vector<position> traj) {
    // get the current position of the car
    position currPos = traj.back();

    // vector of possible trajectories
    std::vector<statePosTraj> posTraj;

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
        //("(%.2f, %.2f)\n", theState.pos.x, theState.pos.y);
    }
    //printf("\n");
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

    std::vector<position> traj;
    traj.push_back(position()); // instead, recieve the state somehow and push_back that
    traj.at(0).x = 0;
    traj.at(0).y = 0;
    traj.at(0).ang = 0;

    int exit = 1;

    // circle path: 
    circle theCircle = {5, 0, 5};

    // position representing the starting goal position for the car (a point on the path, in the future)
    // goal = {x, y, angle}
    position goal = getPointOnCircle(angGoal, theCircle);

    int count = 0;

    // NOTE: theCircle and goal are passed in, and goal is a point on theCircle

    do {
        // moves the car 1 second towards the path
        for (int i = 0; i < 10; i++) {
            // generate possible trajectories for the car
            std::vector<statePosTraj> posTraj = genTraj(traj);
            
            statePosTraj bestPosState = scoreTrajByGoal(posTraj, theCircle, goal, traj.back());

            // calculate moving just 0.1 seconds in that direction
            position actualMove = eqn(traj.back(), bestPosState.newAng, bestPosState.v, bestPosState.w, 0.1);
            
            traj.push_back(actualMove);
            printf("Position is (%.2f, %.2f) after %.1f  second(s) \n", actualMove.x, actualMove.y, count + (i+1)*0.1);
        }
        count++;
        
        // exit or continue the program
        printf("EXIT: type 0, CONTINUE: type 1 \n");
        scanf("%d", &exit);
    } while(exit != 0);

    printf("\n\n\n The Trajectory: \n\n");

    // print the trajectory and add it to the trajectory.txt file in csv format
    std::ofstream myfile;
    myfile.open("trajectory.txt");
    for (int i = 0; i < traj.size(); i++) {
        string theString = to_string(traj[i].x) + "," + to_string(traj[i].y) + "\n";
        myfile << theString;
        printf("(%f, %f)\n", traj[i].x, traj[i].y);
    }
    myfile.close();
}