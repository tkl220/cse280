#include <iostream>
#include <string>
#include <cmath>
#include <vector>

using namespace std;

// TODO: change from point mass model to ackermann

// reward function -- 2 parts: gets you closer to goal, part that causes you to turn
// decouple the 2 parts: just turns, or just gets closer to the thing
// how: forget about rollout part, concentrate on selection of a good trajectory
//      have our line (path), then see that area around the path -- put random poses around it, score each one
//      see what it looks like
//      - visualizing reward function using random points (ex: (x,y,theta)) do it for a ton of randomly selected points around the path

// problem: not enough visualization of the numbers -> blind of how things are performing


// random points around the track: score them all and see how they score

// decouple scoring: first based on closeness to goal, second based on orientation
// see these results on how they score random points, then make an equation

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

bool sortbysec(const pair<int,int> &a, const pair<int,int> &b) { 
    return (a.second < b.second);
} 

/*
* method to score trajectories received from genTraj() for the car and return the best position
* which the car will move towards
*   finds the single position that is the best position for the car to go to,
*   calculates how far it can go in 0.1 second and returns this new position
*
* Method: find the half of the points closest to the path (perpendicular distance)
*         then, of those half points, find which aligns best with the angle of the path
*       
*         weights scaled proportionally for angle and distance
*
*/
statePosTraj scoreTraj(std::vector<statePosTraj> &posTraj, path &thePath, position &goal) {
    // the best score, aka lowest dist and best angle to path
    // combine in a linear eqn 
    // weight times first error + weight for angle term * angle error
    //  make up the weights, trial and error
    // .1, 1, 10
    // figure out automatically --> try out all diff combos, best wins

    std::vector<float> distances;
    distances.reserve(posTraj.size());
    float dist;

    float distFromOrigin = abs(thePath.c / sqrt(pow(thePath.a, 2) + pow(thePath.b, 2)));

    for (int i = 0; i < posTraj.size(); i++) {
        dist = abs((thePath.a * posTraj.at(i).pos.x) + (thePath.b * posTraj.at(i).pos.y) + thePath.c) / sqrt(pow(thePath.a, 2) + pow(thePath.b, 2));
        distances.push_back(dist);
    }

    // angle of the path [slope: (-a/b)]
    float angPath = abs(atan(-(thePath.a / thePath.b)));
    int index = -1;
    
    // just added this too, 
    if (posTraj.at(0).w == -5) { // check if the distance is < .3, if so move along the path
        // move straight along the path 
        // implementing this once the trajectory to the path is working 
    }

    // add: distance to the goal -> end of the line, update goal as robot moves towards it
    // promotes forward motion of the robot
    // comes in from an external source 
    // guide robot around the track
    float score = 999999999;
    for (int i = 0; i < distances.size(); i++) {
        float angDiff = abs(angPath - (90 - posTraj.at(i).pos.ang)); // angle here?
        float tempScore = (pow(distances.at(i), 2) / pow(distFromOrigin, 2)) + abs((1 - (distances.at(i) / distFromOrigin)) * angDiff / angPath);
        printf("the score is %.2f\n", tempScore);
        if (tempScore < score) {
            score = tempScore;
            index = i;
        }
    }
    
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
    //      slope: (-a/b)
    //      y-intercept: (-c/b)
    path thePath;
    thePath.a = 1;
    thePath.b = -1;
    thePath.c = 7;

    // position representing the starting goal position for the car (a point on the path, in the future)
    // goal = {x, y, angle}
    position goal = {-3.5, 3.5, abs(atan(-(thePath.a / thePath.b)))};

    do {
        // moves the car 1 second towards the path
        for (int i = 0; i < 10; i++) {
            // generate possible trajectories for the car
            std::vector<statePosTraj> posTraj = genTraj(traj, thePath);
            
            statePosTraj bestPosState = scoreTraj(posTraj, thePath, goal);
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
}

// Questions:
// - see qn about receiving state (line 36)
//      state = exact, (x,y,theta) get control from path
//      just traj rollout first
// - how to represent the path we want the robot to get to? --> y = mx + b
//      --> assume it is a straight line?
// - velocity being [0,1] why wouldn't I always use 1 for velocity?

// space infront of robot needs to be made computationally feasible