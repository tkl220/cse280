#include <iostream>
#include <string>
#include <cmath>

using namespace std;

struct position {
    double x;
    double y;
    double prevAng;
} tempPos, pos;

/**
*
* x = previous x position
* y = previous y position
* steerAng = angle from car moving straight (aka 0 degrees == car moving straight, -30 == 30 to the left, 30 == 30 to the right)
* v = velocity given to the back wheels
* deltT = the change in time we are calculating for
* oldAng = the old angle we must consider to determine the correct orientation of the car
*
* assuming constant acceleration
*/
position motion(double x, double y, double steerAng, double v, double deltT, double oldAng) {
    const double pi = 3.14159265358979323846;

    // "90 -" to make the angles how we want to input them
    double correctSteerAng = 90 - steerAng - oldAng;
    
    // get it in radians (to use cos and sin functions)
    double radians = (correctSteerAng) * pi / 180;

    // calculate the velocity components
    double x_vel = v * cos(radians);
    double y_vel = v * sin(radians);

    // calculate x and y changes in position
    double x_pos = x_vel * deltT;
    double y_pos = y_vel * deltT;

    // add previous positions, update steering angle, and return the struct
    tempPos.x = x_pos + x;
    tempPos.y = y_pos + y;
    tempPos.prevAng = steerAng + oldAng;
    return tempPos;
}

/*
* note: assumption is that the car starts at (0,0) facing in the direction of the y axis
* so, an initial steering angle of 0 degrees would cause the car to go straight along the y axis
* 
* can steer in various directions from there
*
*/
int main() {

    // initialize variables
    double deltT = 1; // change this??

    pos.x = 0;
    pos.y = 0;

    double steerAng = 0;
    double velocity = 0;
    double initialAngle = 0;
    
    int exit = 1;

    do {
        // determine steering angle
        printf("Enter the steering angle: ");
        scanf("%lf", &steerAng);

        // determine velocity
        printf("Enter the velocity: ");
        scanf("%lf", &velocity);

        // new position of the car
        pos = motion(pos.x, pos.y, steerAng, velocity, deltT, initialAngle);
        printf("Your cars new position is: (%f, %f) \n", pos.x, pos.y);
        
        // reset previous angle for next iteration
        initialAngle = pos.prevAng;

        // exit or continue the program
        printf("To exit the program, type 0, to continue type 1: \n");
        scanf("%d", &exit);
    } while(exit != 0);

}