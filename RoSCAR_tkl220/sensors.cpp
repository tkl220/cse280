//
// Created by Thomas K Leighton on 10/29/20.
//

#include <math.h>
#include <iostream>
#include "sensors.h"

/**
 *
 * @param A <x, y> first coordinated of line segment
 * @param B <x, y> second coordinated of line segment
 * @param sensor representing sensor as function in standard form ax + by = c, where c = element.first,
 *        a = element.second.first, b = element.second.second
 * @return coordinate of intersection, <MAXFLOAT, MAXLOAT> if no intersection
 */
pair<double, double> get_intersection(pair<double, double> A, pair<double, double> B, pair<double, pair<double, double>> sensor) {

    //cout << "A, B: (" << A.first << ", " << A.second << "), ("<< B.first << ", " << B.second << ")" << endl;
    // side of track represented as a1x + b1y = c1
    double a1 = B.second - A.second;
    double b1 = A.first - B.first;
    double c1 = a1*(A.first) + b1*(A.second);
    //cout << "a1, b1, c1: " << a1 << ", " << b1 << ", "<< c1 << endl;

    // sensor represented as a2x + b2y = c2
    double a2 = sensor.second.first;
    double b2 = sensor.second.second;
    double c2 = sensor.first;
    //cout << "a2, b2, c2: " << a2 << ", " << b2 << ", "<< c2 << endl;

    //difference in slope
    double determinant = a1*b2 - a2*b1;

    if (determinant == 0) { //same slope, no intersection
        return make_pair(MAXFLOAT, MAXFLOAT);
    }
    double x = (b2*c1 - b1*c2)/determinant;
    double y = (a1*c2 - a2*c1)/determinant;
    //if the point of intersection is on the segment then it is valid
    if(min(A.first, B.first) <= x && x <= max(A.first, B.first) && min(A.second, B.second) <= y && y <= max(A.second, B.second)) {
        return make_pair(x, y);
    }
    return make_pair(MAXFLOAT, MAXFLOAT);
}

/**
 * main function to get line segments which represent sensors' interaction with track
 *
 * @param track A representation of a track using Segments as the sides of the track
 * @param agent coordinated of agent
 * @param sensors representation of sensors as function in standard form ax + by = c, where c = element.first,
 *        a = element.second.first, b = element.second.second
 * @return representation of sensors until nearest side of track is reached as function in standard form ax + by = c,
 *         where c = element.first, a = element.second.first, b = element.second.second
 */
vector<Segment> get_sensor_segments(vector<Segment> track, pair<double, double> agent, vector<pair<double, pair<double, double>>> sensors) {
    vector<Segment> sensor_segments;
    double sqrt_MF_x = agent.first + sqrt(MAXFLOAT); //set to sqrt to not overflow when used in distance function
    double sqrt_MF_y = agent.second + sqrt(MAXFLOAT);
    //NOTE: There since the sensors are represented as functions they extend on either side of the agent
    //      so inorder to treat the function as two sensors(pointing  out from the agent in opposite directions)
    //      there needs to be an intersection point for either side of teh agent (pos: right or above; neg: left or below)
    pair<double, double> intersection_pos = make_pair(sqrt_MF_x, sqrt_MF_y);
    pair<double, double> intersection_neg = make_pair(sqrt_MF_x, sqrt_MF_y);
    pair<double, double> cur;
    //for each sensor check each segment of the track
    for (int i = 0; i < sensors.size(); i++) {
        for (int j = 0; j < track.size(); j++) {
            //cout << "A, B: (" << track[j].p1.first << ", " << track[j].p1.second << "), ("<< track[j].p2.first << ", " << track[j].p2.second << ")" << endl;
            cur = get_intersection(track[j].p1, track[j].p2, sensors[i]); //get intersection of track segment and sensor function
            //std::cout << "x, y: " << cur.first << ", " << cur.second << endl;

            //check if the intersection is to the (right or above) or to the (left or below)
            //if cur will not over flow when used in the distance equation and cur is closer to the agent then
            //the current intersection point update the intersection point to cur this is done to avoid
            //recording multiple intersections or recording intersections past the first side intersected with
            if (cur.first > agent.first || (cur.first == agent.first && cur.second > agent.second)) { //if to the right or above the agent
                if (cur.first <= sqrt_MF_x && cur.second <= sqrt_MF_y &&                              //if cur will not overflow in distance function
                    sqrt(pow(cur.first - agent.first, 2.0) + pow(cur.second - agent.second, 2.0)) <   //reassign the intersection point if cur is closer
                    sqrt(pow(intersection_pos.first - agent.first, 2.0) + pow(intersection_pos.second - agent.second, 2.0))) {
                    intersection_pos = cur;
                }
            } else if (cur.first < agent.first || (cur.first == agent.first && cur.second < agent.second)) { //if to the left or below the agent
                if (cur.first <= sqrt_MF_x && cur.second <= sqrt_MF_y &&
                    sqrt(pow(cur.first - agent.first, 2.0) + pow(cur.second - agent.second, 2.0)) <
                    sqrt(pow(intersection_neg.first - agent.first, 2.0) + pow(intersection_neg.second - agent.second, 2.0))) {
                    intersection_neg = cur;
                }
            } else { //if the agent is on a side...
                intersection_pos = cur;
                intersection_neg = cur;
            }
        }
        //add intersection points for sensors on either side of the agent
        sensor_segments.push_back(Segment(agent, intersection_pos));
        sensor_segments.push_back(Segment(agent, intersection_neg));
        /*
        std::cout << "agent: " << agent.first << ", " << agent.second << endl;
        std::cout << "intersection_pos, intersection_neg: (" << intersection_pos.first << ", " << intersection_pos.second << "), (" << intersection_neg.first << ", " << intersection_neg.second << ")" << endl;
        std::cout << "sensor_segments pos and neg: (" << sensor_segments[i+i].p2.first << ", " << sensor_segments[i+i].p2.second << "), (" << sensor_segments[i+i+1].p2.first << ", " << sensor_segments[i+i+1].p2.second << ")" << endl;
        */
        //reset intersection points
        intersection_pos = make_pair(sqrt_MF_x, sqrt_MF_y);
        intersection_neg = make_pair(sqrt_MF_x, sqrt_MF_y);
    }
    return sensor_segments;
}

/**
 * Example track to use
 *
 * @return A representation of a track using Segments as the sides of the track
 */
vector<Segment> track1() {
    vector<Segment> segments;
    //outer sides
    segments.push_back(Segment(make_pair(-4.0, 4.0), make_pair(-2.0, 6.0)));
    segments.push_back(Segment(make_pair(-2.0, 6.0), make_pair(2.0, 6.0)));
    segments.push_back(Segment(make_pair(2.0, 6.0), make_pair(4.0, 4.0)));
    segments.push_back(Segment(make_pair(2.0, 0.0), make_pair(4.0, 4.0)));
    segments.push_back(Segment(make_pair(-2.0, 0.0),  make_pair(2.0, 0.0)));
    segments.push_back(Segment(make_pair(-4.0, 4.0),  make_pair(-2.0, 0.0)));

    //inner sides
    segments.push_back(Segment(make_pair(-2.25, 3.75), make_pair(-1.25, 4.75)));
    segments.push_back(Segment(make_pair(-1.25, 4.75), make_pair(1.25, 4.75)));
    segments.push_back(Segment(make_pair(1.25, 4.75), make_pair(2.25, 3.75)));
    segments.push_back(Segment(make_pair(1.25, 1.75), make_pair(2.25, 3.75)));
    segments.push_back(Segment(make_pair(-1.25, 1.75), make_pair(1.25, 1.75)));
    segments.push_back(Segment(make_pair(-2.25, 3.75), make_pair(-1.25, 1.75)));

    /*
    for(int j = 0; j < segments.size(); j++) {
        cout << "A, B: (" << segments[j].p1.first << ", " << segments[j].p1.second << "), (" << segments[j].p2.first
             << ", " << segments[j].p2.second << ")" << endl;
    }*/
    return segments;
}

/**
 * Example sensor array to use
 *
 * @param agent coordinated of agent
 * @return representation of sensors as function in standard form ax + by = c, where c = element.first,
 *         a = element.second.first, b = element.second.second
 */
vector<pair<double, pair<double, double>>> sensor_16(pair<double, double> agent) {
    vector<pair<double, pair<double, double>>> sensors;
    sensors.push_back(make_pair(0 * (agent.first) + 1 * (agent.second), make_pair(0, 1)));
    sensors.push_back(make_pair(1 * (agent.first) + 0 * (agent.second), make_pair(1, 0)));
    sensors.push_back(make_pair(1 * (agent.first) + 1 * (agent.second), make_pair(1, 1)));
    sensors.push_back(make_pair(-1 * (agent.first) + 1 * (agent.second), make_pair(-1, 1)));
    sensors.push_back(make_pair(1 * (agent.first) + 2 * (agent.second), make_pair(1, 2)));
    sensors.push_back(make_pair(-1 * (agent.first) + 2 * (agent.second), make_pair(-1, 2)));
    sensors.push_back(make_pair(2 * (agent.first) + 1 * (agent.second), make_pair(2, 1)));
    sensors.push_back(make_pair(-2 * (agent.first) + 1 * (agent.second), make_pair(-2, 1)));
    return sensors;
}