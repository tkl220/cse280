//
// Created by Thomas K Leighton on 10/29/20.
//

#ifndef UNTITLED_SENSORS_H
#define UNTITLED_SENSORS_H
#include "Segment.h"
#include <utility>
#include <vector>
using namespace std;

/**
 *
 * @param A <x, y> first coordinated of line segment
 * @param B <x, y> second coordinated of line segment
 * @param sensor representing sensor as function in standard form ax + by = c, where c = element.first,
 *        a = element.second.first, b = element.second.second
 * @return coordinate of intersection, <MAXFLOAT, MAXLOAT> if no intersection
 */
pair<double, double> get_intersection(pair<double, double> A, pair<double, double> B, pair<double, pair<double, double>> sensor);

/**
 *
 * @param track A representation of a track using Segments as the sides of the track
 * @param agent coordinated of agent
 * @param sensors representation of sensors as function in standard form ax + by = c, where c = element.first,
 *        a = element.second.first, b = element.second.second
 * @return representation of sensors until nearest side of track is reached as function in standard form ax + by = c,
 *         where c = element.first, a = element.second.first, b = element.second.second
 */
vector<Segment> get_sensor_segments(vector<Segment> track, pair<double, double> agent, vector<pair<double, pair<double, double>>> sensors);

/**
 * Example track to use
 *
 * @return A representation of a track using Segments as the sides of the track
 */
vector<Segment> track1();

/**
 * Example sensor array to use
 *
 * @param agent coordinated of agent
 * @return representation of sensors as function in standard form ax + by = c, where c = element.first,
 *         a = element.second.first, b = element.second.second
 */
vector<pair<double, pair<double, double>>> sensor_16(pair<double, double> agent);

#endif //UNTITLED_SENSORS_H
