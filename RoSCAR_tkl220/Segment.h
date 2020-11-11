//
// Created by Thomas K Leighton on 10/29/20.
//

#ifndef UNTITLED_SEGMENT_H
#define UNTITLED_SEGMENT_H
#include <utility>

class Segment {

public:
    std::pair<double, double> p1;
    std::pair<double, double> p2;
    //pair<double, double> y_range;
    Segment();
    Segment(std::pair<double, double> p1, std::pair<double, double> p2);

};

#endif //UNTITLED_SEGMENT_H
