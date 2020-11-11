//
// Created by Thomas K Leighton on 10/29/20.
//

#include "Segment.h"

Segment::Segment() {
    p1 = std::make_pair(0, 0);
    p2 = std::make_pair(0, 0);
    //x_range = make_pair(0, 0);
}
Segment::Segment(std::pair<double, double> p1, std::pair<double, double> p2) {
    this->p1 = p1;
    this->p2 = p2;
    //this->x_range = make_pair(0, 0);
}