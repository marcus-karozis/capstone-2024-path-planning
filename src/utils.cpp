// utils.cpp
#include "utils.h"
#include <cmath>

double distanceBetweenPoints(point p1, point p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}
