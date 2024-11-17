// utils.h
#ifndef UTILS_H
#define UTILS_H

struct point
{
    double x, y, x_variance = 0, y_variance = 0, xy_covariance = 0, ground_truth_x = this->x, ground_truth_y = this->y;
};

// Declare the distanceBetweenPoints function
double distanceBetweenPoints(point p1, point p2);

#endif // UTILS_H
