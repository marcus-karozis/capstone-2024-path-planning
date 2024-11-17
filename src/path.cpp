#include <vector>
#include <iostream>
#include <cmath>
#include "utils.h"

#pragma once

// A Class for storing a path of points to be used by the midline vector, and raceline vector
class Path
{
private:
    std::vector<point> path;

public:
    Path()
    {
        path = std::vector<point>();
    }
    Path(std::vector<point> path)
    {
        this->path = path;
    }
    Path(point p)
    {
        path = std::vector<point>();
        path.push_back(p);
    }

    std::vector<point> getPath() const
    {
        return path;
    }

    void addPoint(point p)
    {
        path.push_back(p);
    }

    void removePoint(point p)
    {
        for (int i = 0; i < path.size(); i++)
        {
            if (path[i].x == p.x && path[i].y == p.y)
            {
                path.erase(path.begin() + i);
                break;
            }
        }
    }

    void display()
    {
        for (int i = 0; i < path.size(); i++)
        {
            std::cout << "(" << path[i].x << ", " << path[i].y << ") ";
        }
        std::cout << std::endl;
    }

    // checks to see if the last point in the path is close to the first point in the path based on Closure MOE
    void loop_closure(double closure_MOE)
    {
        if (distanceBetweenPoints(path[0], path[path.size() - 1]) < closure_MOE)
        {
            removePoint(path.back());
            path.push_back(path[0]);
        }
    }
};
