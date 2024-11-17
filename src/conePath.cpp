#include <vector>
#include <iostream>
#include "cone.cpp"
#pragma once

// A Class for storing a path of cones to be used by the track limit vectors
class ConePath
{
private:
    std::vector<Cone> path;

public:
    ConePath()
    {
        path = std::vector<Cone>();
    }
    ConePath(std::vector<Cone> path)
    {
        this->path = path;
    }
    ConePath(Cone c)
    {
        path = std::vector<Cone>();
        path.push_back(c);
    }

    void addCone(Cone c)
    {
        path.push_back(c);
    }

    void removeCone(Cone c)
    {
        for (int i = 0; i < path.size(); i++)
        {
            if (path[i].getPos().x == c.getPos().x && path[i].getPos().y == c.getPos().y)
            {
                path.erase(path.begin() + i);
                break;
            }
        }
    }

    // search to see if the vector contains a cone
    bool containsCone(Cone c)
    {
        for (int i = 0; i < path.size(); i++)
        {
            if (path[i].getPos().x == c.getPos().x && path[i].getPos().y == c.getPos().y)
            {
                return true;
            }
        }
        return false;
    }

    void display()
    {
        for (int i = 0; i < path.size(); i++)
        {
            std::cout << "(" << path[i].getPos().x << ", " << path[i].getPos().y << ") ";
        }
        std::cout << std::endl;
    }

    std::vector<Cone> getPath() const
    {
        return path;
    }

    int coneIndex(Cone c)
    {
        for (int i = 0; i < path.size(); i++)
        {
            if (path[i].getPos().x == c.getPos().x && path[i].getPos().y == c.getPos().y)
            {
                return i;
            }
        }
        return -1;
    }

    Cone getCone(int index)
    {
        if (index < 0 || index >= path.size())
        {
            std::cout << "ConePath getCone() Index out of bounds" << std::endl;
        }
        return path[index];
    }

    int size() const
    {
        return path.size();
    }

    // checks to see if the last point in the path is close to the first point in the path based on Closure MOE
    void loop_closure(double closure_MOE)
    {
        if (path[0].distanceBetweenCones(path[path.size() - 1]) < closure_MOE)
        {
            removeCone(path.back());
            path.push_back(path[0]);
        }
    }
};