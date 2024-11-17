#include <string>
#include <vector>
#include <cmath>
#include "utils.h"
#pragma once

using namespace std;

class Cone
{
private:
    point pos;
    int num_detections;
    bool discarded;
    double accuracy_confidence;
    std::string cone_type;

public:
    Cone(point pos, int num_detections, bool discarded, double accuracy_confidence, std::string cone_type)
    {
        this->pos = pos;
        this->num_detections = num_detections;
        this->discarded = discarded;
        this->accuracy_confidence = accuracy_confidence;
        this->cone_type = cone_type;
    }
    Cone(std::string tag, double x, double y, double direction, double x_variance, double y_variance, double xy_covariance)
    {
        this->pos = point{x, y};
        this->num_detections = 1;
        this->discarded = false;
        this->accuracy_confidence = 1;
        this->cone_type = tag;
    }

    point getPos() const
    {
        return pos;
    }

    std::string getConeType() const
    {
        return cone_type;
    }

    // return the distance between two cones as a double
    double distanceBetweenCones(const Cone &cone1) const
    {
        const Cone &cone2 = *this;
        double dx = cone2.getPos().x - cone1.getPos().x;
        double dy = cone2.getPos().y - cone1.getPos().y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // return the angle between two cones in degrees
    double angleBetweenCones(const Cone &cone1) const
    {
        const Cone &cone2 = *this;
        double dx = cone2.getPos().x - cone1.getPos().x;
        double dy = cone2.getPos().y - cone1.getPos().y;
        return std::atan2(dy, dx) * 180 / M_PI;
    }

    // return the angle between a point and a cone in degrees
    double angleBetweenPointAndCone(point p) const
    {
        const Cone &cone = *this;
        double dx = cone.getPos().x - p.x;
        double dy = cone.getPos().y - p.y;
        return std::atan2(dy, dx) * 180 / M_PI;
    }

    // return the distance between a point and a cone
    double distanceToCone(point p) const
    {
        const Cone &cone = *this;
        double dx = cone.getPos().x - p.x;
        double dy = cone.getPos().y - p.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void weightedConePosUpdate(point new_pos)
    {
        pos.x = (pos.x * num_detections + new_pos.x) / (num_detections + 1);
        pos.y = (pos.y * num_detections + new_pos.y) / (num_detections + 1);
        num_detections++;
    }
    void evaluatePosValidity(Cone prev_cone)
    {
        // Check if cone falls within predicted future cone positions
        // based on track rules
        double distance = this->distanceBetweenCones(prev_cone);
        double angle = this->angleBetweenCones(prev_cone);
    }

    // a function to scans area in front of cone based on parameters to find subsequent cones
    std::vector<Cone> scanArea(std::vector<Cone> cones, double range, double arc)
    {
        std::vector<Cone> cones_in_range;
        for (int i = 0; i < cones.size(); i++)
        {
            if (cones[i].getPos().x == this->pos.x && cones[i].getPos().y == this->pos.y)
            {
                continue;
            }
            double distance = this->distanceBetweenCones(cones[i]);
            double angle = this->angleBetweenCones(cones[i]);
            if (distance < range && angle < arc)
            {
                cones_in_range.push_back(cones[i]);
            }
        }
        return cones_in_range;
    }

    // a function to convert cone to a printable string
    std::string toString() const
    {
        return "Cone: " + std::to_string(pos.x) + ", " + std::to_string(pos.y) + ", " + cone_type;
    }
};
