#include <string>
#include <vector>
#include <cmath>
#include <random>
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
    point ground_truth_pos;
    double x_variance;
    double y_variance;
    double xy_covariance;

public:
    Cone(point pos, int num_detections, bool discarded, double accuracy_confidence, std::string cone_type)
    {
        this->pos = pos;
        this->num_detections = num_detections;
        this->discarded = discarded;
        this->accuracy_confidence = accuracy_confidence;
        this->cone_type = cone_type;
        this->ground_truth_pos = pos;
        this->x_variance = 0;
        this->y_variance = 0;
        this->xy_covariance = 0;
    }
    Cone(std::string tag, double x, double y, double direction, double x_variance, double y_variance, double xy_covariance)
    {
        this->pos = point{x, y};
        this->num_detections = 1;
        this->discarded = false;
        this->accuracy_confidence = 1;
        this->cone_type = tag;
        this->ground_truth_pos = point{x, y};
        this->x_variance = x_variance;
        this->y_variance = y_variance;
        this->xy_covariance = xy_covariance;
    }

    point getPos() const
    {
        return pos;
    }

    std::string getConeType() const
    {
        return cone_type;
    }

    double getAccuracyConfidence() const
    {
        return accuracy_confidence;
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
    std::vector<Cone> scanArea(std::vector<Cone> cones, double range)
    {
        std::vector<Cone> cones_in_range;
        for (int i = 0; i < cones.size(); i++)
        {
            if (cones[i].getPos().x == this->pos.x && cones[i].getPos().y == this->pos.y)
            {
                continue;
            }
            double distance = this->distanceBetweenCones(cones[i]);
            if (distance < range)
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

    // a function to reload the cone position based on covariance
    void reloadPos(double varianceMultiplier)
    {
        // Scale the variances and covariance by the varianceMultiplier
        double scaled_x_variance = x_variance * varianceMultiplier;
        double scaled_y_variance = y_variance * varianceMultiplier;
        double scaled_xy_covariance = xy_covariance * varianceMultiplier;

        // Create the covariance matrix
        double cov_matrix[2][2] = {
            {scaled_x_variance, scaled_xy_covariance},
            {scaled_xy_covariance, scaled_y_variance}};

        // Compute the Cholesky decomposition of the covariance matrix
        double L[2][2]; // Lower triangular matrix
        L[0][0] = std::sqrt(cov_matrix[0][0]);
        L[0][1] = 0;
        L[1][0] = cov_matrix[1][0] / L[0][0];
        L[1][1] = std::sqrt(cov_matrix[1][1] - L[1][0] * L[1][0]);

        // Generate random samples from a standard normal distribution
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> d(0, 1);

        double z1 = d(gen);
        double z2 = d(gen);

        // Transform the samples using the Cholesky matrix
        double x_offset = L[0][0] * z1 + L[1][0] * z2;
        double y_offset = L[0][1] * z1 + L[1][1] * z2;

        // Update the position
        pos.x = ground_truth_pos.x + x_offset;
        pos.y = ground_truth_pos.y + y_offset;
    }
};
