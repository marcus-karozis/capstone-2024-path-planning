#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

#include "utils.cpp"

namespace path_planner
{
    // read the contents of the track csv file
    std::vector<Cone> readTrackFile(std::string trackFile)
    {
        std::vector<Cone> cones;
        std::ifstream file(trackFile);
        std::string line;

        if (file.is_open())
        {
            while (std::getline(file, line))
            {
                std::stringstream ss(line);
                std::string tag;
                double x, y, direction, x_variance, y_variance, xy_covariance;

                std::getline(ss, tag, ',');
                ss >> x;
                ss.ignore();
                ss >> y;
                ss.ignore();
                ss >> direction;
                ss.ignore();
                ss >> x_variance;
                ss.ignore();
                ss >> y_variance;
                ss.ignore();
                ss >> xy_covariance;

                Cone cone = {tag, x, y, direction, x_variance, y_variance, xy_covariance};
                cones.push_back(cone);
            }
            file.close();
        }
        else
        {
            std::cerr << "Unable to open file: " << trackFile << std::endl;
        }

    }

    // reload cone positions based on covariance
    std::vector<Cone> reloadCones(std::vector<Cone> cones)
    {
        for (int i = 0; i < cones.size(); i++)
        {
            cones[i].reloadPos();
        }
        return cones;
    }
}