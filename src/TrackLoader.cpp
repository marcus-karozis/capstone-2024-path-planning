#include "TrackLoader.h"
#include <iostream>
#include <fstream>
#include <sstream>

std::vector<Cone> cones;

TrackLoader::TrackLoader(const std::string& trackFile)
{
    loadTrackFile(trackFile);
}

void TrackLoader::loadTrackFile(const std::string &trackFile)
{
    std::ifstream file(trackFile);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + trackFile);
    }

    std::string line;

    // Skip the first line (header)
    if (!std::getline(file, line))
    {
        throw std::runtime_error("Failed to read header line from file: " + trackFile);
    }

    while (std::getline(file, line))
    {
        std::istringstream lineStream(line);
        std::string tag;
        double x, y, direction, x_variance, y_variance, xy_covariance;

        // Read the line in CSV format
        if (std::getline(lineStream, tag, ',') &&
            lineStream >> x && lineStream.ignore(1) && // Ignore ','
            lineStream >> y && lineStream.ignore(1) &&
            lineStream >> direction && lineStream.ignore(1) &&
            lineStream >> x_variance && lineStream.ignore(1) &&
            lineStream >> y_variance && lineStream.ignore(1) &&
            lineStream >> xy_covariance)
        {

            // Create a Cone object and add it to the vector
            cones.emplace_back(tag, x, y, direction, x_variance, y_variance, xy_covariance);
        }
        else
        {
            throw std::runtime_error("Malformed line in CSV file: " + line);
        }
    }
}