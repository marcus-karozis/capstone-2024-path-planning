#ifndef TRACK_LOADER_H
#define TRACK_LOADER_H
#include <vector>
#include <string>
#include <stdexcept>
#include "cone.cpp"

class TrackLoader
{
public:
    std::vector<Cone> cones;

    TrackLoader(const std::string& trackFile);

private:
    void loadTrackFile(const std::string& trackFile);
};

#endif // TRACK_LOADER_H