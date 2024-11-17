#ifndef GRIDPLOTTER_H
#define GRIDPLOTTER_H

#include <string>
#include <vector>
#include <tuple>
#include <cairo/cairo.h>
#include "cone.cpp"
#include "conePath.cpp"
#include "path.cpp"

class GridPlotter
{
private:
    int width, height;
    double scale;
public:
    // Constructor
    GridPlotter(int width, int height, double scale);

    // Public method to plot the grid
    void plotGrid(const std::vector<Cone> &cones,
                  const ConePath &leftTrack,
                  const ConePath &rightTrack,
                  const Path &midline,
                  const std::string &outputFile);

private:
    // Private helper methods
    void drawGridLines(cairo_t *cr);
    void drawCone(cairo_t *cr, double x, double y, const std::string &type);
    void drawPath(cairo_t *cr, const std::vector<Cone> &path, double r, double g, double b);
    void drawMidline(cairo_t *cr, const std::vector<point> &midline, double r, double g, double b);

    double worldToPixelX(double x);
    double worldToPixelY(double y);
    std::tuple<double, double, double> getConeColor(const std::string &coneType);
};

#endif // GRIDPLOTTER_H
