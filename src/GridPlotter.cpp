#include "GridPlotter.h"

#include <cmath>

// GridPlotter implementation
GridPlotter::GridPlotter(int width, int height, double scale)
    : width(width), height(height), scale(scale) {}

void GridPlotter::plotGrid(const std::vector<Cone>& cones, const ConePath& leftTrack, const ConePath& rightTrack, const Path& midline, const std::string& outputFile) {
    // Create a Cairo surface and context
    cairo_surface_t* surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width, height);
    cairo_t* cr = cairo_create(surface);

    // Set background to white
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_paint(cr);

    // Draw grid lines
    drawGridLines(cr);

    // Plot cones
    for (const auto& cone : cones) {
        point pos = cone.getPos();
        drawCone(cr, pos.x, pos.y, cone.getConeType());
    }

    // Plot left track limit
    drawPath(cr, leftTrack.getPath(), 1, 0, 0); // Red for left track

    // Plot right track limit
    drawPath(cr, rightTrack.getPath(), 0, 0, 1); // Blue for right track

    // Plot midline
    drawMidline(cr, midline.getPath(), 0, 1, 0); // Green for midline

    // Save to file
    cairo_surface_write_to_png(surface, outputFile.c_str());

    // Cleanup
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
}

void GridPlotter::drawGridLines(cairo_t *cr)
{
    cairo_set_source_rgb(cr, 0.9, 0.9, 0.9); // Light gray for grid lines
    for (int x = 0; x < width; x += 50) {
        cairo_move_to(cr, x, 0);
        cairo_line_to(cr, x, height);
    }
    for (int y = 0; y < height; y += 50) {
        cairo_move_to(cr, 0, y);
        cairo_line_to(cr, width, y);
    }
    cairo_stroke(cr);
}

void GridPlotter::drawCone(cairo_t *cr, double x, double y, const std::string &type)
{
    auto [r, g, b] = getConeColor(type);
    cairo_set_source_rgb(cr, r, g, b);

    cairo_arc(cr, worldToPixelX(x), worldToPixelY(y), 5, 0, 2 * M_PI);
    cairo_fill(cr);
}

void GridPlotter::drawPath(cairo_t *cr, const std::vector<Cone> &path, double r, double g, double b)
{
    if (path.empty()) return;

    cairo_set_source_rgb(cr, r, g, b);
    auto start = path.front().getPos();
    cairo_move_to(cr, worldToPixelX(start.x), worldToPixelY(start.y));

    for (const auto& cone : path) {
        point pos = cone.getPos();
        cairo_line_to(cr, worldToPixelX(pos.x), worldToPixelY(pos.y));
    }

    cairo_stroke(cr);
}

void GridPlotter::drawMidline(cairo_t *cr, const std::vector<point> &midline, double r, double g, double b)
{
    if (midline.empty()) return;

    cairo_set_source_rgb(cr, r, g, b);
    cairo_move_to(cr, worldToPixelX(midline.front().x), worldToPixelY(midline.front().y));

    for (const auto& pt : midline) {
        cairo_line_to(cr, worldToPixelX(pt.x), worldToPixelY(pt.y));
    }

    cairo_stroke(cr);
}

double GridPlotter::worldToPixelX(double x) { return x * scale + width / 2; }
double GridPlotter::worldToPixelY(double y) { return height / 2 - y * scale; }

std::tuple<double, double, double> GridPlotter::getConeColor(const std::string &coneType)
{
    if (coneType == "yellow") return { 1, 1, 0 }; // Yellow
    if (coneType == "blue") return { 0, 0, 1 };  // Blue
    if (coneType == "orange") return { 1, 0.65, 0 }; // Orange
    return { 0, 0, 0 }; // Default to black
}