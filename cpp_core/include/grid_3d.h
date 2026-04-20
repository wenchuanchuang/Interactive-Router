#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry.h"

namespace interactive_router {

class Grid3D {
public:
    Grid3D(
        double origin_x,
        double origin_y,
        double max_x,
        double max_y,
        double pitch,
        std::vector<std::string> layers
    );

    int nx() const { return nx_; }
    int ny() const { return ny_; }
    int nz() const { return nz_; }
    double pitch() const { return pitch_; }
    double origin_x() const { return origin_x_; }
    double origin_y() const { return origin_y_; }

    bool inBounds(const GridPoint& point) const;
    std::size_t flatten(const GridPoint& point) const;
    GridPoint unflatten(std::size_t index) const;

    GridPoint physicalToGrid(const Point2D& point, int layer_index) const;
    Point2D gridToPhysical(const GridPoint& point) const;

    int layerIndex(const std::string& layer) const;
    const std::vector<std::string>& layers() const { return layers_; }

    bool isBlocked(const GridPoint& point) const;
    void setBlocked(const GridPoint& point, bool blocked = true);

    void markBoardBoundary();
    void markCircle(const Point2D& center, double radius, int layer_index);
    void markRect(const Point2D& center, double half_x, double half_y, int layer_index);
    void markSegment(const Point2D& start, const Point2D& end, double radius, int layer_index);
    void markPad(const PadGeometry& pad, double bloat);
    void clearCircle(const Point2D& center, double radius, int layer_index);
    void clearRect(const Point2D& center, double half_x, double half_y, int layer_index);
    void clearPad(const PadGeometry& pad, double clearance);

    std::vector<GridPoint> verticesInsidePad(const PadGeometry& pad, double bloat, int layer_index) const;

private:
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double pitch_ = 0.1;
    int nx_ = 0;
    int ny_ = 0;
    int nz_ = 0;
    std::vector<std::string> layers_;
    std::unordered_map<std::string, int> layer_to_index_;
    std::vector<std::uint8_t> blocked_;
};

double distancePointToSegment(Point2D point, Point2D start, Point2D end);

}  // namespace interactive_router
