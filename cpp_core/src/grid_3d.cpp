#include "grid_3d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace interactive_router {
namespace {

void fillCircle(Grid3D& grid, const Point2D& center, double radius, int layer_index, bool blocked) {
    if (layer_index < 0 || layer_index >= grid.nz()) {
        return;
    }
    int min_x = std::max(0, static_cast<int>(std::floor((center.x - radius - grid.origin_x()) / grid.pitch())));
    int max_x = std::min(grid.nx() - 1, static_cast<int>(std::ceil((center.x + radius - grid.origin_x()) / grid.pitch())));
    int min_y = std::max(0, static_cast<int>(std::floor((center.y - radius - grid.origin_y()) / grid.pitch())));
    int max_y = std::min(grid.ny() - 1, static_cast<int>(std::ceil((center.y + radius - grid.origin_y()) / grid.pitch())));
    double radius_sq = radius * radius;
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            Point2D physical = grid.gridToPhysical({x, y, layer_index});
            double dx = physical.x - center.x;
            double dy = physical.y - center.y;
            if (dx * dx + dy * dy <= radius_sq) {
                grid.setBlocked({x, y, layer_index}, blocked);
            }
        }
    }
}

void fillRect(Grid3D& grid, const Point2D& center, double half_x, double half_y, int layer_index, bool blocked) {
    if (layer_index < 0 || layer_index >= grid.nz()) {
        return;
    }
    int min_x = std::max(0, static_cast<int>(std::floor((center.x - half_x - grid.origin_x()) / grid.pitch())));
    int max_x = std::min(grid.nx() - 1, static_cast<int>(std::ceil((center.x + half_x - grid.origin_x()) / grid.pitch())));
    int min_y = std::max(0, static_cast<int>(std::floor((center.y - half_y - grid.origin_y()) / grid.pitch())));
    int max_y = std::min(grid.ny() - 1, static_cast<int>(std::ceil((center.y + half_y - grid.origin_y()) / grid.pitch())));
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            grid.setBlocked({x, y, layer_index}, blocked);
        }
    }
}

}  // namespace

Grid3D::Grid3D(
    double origin_x,
    double origin_y,
    double max_x,
    double max_y,
    double pitch,
    std::vector<std::string> layers
) : origin_x_(origin_x), origin_y_(origin_y), pitch_(pitch), layers_(std::move(layers)) {
    if (pitch_ <= 0.0) {
        throw std::invalid_argument("Grid pitch must be positive");
    }
    if (layers_.empty()) {
        layers_.push_back("F.Cu");
    }

    nx_ = std::max(2, static_cast<int>(std::ceil((max_x - origin_x_) / pitch_)) + 1);
    ny_ = std::max(2, static_cast<int>(std::ceil((max_y - origin_y_) / pitch_)) + 1);
    nz_ = static_cast<int>(layers_.size());

    for (int index = 0; index < nz_; ++index) {
        layer_to_index_[layers_[index]] = index;
    }

    blocked_.assign(static_cast<std::size_t>(nx_) * ny_ * nz_, 0);
}

bool Grid3D::inBounds(const GridPoint& point) const {
    return point.x >= 0 && point.x < nx_ && point.y >= 0 && point.y < ny_ && point.z >= 0 && point.z < nz_;
}

std::size_t Grid3D::flatten(const GridPoint& point) const {
    return (static_cast<std::size_t>(point.z) * ny_ + point.y) * nx_ + point.x;
}

GridPoint Grid3D::unflatten(std::size_t index) const {
    GridPoint point;
    point.x = static_cast<int>(index % nx_);
    index /= nx_;
    point.y = static_cast<int>(index % ny_);
    index /= ny_;
    point.z = static_cast<int>(index);
    return point;
}

GridPoint Grid3D::physicalToGrid(const Point2D& point, int layer_index) const {
    return {
        static_cast<int>(std::llround((point.x - origin_x_) / pitch_)),
        static_cast<int>(std::llround((point.y - origin_y_) / pitch_)),
        layer_index,
    };
}

Point2D Grid3D::gridToPhysical(const GridPoint& point) const {
    return {origin_x_ + point.x * pitch_, origin_y_ + point.y * pitch_};
}

int Grid3D::layerIndex(const std::string& layer) const {
    auto found = layer_to_index_.find(layer);
    if (found == layer_to_index_.end()) {
        return -1;
    }
    return found->second;
}

bool Grid3D::isBlocked(const GridPoint& point) const {
    if (!inBounds(point)) {
        return true;
    }
    return blocked_[flatten(point)] != 0;
}

void Grid3D::setBlocked(const GridPoint& point, bool blocked) {
    if (inBounds(point)) {
        blocked_[flatten(point)] = blocked ? 1 : 0;
    }
}

void Grid3D::markBoardBoundary() {
    for (int z = 0; z < nz_; ++z) {
        for (int x = 0; x < nx_; ++x) {
            setBlocked({x, 0, z});
            setBlocked({x, ny_ - 1, z});
        }
        for (int y = 0; y < ny_; ++y) {
            setBlocked({0, y, z});
            setBlocked({nx_ - 1, y, z});
        }
    }
}

void Grid3D::markCircle(const Point2D& center, double radius, int layer_index) {
    fillCircle(*this, center, radius, layer_index, true);
}

void Grid3D::markRect(const Point2D& center, double half_x, double half_y, int layer_index) {
    fillRect(*this, center, half_x, half_y, layer_index, true);
}

void Grid3D::markSegment(const Point2D& start, const Point2D& end, double radius, int layer_index) {
    if (layer_index < 0 || layer_index >= nz_) {
        return;
    }
    double min_px = std::min(start.x, end.x) - radius;
    double max_px = std::max(start.x, end.x) + radius;
    double min_py = std::min(start.y, end.y) - radius;
    double max_py = std::max(start.y, end.y) + radius;
    int min_x = std::max(0, static_cast<int>(std::floor((min_px - origin_x_) / pitch_)));
    int max_x = std::min(nx_ - 1, static_cast<int>(std::ceil((max_px - origin_x_) / pitch_)));
    int min_y = std::max(0, static_cast<int>(std::floor((min_py - origin_y_) / pitch_)));
    int max_y = std::min(ny_ - 1, static_cast<int>(std::ceil((max_py - origin_y_) / pitch_)));

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            Point2D physical = gridToPhysical({x, y, layer_index});
            if (distancePointToSegment(physical, start, end) <= radius) {
                setBlocked({x, y, layer_index});
            }
        }
    }
}

void Grid3D::clearCircle(const Point2D& center, double radius, int layer_index) {
    fillCircle(*this, center, radius, layer_index, false);
}

void Grid3D::clearRect(const Point2D& center, double half_x, double half_y, int layer_index) {
    fillRect(*this, center, half_x, half_y, layer_index, false);
}

std::vector<GridPoint> Grid3D::verticesInsidePad(const PadGeometry& pad, double bloat, int layer_index) const {
    std::vector<GridPoint> vertices;
    if (layer_index < 0 || layer_index >= nz_) {
        return vertices;
    }

    double half_x = pad.size_x * 0.5 + bloat;
    double half_y = pad.size_y * 0.5 + bloat;
    int min_x = std::max(0, static_cast<int>(std::floor((pad.center.x - half_x - origin_x_) / pitch_)));
    int max_x = std::min(nx_ - 1, static_cast<int>(std::ceil((pad.center.x + half_x - origin_x_) / pitch_)));
    int min_y = std::max(0, static_cast<int>(std::floor((pad.center.y - half_y - origin_y_) / pitch_)));
    int max_y = std::min(ny_ - 1, static_cast<int>(std::ceil((pad.center.y + half_y - origin_y_) / pitch_)));

    bool circular = pad.shape == "circle" || pad.shape == "oval";
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            Point2D physical = gridToPhysical({x, y, layer_index});
            bool inside = false;
            if (circular) {
                double rx = half_x > 0.0 ? (physical.x - pad.center.x) / half_x : 0.0;
                double ry = half_y > 0.0 ? (physical.y - pad.center.y) / half_y : 0.0;
                inside = rx * rx + ry * ry <= 1.0;
            } else {
                inside = std::abs(physical.x - pad.center.x) <= half_x && std::abs(physical.y - pad.center.y) <= half_y;
            }
            if (inside) {
                vertices.push_back({x, y, layer_index});
            }
        }
    }
    return vertices;
}

double distancePointToSegment(Point2D point, Point2D start, Point2D end) {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double length_sq = dx * dx + dy * dy;
    if (length_sq <= std::numeric_limits<double>::epsilon()) {
        double px = point.x - start.x;
        double py = point.y - start.y;
        return std::sqrt(px * px + py * py);
    }
    double t = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_sq;
    t = std::clamp(t, 0.0, 1.0);
    double closest_x = start.x + t * dx;
    double closest_y = start.y + t * dy;
    double px = point.x - closest_x;
    double py = point.y - closest_y;
    return std::sqrt(px * px + py * py);
}

}  // namespace interactive_router
