#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace interactive_router {

struct Point2D {
    Point2D() = default;
    Point2D(double x_value, double y_value) : x(x_value), y(y_value) {}
    double x = 0.0;
    double y = 0.0;
};

struct GridPoint {
    GridPoint() = default;
    GridPoint(int x_value, int y_value, int z_value) : x(x_value), y(y_value), z(z_value) {}
    int x = 0;
    int y = 0;
    int z = 0;
};

struct TrackGeometry {
    Point2D start;
    Point2D end;
    double width = 0.0;
    double clearance = 0.0;
    int net_id = 0;
    std::string layer;
};

struct PadGeometry {
    Point2D center;
    Point2D footprint_center;
    double size_x = 0.0;
    double size_y = 0.0;
    double rotation_degrees = 0.0;
    std::string shape;
    int net_id = 0;
    std::vector<std::string> layers;
};

struct ViaGeometry {
    Point2D center;
    double diameter = 0.0;
    int net_id = 0;
};

struct RouteRequest {
    std::vector<TrackGeometry> tracks;
    std::vector<PadGeometry> pads;
    std::vector<ViaGeometry> vias;
    std::vector<int> ripped_net_ids;
    std::vector<std::string> layers;
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double min_trace_width = 0.2;
    double min_clearance = 0.2;
    double generated_via_diameter = 0.6;
    double grid_steps_per_mm = 10.0;
};

struct RouteResult {
    int net_id = 0;
    bool found = false;
    std::string failure_reason;
    double grid_pitch = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<int> terminal_group_sizes;
    std::vector<GridPoint> start_vertices;
    std::vector<GridPoint> goal_vertices;
    std::vector<GridPoint> path_grid;
    std::vector<Point2D> path_mm;
    std::vector<std::vector<GridPoint>> candidate_paths_grid;
    std::vector<std::vector<Point2D>> candidate_paths_mm;
};

}  // namespace interactive_router
