#pragma once

#include <vector>

#include "geometry.h"
#include "grid_3d.h"

namespace interactive_router {

RouteResult runDijkstraTest(const RouteRequest& request);

Grid3D buildObstacleGridForNet(const RouteRequest& request, int net_id, double net_width, double clearance);

std::vector<GridPoint> collectTerminalVertices(
    const Grid3D& grid,
    const RouteRequest& request,
    int net_id,
    double net_width,
    double clearance
);

}  // namespace interactive_router
