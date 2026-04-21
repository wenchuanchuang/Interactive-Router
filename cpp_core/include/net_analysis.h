#pragma once

#include <string>

#include "geometry.h"
#include "grid_3d.h"

namespace interactive_router {

double widthForNet(const RouteRequest& request, int net_id);

double clearanceForNet(const RouteRequest& request, int net_id);

int originalRouteSegmentCount(const RouteRequest& request, int net_id);

bool layerMatchesPad(const PadGeometry& pad, const std::string& layer);

bool pointInsidePad(const PadGeometry& pad, const Point2D& point, double bloat = 0.0);

int preferredPadStartLayer(
    const RouteRequest& request,
    const PadGeometry& pad,
    int net_id,
    const Grid3D& grid
);

}  // namespace interactive_router
