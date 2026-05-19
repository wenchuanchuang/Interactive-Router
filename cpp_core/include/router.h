#pragma once

#include <vector>

#include "geometry.h"
#include "grid_3d.h"

namespace interactive_router {

RouteResult runDijkstraTest(const RouteRequest& request);
SelectionResult selectCandidatePaths(const SelectionRequest& request);

Grid3D buildObstacleGridForNet(const RouteRequest& request, int net_id, double net_width, double clearance);

std::vector<GridPoint> collectTerminalVertices(
    const Grid3D& grid,
    const RouteRequest& request,
    int net_id,
    double net_width,
    double clearance
);

RasterResult rasterizeSelectorGeometry(const RasterRequest& request);
std::vector<std::vector<GridPoint>> buildPadBoundaryGroups(const RasterRequest& request);
PadCoverageResult analyzePadCoverage(const RasterRequest& request);
std::vector<RasterAnalysisResult> analyzeSelectorGeometryBatch(
    const std::vector<RasterAnalysisPairRequest>& requests
);
std::vector<RasterAnalysisResult> analyzeSelectorGeometryCandidateBatch(
    const RasterCandidateBatchRequest& request
);

}  // namespace interactive_router
