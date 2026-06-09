#include "router.h"

#include "roaring_bitmap.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <map>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace interactive_router {
namespace {

using PackedVertexId = std::uint64_t;

constexpr std::uint64_t kPackedXBits = 28;
constexpr std::uint64_t kPackedYBits = 28;
constexpr std::uint64_t kPackedZBits = 8;
constexpr std::uint64_t kPackedYShift = kPackedXBits;
constexpr std::uint64_t kPackedZShift = kPackedXBits + kPackedYBits;
constexpr std::uint64_t kPackedXMask = (1ULL << kPackedXBits) - 1ULL;
constexpr std::uint64_t kPackedYMask = (1ULL << kPackedYBits) - 1ULL;
constexpr std::uint64_t kPackedZMask = (1ULL << kPackedZBits) - 1ULL;

PackedVertexId packVertex(int x, int y, int z) {
    if (x < 0 || y < 0 || z < 0) {
        throw std::overflow_error("Negative grid coordinate cannot be packed.");
    }
    return (static_cast<std::uint64_t>(z) << kPackedZShift) |
           (static_cast<std::uint64_t>(y) << kPackedYShift) |
           static_cast<std::uint64_t>(x);
}

GridPoint unpackVertex(PackedVertexId packed) {
    return GridPoint(
        static_cast<int>(packed & kPackedXMask),
        static_cast<int>((packed >> kPackedYShift) & kPackedYMask),
        static_cast<int>((packed >> kPackedZShift) & kPackedZMask)
    );
}

GridPoint mmToGrid(const RasterRequest& request, double x_mm, double y_mm, int z) {
    if (request.grid_pitch <= 0.0 || request.nx <= 0 || request.ny <= 0 || request.nz <= 0) {
        return GridPoint(0, 0, 0);
    }
    int x = static_cast<int>(std::llround((x_mm - request.origin_x) / request.grid_pitch));
    int y = static_cast<int>(std::llround((y_mm - request.origin_y) / request.grid_pitch));
    x = std::max(0, std::min(request.nx - 1, x));
    y = std::max(0, std::min(request.ny - 1, y));
    z = std::max(0, std::min(request.nz - 1, z));
    return GridPoint(x, y, z);
}

std::pair<double, double> gridToMm(const RasterRequest& request, int x, int y) {
    return {
        request.origin_x + static_cast<double>(x) * request.grid_pitch,
        request.origin_y + static_cast<double>(y) * request.grid_pitch,
    };
}

double distancePointToSegment(
    double px,
    double py,
    double x1,
    double y1,
    double x2,
    double y2
) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    if (std::abs(dx) < 1e-12 && std::abs(dy) < 1e-12) {
        const double ddx = px - x1;
        const double ddy = py - y1;
        return std::sqrt(ddx * ddx + ddy * ddy);
    }
    const double t = std::max(0.0, std::min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)));
    const double proj_x = x1 + t * dx;
    const double proj_y = y1 + t * dy;
    const double ddx = px - proj_x;
    const double ddy = py - proj_y;
    return std::sqrt(ddx * ddx + ddy * ddy);
}

double distancePointToAabb(
    double px,
    double py,
    double min_x,
    double min_y,
    double max_x,
    double max_y
) {
    // Measure how far a point lies outside an axis-aligned grid cell box.
    const double dx = std::max({min_x - px, 0.0, px - max_x});
    const double dy = std::max({min_y - py, 0.0, py - max_y});
    return std::sqrt(dx * dx + dy * dy);
}

double orientation(
    double ax,
    double ay,
    double bx,
    double by,
    double cx,
    double cy
) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool pointInsideAabb(
    double px,
    double py,
    double min_x,
    double min_y,
    double max_x,
    double max_y
) {
    constexpr double kEpsilon = 1e-12;
    return px >= min_x - kEpsilon && px <= max_x + kEpsilon &&
           py >= min_y - kEpsilon && py <= max_y + kEpsilon;
}

bool segmentsIntersect(
    double ax,
    double ay,
    double bx,
    double by,
    double cx,
    double cy,
    double dx,
    double dy
) {
    // Detect whether two line segments cross or touch, including collinear contact.
    constexpr double kEpsilon = 1e-12;
    const double o1 = orientation(ax, ay, bx, by, cx, cy);
    const double o2 = orientation(ax, ay, bx, by, dx, dy);
    const double o3 = orientation(cx, cy, dx, dy, ax, ay);
    const double o4 = orientation(cx, cy, dx, dy, bx, by);
    if (((o1 > kEpsilon && o2 < -kEpsilon) || (o1 < -kEpsilon && o2 > kEpsilon)) &&
        ((o3 > kEpsilon && o4 < -kEpsilon) || (o3 < -kEpsilon && o4 > kEpsilon))) {
        return true;
    }
    if (std::abs(o1) <= kEpsilon && pointInsideAabb(cx, cy, std::min(ax, bx), std::min(ay, by), std::max(ax, bx), std::max(ay, by))) {
        return true;
    }
    if (std::abs(o2) <= kEpsilon && pointInsideAabb(dx, dy, std::min(ax, bx), std::min(ay, by), std::max(ax, bx), std::max(ay, by))) {
        return true;
    }
    if (std::abs(o3) <= kEpsilon && pointInsideAabb(ax, ay, std::min(cx, dx), std::min(cy, dy), std::max(cx, dx), std::max(cy, dy))) {
        return true;
    }
    if (std::abs(o4) <= kEpsilon && pointInsideAabb(bx, by, std::min(cx, dx), std::min(cy, dy), std::max(cx, dx), std::max(cy, dy))) {
        return true;
    }
    return false;
}

bool segmentIntersectsAabb(
    double sx,
    double sy,
    double ex,
    double ey,
    double min_x,
    double min_y,
    double max_x,
    double max_y
) {
    // Fast exact test for the centerline crossing a grid cell box.
    if (pointInsideAabb(sx, sy, min_x, min_y, max_x, max_y) ||
        pointInsideAabb(ex, ey, min_x, min_y, max_x, max_y)) {
        return true;
    }
    return segmentsIntersect(sx, sy, ex, ey, min_x, min_y, max_x, min_y) ||
           segmentsIntersect(sx, sy, ex, ey, max_x, min_y, max_x, max_y) ||
           segmentsIntersect(sx, sy, ex, ey, max_x, max_y, min_x, max_y) ||
           segmentsIntersect(sx, sy, ex, ey, min_x, max_y, min_x, min_y);
}

bool segmentCapsuleIntersectsCell(
    const RasterRequest& request,
    int x,
    int y,
    double sx,
    double sy,
    double ex,
    double ey,
    double radius
) {
    // Treat each selector vertex as a square cell and mark it when the
    // clearance-expanded segment capsule touches any part of that cell.
    const auto [cx, cy] = gridToMm(request, x, y);
    const double half_pitch = request.grid_pitch * 0.5;
    const double min_x = cx - half_pitch;
    const double max_x = cx + half_pitch;
    const double min_y = cy - half_pitch;
    const double max_y = cy + half_pitch;
    if (segmentIntersectsAabb(sx, sy, ex, ey, min_x, min_y, max_x, max_y)) {
        return true;
    }
    const double endpoint_distance = std::min(
        distancePointToAabb(sx, sy, min_x, min_y, max_x, max_y),
        distancePointToAabb(ex, ey, min_x, min_y, max_x, max_y)
    );
    const double corner_distance = std::min({
        distancePointToSegment(min_x, min_y, sx, sy, ex, ey),
        distancePointToSegment(max_x, min_y, sx, sy, ex, ey),
        distancePointToSegment(max_x, max_y, sx, sy, ex, ey),
        distancePointToSegment(min_x, max_y, sx, sy, ex, ey),
    });
    return std::min(endpoint_distance, corner_distance) <= radius + 1e-9;
}

bool circleIntersectsCell(
    const RasterRequest& request,
    int x,
    int y,
    double circle_x,
    double circle_y,
    double radius
) {
    // Mark the cell when a clearance-expanded via circle touches its square area.
    const auto [cx, cy] = gridToMm(request, x, y);
    const double half_pitch = request.grid_pitch * 0.5;
    return distancePointToAabb(
        circle_x,
        circle_y,
        cx - half_pitch,
        cy - half_pitch,
        cx + half_pitch,
        cy + half_pitch
    ) <= radius + 1e-9;
}

void markSegmentVertices(
    Roaring64Bitmap& occupied,
    const RasterRequest& request,
    const RasterSegment& segment
) {
    const double sx = segment.start.x;
    const double sy = segment.start.y;
    const double ex = segment.end.x;
    const double ey = segment.end.y;
    const double r = segment.radius_mm;
    const int z = segment.z;
    if (z < 0 || z >= request.nz) {
        return;
    }
    const int min_x = std::max(0, static_cast<int>(std::floor((std::min(sx, ex) - r - request.origin_x) / request.grid_pitch)));
    const int max_x = std::min(request.nx - 1, static_cast<int>(std::ceil((std::max(sx, ex) + r - request.origin_x) / request.grid_pitch)));
    const int min_y = std::max(0, static_cast<int>(std::floor((std::min(sy, ey) - r - request.origin_y) / request.grid_pitch)));
    const int max_y = std::min(request.ny - 1, static_cast<int>(std::ceil((std::max(sy, ey) + r - request.origin_y) / request.grid_pitch)));
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            // Mark only grid vertices whose center point falls inside the
            // clearance-expanded trace capsule.
            const auto [px, py] = gridToMm(request, x, y);
            if (distancePointToSegment(px, py, sx, sy, ex, ey) <= r + 1e-9) {
                occupied.add(packVertex(x, y, z));
            }
        }
    }
}

void markCircleVertices(
    Roaring64Bitmap& occupied,
    const RasterRequest& request,
    double cx,
    double cy,
    int z,
    double radius
) {
    if (z < 0 || z >= request.nz) {
        return;
    }
    const int min_x = std::max(0, static_cast<int>(std::floor((cx - radius - request.origin_x) / request.grid_pitch)));
    const int max_x = std::min(request.nx - 1, static_cast<int>(std::ceil((cx + radius - request.origin_x) / request.grid_pitch)));
    const int min_y = std::max(0, static_cast<int>(std::floor((cy - radius - request.origin_y) / request.grid_pitch)));
    const int max_y = std::min(request.ny - 1, static_cast<int>(std::ceil((cy + radius - request.origin_y) / request.grid_pitch)));
    const double rr = radius * radius + 1e-9;
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            // Mark only grid vertices whose center point falls inside the
            // clearance-expanded via disk.
            const auto [px, py] = gridToMm(request, x, y);
            const double dx = px - cx;
            const double dy = py - cy;
            if (dx * dx + dy * dy <= rr) {
                occupied.add(packVertex(x, y, z));
            }
        }
    }
}

bool pointInsidePad(
    double x_mm,
    double y_mm,
    const RasterPad& pad,
    double clearance
) {
    const double cx = pad.center.x;
    const double cy = pad.center.y;
    const double angle = pad.rotation_degrees * 3.14159265358979323846 / 180.0;
    const double dx = x_mm - cx;
    const double dy = y_mm - cy;
    const double local_x = dx * std::cos(angle) + dy * std::sin(angle);
    const double local_y = -dx * std::sin(angle) + dy * std::cos(angle);
    const double half_x = pad.size_x * 0.5 + clearance;
    const double half_y = pad.size_y * 0.5 + clearance;
    if (half_x <= 0.0 || half_y <= 0.0) {
        return false;
    }
    const std::string shape = pad.shape;
    if (shape == "circle" || shape == "oval" || shape == "ellipse") {
        const double vx = local_x / half_x;
        const double vy = local_y / half_y;
        return vx * vx + vy * vy <= 1.0 + 1e-9;
    }
    return std::abs(local_x) <= half_x + 1e-9 && std::abs(local_y) <= half_y + 1e-9;
}

bool vertexInsidePad(
    const RasterRequest& request,
    PackedVertexId vertex,
    const RasterPad& pad,
    double clearance
) {
    const GridPoint grid = unpackVertex(vertex);
    if (std::find(pad.candidate_layers.begin(), pad.candidate_layers.end(), grid.z) == pad.candidate_layers.end()) {
        return false;
    }
    const auto [x_mm, y_mm] = gridToMm(request, grid.x, grid.y);
    return pointInsidePad(x_mm, y_mm, pad, clearance);
}

std::vector<GridPoint> sortedGridPoints(const std::unordered_set<PackedVertexId>& values) {
    std::vector<PackedVertexId> ordered(values.begin(), values.end());
    std::sort(ordered.begin(), ordered.end());
    std::vector<GridPoint> out;
    out.reserve(ordered.size());
    for (PackedVertexId packed : ordered) {
        out.push_back(unpackVertex(packed));
    }
    return out;
}

std::vector<GridPoint> sortedGridPoints(const Roaring64Bitmap& values) {
    std::vector<GridPoint> out;
    out.reserve(values.size());
    values.forEach([&out](PackedVertexId packed) {
        out.push_back(unpackVertex(packed));
    });
    return out;
}

std::vector<std::vector<GridPoint>> buildPadGroupsInternal(const RasterRequest& request) {
    std::vector<std::vector<GridPoint>> groups;
    groups.reserve(request.pads.size());
    static const int neighbors[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };
    for (const auto& pad : request.pads) {
        Roaring64Bitmap inside;
        const double radius = std::hypot(pad.size_x * 0.5, pad.size_y * 0.5) + request.grid_pitch * 1.5;
        const int gx0 = std::max(0, static_cast<int>(std::floor((pad.center.x - radius - request.origin_x) / request.grid_pitch)) - 1);
        const int gx1 = std::min(request.nx - 1, static_cast<int>(std::ceil((pad.center.x + radius - request.origin_x) / request.grid_pitch)) + 1);
        const int gy0 = std::max(0, static_cast<int>(std::floor((pad.center.y - radius - request.origin_y) / request.grid_pitch)) - 1);
        const int gy1 = std::min(request.ny - 1, static_cast<int>(std::ceil((pad.center.y + radius - request.origin_y) / request.grid_pitch)) + 1);
        for (int z : pad.candidate_layers) {
            if (z < 0 || z >= request.nz) {
                continue;
            }
            for (int x = gx0; x <= gx1; ++x) {
                for (int y = gy0; y <= gy1; ++y) {
                    const auto [x_mm, y_mm] = gridToMm(request, x, y);
                    if (pointInsidePad(x_mm, y_mm, pad, 0.0)) {
                        inside.add(packVertex(x, y, z));
                    }
                }
            }
        }

        Roaring64Bitmap boundary;
        inside.forEach([&](PackedVertexId packed) {
            const GridPoint p = unpackVertex(packed);
            bool is_boundary = false;
            for (const auto& delta : neighbors) {
                const int nx = p.x + delta[0];
                const int ny = p.y + delta[1];
                if (nx < 0 || nx >= request.nx || ny < 0 || ny >= request.ny ||
                    !inside.contains(packVertex(nx, ny, p.z))) {
                    is_boundary = true;
                    break;
                }
            }
            if (is_boundary) {
                boundary.add(packed);
            }
        });
        groups.push_back(sortedGridPoints(boundary));
    }
    return groups;
}

std::vector<GridPoint> rasterizeGridLine2D(const GridPoint& start, const GridPoint& end) {
    std::vector<GridPoint> points;
    int x0 = start.x;
    int y0 = start.y;
    const int x1 = end.x;
    const int y1 = end.y;
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        points.emplace_back(x0, y0, start.z);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    return points;
}

std::vector<GridPoint> expandGraphEdge(const GridPoint& start, const GridPoint& end) {
    if (start.z == end.z) {
        return rasterizeGridLine2D(start, end);
    }
    if (start.x == end.x && start.y == end.y) {
        std::vector<GridPoint> points;
        const int step = (end.z >= start.z) ? 1 : -1;
        for (int z = start.z; z != end.z + step; z += step) {
            points.emplace_back(start.x, start.y, z);
        }
        return points;
    }
    return {start, end};
}

using PackedGraph = std::map<PackedVertexId, std::unordered_set<PackedVertexId>>;

void addGraphEdge(PackedGraph& graph, PackedVertexId a, PackedVertexId b) {
    graph[a].insert(b);
    graph[b].insert(a);
}

PackedGraph buildRasterGraph(const RasterRequest& request) {
    PackedGraph graph;

    if (!request.explicit_graph_nodes.empty()) {
        std::map<int, GridPoint> vertices_by_id;
        for (const auto& node : request.explicit_graph_nodes) {
            vertices_by_id[node.id] = node.vertex;
            graph[packVertex(node.vertex.x, node.vertex.y, node.vertex.z)];
        }
        for (const auto& edge : request.explicit_graph_edges) {
            const auto from_it = vertices_by_id.find(edge.from_id);
            const auto to_it = vertices_by_id.find(edge.to_id);
            if (from_it == vertices_by_id.end() || to_it == vertices_by_id.end()) {
                continue;
            }
            const auto expanded = expandGraphEdge(from_it->second, to_it->second);
            for (std::size_t i = 1; i < expanded.size(); ++i) {
                addGraphEdge(
                    graph,
                    packVertex(expanded[i - 1].x, expanded[i - 1].y, expanded[i - 1].z),
                    packVertex(expanded[i].x, expanded[i].y, expanded[i].z)
                );
            }
            if (expanded.size() == 1) {
                graph[packVertex(expanded.front().x, expanded.front().y, expanded.front().z)];
            }
        }
        if (!graph.empty()) {
            return graph;
        }
    }

    for (const auto& segment : request.segments) {
        if (segment.z < 0 || segment.z >= request.nz) {
            continue;
        }
        const auto start = mmToGrid(request, segment.start.x, segment.start.y, segment.z);
        const auto end = mmToGrid(request, segment.end.x, segment.end.y, segment.z);
        const auto line = rasterizeGridLine2D(start, end);
        if (line.empty()) {
            continue;
        }
        graph[packVertex(line.front().x, line.front().y, line.front().z)];
        for (std::size_t i = 1; i < line.size(); ++i) {
            addGraphEdge(
                graph,
                packVertex(line[i - 1].x, line[i - 1].y, line[i - 1].z),
                packVertex(line[i].x, line[i].y, line[i].z)
            );
        }
    }

    for (const auto& via : request.vias) {
        const int z0 = std::min(via.z_start, via.z_end);
        const int z1 = std::max(via.z_start, via.z_end);
        std::vector<GridPoint> column;
        for (int z = z0; z <= z1; ++z) {
            column.push_back(mmToGrid(request, via.center.x, via.center.y, z));
        }
        if (column.empty()) {
            continue;
        }
        graph[packVertex(column.front().x, column.front().y, column.front().z)];
        for (std::size_t i = 1; i < column.size(); ++i) {
            addGraphEdge(
                graph,
                packVertex(column[i - 1].x, column[i - 1].y, column[i - 1].z),
                packVertex(column[i].x, column[i].y, column[i].z)
            );
        }
    }

    return graph;
}

void appendPointKey(std::ostringstream& out, const Point2D& point) {
    out << point.x << "," << point.y << ";";
}

void appendGridPointKey(std::ostringstream& out, const GridPoint& point) {
    out << point.x << "," << point.y << "," << point.z << ";";
}

void appendRasterRequestKey(std::ostringstream& out, const RasterRequest& request) {
    // This key is intentionally exact for selector-visible request fields.
    // It only reuses batch work when the raster and coverage inputs match bit-for-bit
    // after Python has converted them into C++ request objects.
    out << std::setprecision(17);
    out << "grid:"
        << request.grid_pitch << ","
        << request.origin_x << ","
        << request.origin_y << ","
        << request.nx << ","
        << request.ny << ","
        << request.nz << ";";
    out << "pad_clearance:" << request.pad_clearance << ";";
    out << "anchors:";
    for (const auto& anchor : request.anchor_vertices) {
        appendGridPointKey(out, anchor);
    }
    out << "segments:";
    for (const auto& segment : request.segments) {
        appendPointKey(out, segment.start);
        appendPointKey(out, segment.end);
        out << segment.radius_mm << "," << segment.z << ";";
    }
    out << "vias:";
    for (const auto& via : request.vias) {
        appendPointKey(out, via.center);
        out << via.radius_mm << "," << via.z_start << "," << via.z_end << ";";
    }
    out << "pads:";
    for (const auto& pad : request.pads) {
        appendPointKey(out, pad.center);
        out << pad.size_x << "," << pad.size_y << ","
            << pad.rotation_degrees << "," << pad.shape << ":";
        for (int layer : pad.candidate_layers) {
            out << layer << ",";
        }
        out << ";";
    }
    out << "graph_nodes:";
    for (const auto& node : request.explicit_graph_nodes) {
        out << node.id << ":";
        appendGridPointKey(out, node.vertex);
    }
    out << "graph_edges:";
    for (const auto& edge : request.explicit_graph_edges) {
        out << edge.from_id << "," << edge.to_id << ";";
    }
}

std::string rasterAnalysisPairKey(const RasterAnalysisPairRequest& request) {
    std::ostringstream out;
    out << "raster{";
    appendRasterRequestKey(out, request.raster_request);
    out << "}coverage{";
    appendRasterRequestKey(out, request.coverage_request);
    out << "}";
    return out.str();
}

}  // namespace

RasterResult rasterizeSelectorGeometry(const RasterRequest& request) {
    RasterResult result;
    if (request.grid_pitch <= 0.0 || request.nx <= 0 || request.ny <= 0 || request.nz <= 0) {
        return result;
    }

    Roaring64Bitmap occupied;
    for (const auto& segment : request.segments) {
        markSegmentVertices(occupied, request, segment);
    }
    for (const auto& via : request.vias) {
        int z0 = std::max(0, std::min(via.z_start, via.z_end));
        int z1 = std::min(request.nz - 1, std::max(via.z_start, via.z_end));
        for (int z = z0; z <= z1; ++z) {
            markCircleVertices(occupied, request, via.center.x, via.center.y, z, via.radius_mm);
        }
    }
    result.occupied_vertex_ids = occupied.values();
    result.occupied_vertices = sortedGridPoints(occupied);

    static const int neighbors[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };
    Roaring64Bitmap shell;
    occupied.forEach([&](PackedVertexId packed) {
        const GridPoint p = unpackVertex(packed);
        for (const auto& delta : neighbors) {
            const int nx = p.x + delta[0];
            const int ny = p.y + delta[1];
            if (nx < 0 || nx >= request.nx || ny < 0 || ny >= request.ny ||
                !occupied.contains(packVertex(nx, ny, p.z))) {
                shell.add(packed);
                break;
            }
        }
    });

    Roaring64Bitmap anchors;
    for (const auto& anchor : request.anchor_vertices) {
        anchors.add(packVertex(anchor.x, anchor.y, anchor.z));
    }

    Roaring64Bitmap filtered;
    shell.forEach([&](PackedVertexId packed) {
        if (anchors.contains(packed)) {
            filtered.add(packed);
        } else {
            bool inside_any_pad = false;
            for (const auto& pad : request.pads) {
                if (vertexInsidePad(request, packed, pad, request.pad_clearance)) {
                    inside_any_pad = true;
                    break;
                }
            }
            if (!inside_any_pad) {
                filtered.add(packed);
            }
        }
    });
    anchors.forEach([&](PackedVertexId anchor) {
        filtered.add(anchor);
    });
    result.boundary_vertex_ids = filtered.values();
    result.boundary_vertices = sortedGridPoints(filtered);
    result.pad_boundary_groups = buildPadGroupsInternal(request);
    return result;
}

std::vector<RasterAnalysisResult> analyzeSelectorGeometryBatch(
    const std::vector<RasterAnalysisPairRequest>& requests
) {
    std::vector<RasterAnalysisResult> results;
    results.reserve(requests.size());
    std::unordered_map<std::string, std::size_t> analysis_cache;
    analysis_cache.reserve(requests.size());

    for (std::size_t index = 0; index < requests.size(); ++index) {
        // Reuse raster and pad-coverage work only when the full selector-visible
        // request is exactly identical. This preserves every candidate identity
        // while avoiding repeated geometry analysis for duplicate candidates.
        const std::string key = rasterAnalysisPairKey(requests[index]);
        const auto cache_it = analysis_cache.find(key);
        if (cache_it != analysis_cache.end()) {
            RasterAnalysisResult cached = results[cache_it->second];
            cached.cache_hit = true;
            cached.cache_source_index = static_cast<int>(cache_it->second);
            results.push_back(std::move(cached));
            continue;
        }

        RasterAnalysisResult result;
        result.raster = rasterizeSelectorGeometry(requests[index].raster_request);
        result.coverage = analyzePadCoverage(requests[index].coverage_request);
        result.cache_hit = false;
        result.cache_source_index = static_cast<int>(index);
        analysis_cache.emplace(key, index);
        results.push_back(std::move(result));
    }

    return results;
}

std::vector<RasterGraphNode> graphNodesFromCandidate(const RasterCandidateGeometry& candidate) {
    if (!candidate.explicit_graph_nodes.empty()) {
        return candidate.explicit_graph_nodes;
    }
    const std::size_t count = std::min({
        candidate.graph_node_ids.size(),
        candidate.graph_node_x.size(),
        candidate.graph_node_y.size(),
        candidate.graph_node_z.size(),
    });
    std::vector<RasterGraphNode> nodes;
    nodes.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        RasterGraphNode node;
        node.id = candidate.graph_node_ids[index];
        node.vertex = GridPoint(
            candidate.graph_node_x[index],
            candidate.graph_node_y[index],
            candidate.graph_node_z[index]
        );
        nodes.push_back(node);
    }
    return nodes;
}

std::vector<RasterGraphEdge> graphEdgesFromCandidate(const RasterCandidateGeometry& candidate) {
    if (!candidate.explicit_graph_edges.empty()) {
        return candidate.explicit_graph_edges;
    }
    const std::size_t count = std::min(candidate.graph_edge_from.size(), candidate.graph_edge_to.size());
    std::vector<RasterGraphEdge> edges;
    edges.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        RasterGraphEdge edge;
        edge.from_id = candidate.graph_edge_from[index];
        edge.to_id = candidate.graph_edge_to[index];
        edges.push_back(edge);
    }
    return edges;
}

std::vector<RasterAnalysisResult> analyzeSelectorGeometryCandidateBatch(
    const RasterCandidateBatchRequest& request
) {
    std::vector<RasterAnalysisResult> results;
    results.reserve(request.candidates.size());
    std::unordered_map<std::string, std::size_t> analysis_cache;
    analysis_cache.reserve(request.candidates.size());

    for (std::size_t index = 0; index < request.candidates.size(); ++index) {
        const auto& candidate = request.candidates[index];
        RasterRequest raster_request = request.raster_base;
        raster_request.segments = candidate.segments;
        raster_request.vias = candidate.vias;
        raster_request.explicit_graph_nodes = graphNodesFromCandidate(candidate);
        raster_request.explicit_graph_edges = graphEdgesFromCandidate(candidate);

        RasterRequest coverage_request = request.coverage_base;
        coverage_request.segments = candidate.segments;
        coverage_request.vias = candidate.vias;
        coverage_request.explicit_graph_nodes = raster_request.explicit_graph_nodes;
        coverage_request.explicit_graph_edges = raster_request.explicit_graph_edges;

        RasterAnalysisPairRequest pair_request;
        pair_request.raster_request = raster_request;
        pair_request.coverage_request = coverage_request;
        // Keep duplicate candidates as separate choices, but share the expensive
        // raster and pad-coverage result when their exact primitive input matches.
        const std::string key = rasterAnalysisPairKey(pair_request);
        const auto cache_it = analysis_cache.find(key);
        if (cache_it != analysis_cache.end()) {
            RasterAnalysisResult cached = results[cache_it->second];
            cached.cache_hit = true;
            cached.cache_source_index = static_cast<int>(cache_it->second);
            results.push_back(std::move(cached));
            continue;
        }

        RasterAnalysisResult result;
        result.raster = rasterizeSelectorGeometry(raster_request);
        result.coverage = analyzePadCoverage(coverage_request);
        result.cache_hit = false;
        result.cache_source_index = static_cast<int>(index);
        analysis_cache.emplace(key, index);
        results.push_back(std::move(result));
    }

    return results;
}

std::vector<std::vector<GridPoint>> buildPadBoundaryGroups(const RasterRequest& request) {
    return buildPadGroupsInternal(request);
}

PadCoverageResult analyzePadCoverage(const RasterRequest& request) {
    PadCoverageResult result;
    const PackedGraph graph = buildRasterGraph(request);
    result.graph_vertex_count = static_cast<int>(graph.size());
    int edge_count = 0;
    for (const auto& entry : graph) {
        edge_count += static_cast<int>(entry.second.size());
    }
    result.graph_edge_count = edge_count / 2;
    result.has_graph = !graph.empty();
    if (graph.empty()) {
        return result;
    }

    // Label connected components in the graph first.
    // Then each pad only needs to check which components it touches.
    // This lets us detect unmatched pads and split candidates cheaply.
    std::map<PackedVertexId, int> component_by_vertex;
    int component_index = 0;
    for (const auto& entry : graph) {
        const PackedVertexId start = entry.first;
        if (component_by_vertex.find(start) != component_by_vertex.end()) {
            continue;
        }
        std::deque<PackedVertexId> queue;
        queue.push_back(start);
        component_by_vertex[start] = component_index;
        while (!queue.empty()) {
            const PackedVertexId current = queue.front();
            queue.pop_front();
            const auto graph_it = graph.find(current);
            if (graph_it == graph.end()) {
                continue;
            }
            for (PackedVertexId next : graph_it->second) {
                if (component_by_vertex.find(next) != component_by_vertex.end()) {
                    continue;
                }
                component_by_vertex[next] = component_index;
                queue.push_back(next);
            }
        }
        ++component_index;
    }

    result.total_pads = static_cast<int>(request.pads.size());
    std::vector<PackedVertexId> vertices;
    vertices.reserve(graph.size());
    for (const auto& entry : graph) {
        vertices.push_back(entry.first);
    }

    std::vector<bool> vertex_hits_pad(vertices.size(), false);
    std::vector<bool> component_has_pad(component_index, false);
    for (std::size_t pad_index = 0; pad_index < request.pads.size(); ++pad_index) {
        const auto& pad = request.pads[pad_index];
        std::set<int> matched_components;
        bool pad_matched = false;
        for (std::size_t vertex_index = 0; vertex_index < vertices.size(); ++vertex_index) {
            const PackedVertexId vertex = vertices[vertex_index];
            if (!vertexInsidePad(request, vertex, pad, 0.0)) {
                continue;
            }
            pad_matched = true;
            vertex_hits_pad[vertex_index] = true;
            const auto component_it = component_by_vertex.find(vertex);
            if (component_it != component_by_vertex.end()) {
                matched_components.insert(component_it->second);
                if (component_it->second >= 0 &&
                    component_it->second < static_cast<int>(component_has_pad.size())) {
                    component_has_pad[static_cast<std::size_t>(component_it->second)] = true;
                }
            }
        }
        if (!pad_matched || matched_components.empty()) {
            ++result.unmatched_pads;
            continue;
        }
        result.matched_pad_indices.push_back(static_cast<int>(pad_index));
        result.matched_components.insert(
            result.matched_components.end(),
            matched_components.begin(),
            matched_components.end()
        );
    }

    for (bool has_pad : component_has_pad) {
        if (!has_pad) {
            ++result.padless_components;
        }
    }

    for (std::size_t vertex_index = 0; vertex_index < vertices.size(); ++vertex_index) {
        if (vertex_hits_pad[vertex_index]) {
            continue;
        }
        const auto graph_it = graph.find(vertices[vertex_index]);
        const std::size_t degree = (graph_it == graph.end()) ? 0U : graph_it->second.size();
        if (degree <= 1U) {
            ++result.dangling_endpoints;
        }
    }

    return result;
}

}  // namespace interactive_router
