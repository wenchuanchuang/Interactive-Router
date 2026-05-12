#include "router.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <stdexcept>
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

void markSegmentVertices(
    std::unordered_set<PackedVertexId>& occupied,
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
            const auto [px, py] = gridToMm(request, x, y);
            if (distancePointToSegment(px, py, sx, sy, ex, ey) <= r + 1e-9) {
                occupied.insert(packVertex(x, y, z));
            }
        }
    }
}

void markCircleVertices(
    std::unordered_set<PackedVertexId>& occupied,
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
            const auto [px, py] = gridToMm(request, x, y);
            const double dx = px - cx;
            const double dy = py - cy;
            if (dx * dx + dy * dy <= rr) {
                occupied.insert(packVertex(x, y, z));
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

std::vector<std::vector<GridPoint>> buildPadGroupsInternal(const RasterRequest& request) {
    std::vector<std::vector<GridPoint>> groups;
    groups.reserve(request.pads.size());
    static const int neighbors[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };
    for (const auto& pad : request.pads) {
        std::unordered_set<PackedVertexId> inside;
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
                        inside.insert(packVertex(x, y, z));
                    }
                }
            }
        }

        std::unordered_set<PackedVertexId> boundary;
        for (PackedVertexId packed : inside) {
            const GridPoint p = unpackVertex(packed);
            bool is_boundary = false;
            for (const auto& delta : neighbors) {
                const int nx = p.x + delta[0];
                const int ny = p.y + delta[1];
                if (nx < 0 || nx >= request.nx || ny < 0 || ny >= request.ny ||
                    inside.find(packVertex(nx, ny, p.z)) == inside.end()) {
                    is_boundary = true;
                    break;
                }
            }
            if (is_boundary) {
                boundary.insert(packed);
            }
        }
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

}  // namespace

RasterResult rasterizeSelectorGeometry(const RasterRequest& request) {
    RasterResult result;
    if (request.grid_pitch <= 0.0 || request.nx <= 0 || request.ny <= 0 || request.nz <= 0) {
        return result;
    }

    std::unordered_set<PackedVertexId> occupied;
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
    result.occupied_vertices = sortedGridPoints(occupied);

    static const int neighbors[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };
    std::unordered_set<PackedVertexId> shell;
    for (PackedVertexId packed : occupied) {
        const GridPoint p = unpackVertex(packed);
        for (const auto& delta : neighbors) {
            const int nx = p.x + delta[0];
            const int ny = p.y + delta[1];
            if (nx < 0 || nx >= request.nx || ny < 0 || ny >= request.ny ||
                occupied.find(packVertex(nx, ny, p.z)) == occupied.end()) {
                shell.insert(packed);
                break;
            }
        }
    }

    std::unordered_set<PackedVertexId> anchors;
    for (const auto& anchor : request.anchor_vertices) {
        anchors.insert(packVertex(anchor.x, anchor.y, anchor.z));
    }

    std::unordered_set<PackedVertexId> filtered;
    for (PackedVertexId packed : shell) {
        if (anchors.find(packed) != anchors.end()) {
            filtered.insert(packed);
            continue;
        }
        bool inside_any_pad = false;
        for (const auto& pad : request.pads) {
            if (vertexInsidePad(request, packed, pad, request.pad_clearance)) {
                inside_any_pad = true;
                break;
            }
        }
        if (!inside_any_pad) {
            filtered.insert(packed);
        }
    }
    for (PackedVertexId anchor : anchors) {
        filtered.insert(anchor);
    }
    result.boundary_vertices = sortedGridPoints(filtered);
    result.pad_boundary_groups = buildPadGroupsInternal(request);
    return result;
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

    std::vector<PackedVertexId> vertices;
    vertices.reserve(graph.size());
    for (const auto& entry : graph) {
        vertices.push_back(entry.first);
    }

    result.total_pads = static_cast<int>(request.pads.size());
    for (const auto& pad : request.pads) {
        std::set<int> matched_components;
        for (PackedVertexId vertex : vertices) {
            if (!vertexInsidePad(request, vertex, pad, 0.0)) {
                continue;
            }
            const auto component_it = component_by_vertex.find(vertex);
            if (component_it != component_by_vertex.end()) {
                matched_components.insert(component_it->second);
            }
        }
        if (matched_components.empty()) {
            ++result.unmatched_pads;
            continue;
        }
        result.matched_components.insert(
            result.matched_components.end(),
            matched_components.begin(),
            matched_components.end()
        );
    }

    return result;
}

}  // namespace interactive_router
