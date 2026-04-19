#include "router.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_set>

namespace interactive_router {
namespace {

bool containsNet(const std::vector<int>& nets, int net_id) {
    return std::find(nets.begin(), nets.end(), net_id) != nets.end();
}

double widthForNet(const RouteRequest& request, int net_id) {
    for (const auto& track : request.tracks) {
        if (track.net_id == net_id && track.width > 0.0) {
            return track.width;
        }
    }
    return request.min_trace_width;
}

bool layerMatchesPad(const PadGeometry& pad, const std::string& layer) {
    return std::find(pad.layers.begin(), pad.layers.end(), layer) != pad.layers.end()
        || std::find(pad.layers.begin(), pad.layers.end(), "*.Cu") != pad.layers.end();
}

std::vector<GridPoint> freeVertices(std::vector<GridPoint> vertices, const Grid3D& grid) {
    vertices.erase(
        std::remove_if(
            vertices.begin(),
            vertices.end(),
            [&grid](const GridPoint& point) { return grid.isBlocked(point); }
        ),
        vertices.end()
    );
    return vertices;
}

const int kNeighborDeltas[6][3] = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
};

std::vector<GridPoint> neighbors(const Grid3D& grid, const GridPoint& point) {
    std::vector<GridPoint> result;
    result.reserve(6);
    for (const auto& delta : kNeighborDeltas) {
        GridPoint next{point.x + delta[0], point.y + delta[1], point.z + delta[2]};
        if (grid.inBounds(next) && !grid.isBlocked(next)) {
            result.push_back(next);
        }
    }
    return result;
}

bool isOtherNet(int object_net_id, int current_net_id) {
    return object_net_id != current_net_id;
}

void markPad(Grid3D& grid, const PadGeometry& pad, double bloat) {
    for (int z = 0; z < grid.nz(); ++z) {
        const auto& layer_name = grid.layers()[z];
        if (!layerMatchesPad(pad, layer_name)) {
            continue;
        }
        if (pad.shape == "circle" || pad.shape == "oval") {
            grid.markCircle(pad.center, std::max(pad.size_x, pad.size_y) * 0.5 + bloat, z);
        } else {
            grid.markRect(pad.center, pad.size_x * 0.5 + bloat, pad.size_y * 0.5 + bloat, z);
        }
    }
}

void clearPadAccess(Grid3D& grid, const PadGeometry& pad, double clearance) {
    for (int z = 0; z < grid.nz(); ++z) {
        const auto& layer_name = grid.layers()[z];
        if (!layerMatchesPad(pad, layer_name)) {
            continue;
        }
        if (pad.shape == "circle" || pad.shape == "oval") {
            grid.clearCircle(pad.center, std::max(pad.size_x, pad.size_y) * 0.5 + clearance, z);
        } else {
            grid.clearRect(pad.center, pad.size_x * 0.5 + clearance, pad.size_y * 0.5 + clearance, z);
        }
    }
}

void markVia(Grid3D& grid, const ViaGeometry& via, double bloat) {
    double radius = via.diameter * 0.5 + bloat;
    for (int z = 0; z < grid.nz(); ++z) {
        grid.markCircle(via.center, radius, z);
    }
}

void markTrack(Grid3D& grid, const TrackGeometry& track, double bloat) {
    int layer = grid.layerIndex(track.layer);
    grid.markSegment(track.start, track.end, track.width * 0.5 + bloat, layer);
}

}  // namespace

Grid3D buildObstacleGridForNet(const RouteRequest& request, int net_id, double net_width, double clearance) {
    double pitch = std::max(0.001, (request.min_trace_width + request.min_clearance) * 0.5);
    double bloat = net_width * 0.5 + clearance;
    double margin = std::max(2.0 * pitch, bloat + pitch);

    Grid3D grid(
        request.min_x - margin,
        request.min_y - margin,
        request.max_x + margin,
        request.max_y + margin,
        pitch,
        request.layers
    );
    grid.markBoardBoundary();

    // Stage 1: conservative obstacle bloat. Common obstacles include board boundary
    // and unripped tracks. Per-net obstacles include other nets' vias and pads.
    for (const auto& track : request.tracks) {
        if (track.net_id == net_id || containsNet(request.ripped_net_ids, track.net_id)) {
            continue;
        }
        markTrack(grid, track, bloat);
    }

    for (const auto& via : request.vias) {
        if (via.net_id == net_id || containsNet(request.ripped_net_ids, via.net_id)) {
            continue;
        }
        markVia(grid, via, bloat);
    }

    for (const auto& pad : request.pads) {
        if (pad.net_id == net_id) {
            continue;
        }
        markPad(grid, pad, bloat);
    }

    // Stage 2: open all grid vertices inside this net's pads plus this net's
    // clearance. This prevents the terminal area from being trapped by the
    // conservative bloat applied above.
    for (const auto& pad : request.pads) {
        if (pad.net_id == net_id) {
            clearPadAccess(grid, pad, clearance);
        }
    }

    // Stage 3: redraw other nets' actual conductors with only a one-grid guard
    // band. This keeps the access opening from erasing nearby foreign copper.
    double one_grid_guard = grid.pitch();
    for (const auto& track : request.tracks) {
        if (isOtherNet(track.net_id, net_id) && !containsNet(request.ripped_net_ids, track.net_id)) {
            markTrack(grid, track, one_grid_guard);
        }
    }
    for (const auto& via : request.vias) {
        if (isOtherNet(via.net_id, net_id) && !containsNet(request.ripped_net_ids, via.net_id)) {
            markVia(grid, via, one_grid_guard);
        }
    }
    for (const auto& pad : request.pads) {
        if (isOtherNet(pad.net_id, net_id)) {
            markPad(grid, pad, one_grid_guard);
        }
    }

    return grid;
}

std::vector<GridPoint> collectTerminalVertices(
    const Grid3D& grid,
    const RouteRequest& request,
    int net_id,
    double net_width,
    double clearance
) {
    std::vector<GridPoint> terminals;
    double pad_entry_bloat = std::max(grid.pitch() * 0.25, net_width * 0.25);
    for (const auto& pad : request.pads) {
        if (pad.net_id != net_id) {
            continue;
        }
        for (int z = 0; z < grid.nz(); ++z) {
            if (!layerMatchesPad(pad, grid.layers()[z])) {
                continue;
            }
            auto vertices = grid.verticesInsidePad(pad, pad_entry_bloat, z);
            terminals.insert(terminals.end(), vertices.begin(), vertices.end());
        }
    }
    return terminals;
}

RouteResult runDijkstraTest(const RouteRequest& request) {
    RouteResult result;
    if (request.ripped_net_ids.empty()) {
        result.failure_reason = "No ripped-up net was provided.";
        return result;
    }

    int net_id = request.ripped_net_ids.front();
    double net_width = widthForNet(request, net_id);
    double clearance = request.min_clearance;
    Grid3D grid = buildObstacleGridForNet(request, net_id, net_width, clearance);

    result.net_id = net_id;
    result.grid_pitch = grid.pitch();
    result.origin_x = grid.origin_x();
    result.origin_y = grid.origin_y();
    result.nx = grid.nx();
    result.ny = grid.ny();
    result.nz = grid.nz();

    std::vector<std::vector<GridPoint>> pad_vertex_groups;
    for (const auto& pad : request.pads) {
        if (pad.net_id != net_id) {
            continue;
        }
        std::vector<GridPoint> group;
        for (int z = 0; z < grid.nz(); ++z) {
            if (!layerMatchesPad(pad, grid.layers()[z])) {
                continue;
            }
            auto vertices = grid.verticesInsidePad(pad, std::max(grid.pitch() * 0.25, net_width * 0.25), z);
            group.insert(group.end(), vertices.begin(), vertices.end());
        }
        result.terminal_group_sizes.push_back(static_cast<int>(group.size()));
        if (!group.empty()) {
            pad_vertex_groups.push_back(group);
        }
    }

    if (pad_vertex_groups.size() < 2) {
        result.failure_reason = "Fewer than two pads have usable terminal grid vertices.";
        return result;
    }
    result.start_vertices = pad_vertex_groups[0];
    result.goal_vertices = pad_vertex_groups[1];

    struct QueueNode {
        double cost;
        std::size_t index;
        bool operator>(const QueueNode& other) const { return cost > other.cost; }
    };

    std::size_t total = static_cast<std::size_t>(grid.nx()) * grid.ny() * grid.nz();
    std::vector<double> dist(total, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> previous(total, total);
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;
    std::unordered_set<std::size_t> goal_indices;

    for (const auto& start : result.start_vertices) {
        std::size_t start_index = grid.flatten(start);
        dist[start_index] = 0.0;
        queue.push({0.0, start_index});
    }
    for (const auto& goal : result.goal_vertices) {
        goal_indices.insert(grid.flatten(goal));
    }

    std::size_t reached_goal = total;
    while (!queue.empty()) {
        QueueNode current = queue.top();
        queue.pop();
        if (current.cost != dist[current.index]) {
            continue;
        }
        if (goal_indices.find(current.index) != goal_indices.end()) {
            reached_goal = current.index;
            break;
        }
        GridPoint point = grid.unflatten(current.index);
        for (const auto& delta : kNeighborDeltas) {
            GridPoint next{point.x + delta[0], point.y + delta[1], point.z + delta[2]};
            if (!grid.inBounds(next)) {
                continue;
            }
            std::size_t next_index = grid.flatten(next);
            bool is_goal = goal_indices.find(next_index) != goal_indices.end();
            if (grid.isBlocked(next) && !is_goal) {
                continue;
            }
            double step = (next.z == point.z) ? 1.0 : 4.0;
            double next_cost = current.cost + step;
            if (next_cost < dist[next_index]) {
                dist[next_index] = next_cost;
                previous[next_index] = current.index;
                queue.push({next_cost, next_index});
            }
        }
    }

    if (reached_goal == total || !std::isfinite(dist[reached_goal])) {
        result.failure_reason = "All available routes between the selected terminal vertices are blocked.";
        return result;
    }

    result.found = true;
    result.failure_reason.clear();
    for (std::size_t at = reached_goal; at != total; at = previous[at]) {
        GridPoint point = grid.unflatten(at);
        result.path_grid.push_back(point);
        result.path_mm.push_back(grid.gridToPhysical(point));
        if (dist[at] == 0.0) {
            break;
        }
    }
    std::reverse(result.path_grid.begin(), result.path_grid.end());
    std::reverse(result.path_mm.begin(), result.path_mm.end());
    return result;
}

}  // namespace interactive_router
