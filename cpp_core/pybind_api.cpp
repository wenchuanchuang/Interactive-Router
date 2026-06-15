#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "include/geometry.h"
#include "include/router.h"

namespace py = pybind11;
using namespace interactive_router;

namespace {

struct RandomGridQueueItem {
    double priority = 0.0;
    double cost = 0.0;
    int state = 0;
};

struct RandomGridQueueGreater {
    bool operator()(const RandomGridQueueItem& lhs, const RandomGridQueueItem& rhs) const {
        if (lhs.priority == rhs.priority) {
            return lhs.cost > rhs.cost;
        }
        return lhs.priority > rhs.priority;
    }
};

int randomGridPackVertex(int nx, int ny, int x, int y, int z) {
    return z * nx * ny + y * nx + x;
}

void randomGridUnpackVertex(int nx, int ny, int vertex, int& x, int& y, int& z) {
    const int xy_count = nx * ny;
    z = vertex / xy_count;
    const int xy = vertex - z * xy_count;
    y = xy / nx;
    x = xy - y * nx;
}

double randomGridHeuristic(
    int nx,
    int ny,
    double pitch,
    int vertex,
    const std::vector<std::pair<int, int>>& target_grid_xy
) {
    int x = 0;
    int y = 0;
    int z = 0;
    randomGridUnpackVertex(nx, ny, vertex, x, y, z);
    double best = std::numeric_limits<double>::infinity();
    for (const auto& target : target_grid_xy) {
        const double dx = static_cast<double>(x - target.first) * pitch;
        const double dy = static_cast<double>(y - target.second) * pitch;
        best = std::min(best, std::hypot(dx, dy));
    }
    return best;
}

int randomGridDirectionDelta(int first, int second) {
    const int delta = std::abs(first - second);
    return std::min(delta, 8 - delta);
}

double randomGridUnitRandom(std::uint64_t& state) {
    // Use a tiny deterministic generator to avoid Python RNG calls inside A*.
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return static_cast<double>(state & 0xFFFFFFULL) / static_cast<double>(0x1000000ULL);
}

std::pair<int, std::vector<int>> randomGridAstar(
    int nx,
    int ny,
    int nz,
    double pitch,
    const std::vector<int>& sources,
    const std::vector<int>& target_vertices,
    const std::vector<int>& target_indices,
    const std::vector<std::pair<int, int>>& target_grid_xy,
    py::buffer trace_blocked_buffer,
    py::buffer via_blocked_buffer,
    const std::vector<int>& used_penalty_vertices,
    const std::vector<int>& bounds,
    bool randomize,
    std::uint64_t seed,
    int max_expanded
) {
    if (target_vertices.size() != target_indices.size()) {
        throw std::runtime_error("target_vertices and target_indices must have the same length");
    }
    const py::buffer_info trace_info = trace_blocked_buffer.request();
    const py::buffer_info via_info = via_blocked_buffer.request();
    const auto* trace_blocked = static_cast<const unsigned char*>(trace_info.ptr);
    const auto* via_blocked = static_cast<const unsigned char*>(via_info.ptr);
    const int xy_count = nx * ny;
    const int vertex_count = xy_count * nz;
    if (trace_info.size < vertex_count || via_info.size < xy_count) {
        throw std::runtime_error("random-grid blocked buffers are smaller than the grid");
    }

    std::unordered_map<int, int> target_by_vertex;
    target_by_vertex.reserve(target_vertices.size() * 2 + 1);
    for (std::size_t index = 0; index < target_vertices.size(); ++index) {
        target_by_vertex.emplace(target_vertices[index], target_indices[index]);
    }

    std::unordered_set<int> used_penalty;
    if (randomize) {
        used_penalty.reserve(used_penalty_vertices.size() * 2 + 1);
        for (int vertex : used_penalty_vertices) {
            used_penalty.insert(vertex);
        }
    }

    const bool has_bounds = bounds.size() == 4;
    const int min_bound_x = has_bounds ? bounds[0] : 0;
    const int min_bound_y = has_bounds ? bounds[1] : 0;
    const int max_bound_x = has_bounds ? bounds[2] : nx - 1;
    const int max_bound_y = has_bounds ? bounds[3] : ny - 1;
    auto in_bounds = [&](int x, int y) {
        return x >= 0 && x < nx && y >= 0 && y < ny
            && x >= min_bound_x && x <= max_bound_x
            && y >= min_bound_y && y <= max_bound_y;
    };

    static constexpr int dirs[8][2] = {
        {1, 0}, {1, 1}, {0, 1}, {-1, 1},
        {-1, 0}, {-1, -1}, {0, -1}, {1, -1},
    };

    std::priority_queue<RandomGridQueueItem, std::vector<RandomGridQueueItem>, RandomGridQueueGreater> heap;
    std::unordered_map<int, double> best_cost;
    std::unordered_map<int, int> came_from;
    best_cost.reserve(4096);
    came_from.reserve(4096);

    for (int source : sources) {
        int x = 0;
        int y = 0;
        int z = 0;
        randomGridUnpackVertex(nx, ny, source, x, y, z);
        if (!in_bounds(x, y) || trace_blocked[source]) {
            continue;
        }
        const int state = source * 9 + 8;
        best_cost[state] = 0.0;
        came_from[state] = -1;
        heap.push({randomGridHeuristic(nx, ny, pitch, source, target_grid_xy), 0.0, state});
    }

    std::uint64_t rng_state = seed ? seed : 1ULL;
    int expanded = 0;
    while (!heap.empty() && expanded < max_expanded) {
        const RandomGridQueueItem item = heap.top();
        heap.pop();
        const auto best_it = best_cost.find(item.state);
        if (best_it == best_cost.end() || item.cost > best_it->second + 1e-9) {
            continue;
        }
        ++expanded;

        const int vertex = item.state / 9;
        const int prev_dir = item.state % 9;
        const auto target_it = target_by_vertex.find(vertex);
        if (target_it != target_by_vertex.end()) {
            std::vector<int> path;
            int current = item.state;
            while (current >= 0) {
                path.push_back(current / 9);
                const auto parent_it = came_from.find(current);
                if (parent_it == came_from.end()) {
                    break;
                }
                current = parent_it->second;
            }
            std::reverse(path.begin(), path.end());
            return {target_it->second, path};
        }

        int x = 0;
        int y = 0;
        int z = 0;
        randomGridUnpackVertex(nx, ny, vertex, x, y, z);
        for (int dir_index = 0; dir_index < 8; ++dir_index) {
            if (prev_dir != 8 && randomGridDirectionDelta(prev_dir, dir_index) > 1) {
                continue;
            }
            const int next_x = x + dirs[dir_index][0];
            const int next_y = y + dirs[dir_index][1];
            if (!in_bounds(next_x, next_y)) {
                continue;
            }
            const int next_vertex = randomGridPackVertex(nx, ny, next_x, next_y, z);
            if (trace_blocked[next_vertex]) {
                continue;
            }
            const double step = std::hypot(static_cast<double>(dirs[dir_index][0]), static_cast<double>(dirs[dir_index][1])) * pitch;
            const double bend = (prev_dir == 8 || prev_dir == dir_index) ? 0.0 : 0.30;
            double penalty = 0.0;
            if (randomize) {
                penalty += randomGridUnitRandom(rng_state) * 0.08;
                if (used_penalty.find(next_vertex) != used_penalty.end()) {
                    penalty += 1.5;
                }
            }
            const double next_cost = item.cost + step + bend + penalty;
            const int next_state = next_vertex * 9 + dir_index;
            const auto next_best = best_cost.find(next_state);
            if (next_best == best_cost.end() || next_cost + 1e-9 < next_best->second) {
                best_cost[next_state] = next_cost;
                came_from[next_state] = item.state;
                heap.push({next_cost + randomGridHeuristic(nx, ny, pitch, next_vertex, target_grid_xy), next_cost, next_state});
            }
        }

        if (nz > 1 && !via_blocked[y * nx + x]) {
            for (int next_z = 0; next_z < nz; ++next_z) {
                if (next_z == z) {
                    continue;
                }
                const int next_vertex = randomGridPackVertex(nx, ny, x, y, next_z);
                if (trace_blocked[next_vertex]) {
                    continue;
                }
                const double via_cost = randomize ? (5.0 + randomGridUnitRandom(rng_state) * 9.0) : 8.0;
                const double next_cost = item.cost + via_cost;
                const int next_state = next_vertex * 9 + prev_dir;
                const auto next_best = best_cost.find(next_state);
                if (next_best == best_cost.end() || next_cost + 1e-9 < next_best->second) {
                    best_cost[next_state] = next_cost;
                    came_from[next_state] = item.state;
                    heap.push({next_cost + randomGridHeuristic(nx, ny, pitch, next_vertex, target_grid_xy), next_cost, next_state});
                }
            }
        }
    }
    return {-1, {}};
}

}  // namespace

PYBIND11_MODULE(router_core, m) {
    m.doc() = "Interactive Router C++ grid and Dijkstra test core";

    py::class_<Point2D>(m, "Point2D")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("x", &Point2D::x)
        .def_readwrite("y", &Point2D::y);

    py::class_<GridPoint>(m, "GridPoint")
        .def(py::init<>())
        .def(py::init<int, int, int>())
        .def_readwrite("x", &GridPoint::x)
        .def_readwrite("y", &GridPoint::y)
        .def_readwrite("z", &GridPoint::z);

    py::class_<TrackGeometry>(m, "TrackGeometry")
        .def(py::init<>())
        .def_readwrite("start", &TrackGeometry::start)
        .def_readwrite("end", &TrackGeometry::end)
        .def_readwrite("width", &TrackGeometry::width)
        .def_readwrite("clearance", &TrackGeometry::clearance)
        .def_readwrite("net_id", &TrackGeometry::net_id)
        .def_readwrite("layer", &TrackGeometry::layer);

    py::class_<PadGeometry>(m, "PadGeometry")
        .def(py::init<>())
        .def_readwrite("center", &PadGeometry::center)
        .def_readwrite("footprint_center", &PadGeometry::footprint_center)
        .def_readwrite("size_x", &PadGeometry::size_x)
        .def_readwrite("size_y", &PadGeometry::size_y)
        .def_readwrite("rotation_degrees", &PadGeometry::rotation_degrees)
        .def_readwrite("shape", &PadGeometry::shape)
        .def_readwrite("net_id", &PadGeometry::net_id)
        .def_readwrite("layers", &PadGeometry::layers);

    py::class_<ViaGeometry>(m, "ViaGeometry")
        .def(py::init<>())
        .def_readwrite("center", &ViaGeometry::center)
        .def_readwrite("diameter", &ViaGeometry::diameter)
        .def_readwrite("net_id", &ViaGeometry::net_id);

    py::class_<RasterSegment>(m, "RasterSegment")
        .def(py::init<>())
        .def_readwrite("start", &RasterSegment::start)
        .def_readwrite("end", &RasterSegment::end)
        .def_readwrite("radius_mm", &RasterSegment::radius_mm)
        .def_readwrite("z", &RasterSegment::z);

    py::class_<RasterVia>(m, "RasterVia")
        .def(py::init<>())
        .def_readwrite("center", &RasterVia::center)
        .def_readwrite("radius_mm", &RasterVia::radius_mm)
        .def_readwrite("z_start", &RasterVia::z_start)
        .def_readwrite("z_end", &RasterVia::z_end);

    py::class_<RasterPad>(m, "RasterPad")
        .def(py::init<>())
        .def_readwrite("center", &RasterPad::center)
        .def_readwrite("size_x", &RasterPad::size_x)
        .def_readwrite("size_y", &RasterPad::size_y)
        .def_readwrite("rotation_degrees", &RasterPad::rotation_degrees)
        .def_readwrite("shape", &RasterPad::shape)
        .def_readwrite("candidate_layers", &RasterPad::candidate_layers);

    py::class_<RasterGraphNode>(m, "RasterGraphNode")
        .def(py::init<>())
        .def_readwrite("id", &RasterGraphNode::id)
        .def_readwrite("vertex", &RasterGraphNode::vertex);

    py::class_<RasterGraphEdge>(m, "RasterGraphEdge")
        .def(py::init<>())
        .def_readwrite("from_id", &RasterGraphEdge::from_id)
        .def_readwrite("to_id", &RasterGraphEdge::to_id);

    py::class_<RasterRequest>(m, "RasterRequest")
        .def(py::init<>())
        .def_readwrite("grid_pitch", &RasterRequest::grid_pitch)
        .def_readwrite("origin_x", &RasterRequest::origin_x)
        .def_readwrite("origin_y", &RasterRequest::origin_y)
        .def_readwrite("nx", &RasterRequest::nx)
        .def_readwrite("ny", &RasterRequest::ny)
        .def_readwrite("nz", &RasterRequest::nz)
        .def_readwrite("segments", &RasterRequest::segments)
        .def_readwrite("vias", &RasterRequest::vias)
        .def_readwrite("pads", &RasterRequest::pads)
        .def_readwrite("pad_clearance", &RasterRequest::pad_clearance)
        .def_readwrite("anchor_vertices", &RasterRequest::anchor_vertices)
        .def_readwrite("explicit_graph_nodes", &RasterRequest::explicit_graph_nodes)
        .def_readwrite("explicit_graph_edges", &RasterRequest::explicit_graph_edges);

    py::class_<RasterResult>(m, "RasterResult")
        .def(py::init<>())
        .def_readonly("occupied_vertices", &RasterResult::occupied_vertices)
        .def_readonly("boundary_vertices", &RasterResult::boundary_vertices)
        .def_readonly("occupied_vertex_ids", &RasterResult::occupied_vertex_ids)
        .def_readonly("boundary_vertex_ids", &RasterResult::boundary_vertex_ids)
        .def_readonly("pad_boundary_groups", &RasterResult::pad_boundary_groups);

    py::class_<PadCoverageResult>(m, "PadCoverageResult")
        .def(py::init<>())
        .def_readonly("has_graph", &PadCoverageResult::has_graph)
        .def_readonly("graph_vertex_count", &PadCoverageResult::graph_vertex_count)
        .def_readonly("graph_edge_count", &PadCoverageResult::graph_edge_count)
        .def_readonly("total_pads", &PadCoverageResult::total_pads)
        .def_readonly("unmatched_pads", &PadCoverageResult::unmatched_pads)
        .def_readonly("padless_components", &PadCoverageResult::padless_components)
        .def_readonly("dangling_endpoints", &PadCoverageResult::dangling_endpoints)
        .def_readonly("matched_pad_indices", &PadCoverageResult::matched_pad_indices)
        .def_readonly("matched_components", &PadCoverageResult::matched_components);

    py::class_<RasterAnalysisPairRequest>(m, "RasterAnalysisPairRequest")
        .def(py::init<>())
        .def_readwrite("raster_request", &RasterAnalysisPairRequest::raster_request)
        .def_readwrite("coverage_request", &RasterAnalysisPairRequest::coverage_request);

    py::class_<RasterAnalysisResult>(m, "RasterAnalysisResult")
        .def(py::init<>())
        .def_readonly("raster", &RasterAnalysisResult::raster)
        .def_readonly("coverage", &RasterAnalysisResult::coverage)
        .def_readonly("cache_hit", &RasterAnalysisResult::cache_hit)
        .def_readonly("cache_source_index", &RasterAnalysisResult::cache_source_index);

    py::class_<RasterCandidateGeometry>(m, "RasterCandidateGeometry")
        .def(py::init<>())
        .def_readwrite("segments", &RasterCandidateGeometry::segments)
        .def_readwrite("vias", &RasterCandidateGeometry::vias)
        .def_readwrite("explicit_graph_nodes", &RasterCandidateGeometry::explicit_graph_nodes)
        .def_readwrite("explicit_graph_edges", &RasterCandidateGeometry::explicit_graph_edges)
        .def_readwrite("graph_node_ids", &RasterCandidateGeometry::graph_node_ids)
        .def_readwrite("graph_node_x", &RasterCandidateGeometry::graph_node_x)
        .def_readwrite("graph_node_y", &RasterCandidateGeometry::graph_node_y)
        .def_readwrite("graph_node_z", &RasterCandidateGeometry::graph_node_z)
        .def_readwrite("graph_edge_from", &RasterCandidateGeometry::graph_edge_from)
        .def_readwrite("graph_edge_to", &RasterCandidateGeometry::graph_edge_to);

    py::class_<RasterCandidateBatchRequest>(m, "RasterCandidateBatchRequest")
        .def(py::init<>())
        .def_readwrite("raster_base", &RasterCandidateBatchRequest::raster_base)
        .def_readwrite("coverage_base", &RasterCandidateBatchRequest::coverage_base)
        .def_readwrite("candidates", &RasterCandidateBatchRequest::candidates);

    py::class_<RouteRequest>(m, "RouteRequest")
        .def(py::init<>())
        .def_readwrite("tracks", &RouteRequest::tracks)
        .def_readwrite("pads", &RouteRequest::pads)
        .def_readwrite("vias", &RouteRequest::vias)
        .def_readwrite("ripped_net_ids", &RouteRequest::ripped_net_ids)
        .def_readwrite("layers", &RouteRequest::layers)
        .def_readwrite("min_x", &RouteRequest::min_x)
        .def_readwrite("min_y", &RouteRequest::min_y)
        .def_readwrite("max_x", &RouteRequest::max_x)
        .def_readwrite("max_y", &RouteRequest::max_y)
        .def_readwrite("min_trace_width", &RouteRequest::min_trace_width)
        .def_readwrite("min_clearance", &RouteRequest::min_clearance)
        .def_readwrite("generated_via_diameter", &RouteRequest::generated_via_diameter)
        .def_readwrite("grid_steps_per_mm", &RouteRequest::grid_steps_per_mm);

    py::class_<RouteResult>(m, "RouteResult")
        .def(py::init<>())
        .def_readonly("net_id", &RouteResult::net_id)
        .def_readonly("found", &RouteResult::found)
        .def_readonly("failure_reason", &RouteResult::failure_reason)
        .def_readonly("grid_pitch", &RouteResult::grid_pitch)
        .def_readonly("origin_x", &RouteResult::origin_x)
        .def_readonly("origin_y", &RouteResult::origin_y)
        .def_readonly("nx", &RouteResult::nx)
        .def_readonly("ny", &RouteResult::ny)
        .def_readonly("nz", &RouteResult::nz)
        .def_readonly("terminal_group_sizes", &RouteResult::terminal_group_sizes)
        .def_readonly("start_vertices", &RouteResult::start_vertices)
        .def_readonly("goal_vertices", &RouteResult::goal_vertices)
        .def_readonly("path_grid", &RouteResult::path_grid)
        .def_readonly("path_mm", &RouteResult::path_mm)
        .def_readonly("candidate_paths_grid", &RouteResult::candidate_paths_grid)
        .def_readonly("candidate_paths_mm", &RouteResult::candidate_paths_mm);

    py::class_<NetCandidateSet>(m, "NetCandidateSet")
        .def(py::init<>())
        .def_readwrite("net_id", &NetCandidateSet::net_id)
        .def_readwrite("candidate_paths_grid", &NetCandidateSet::candidate_paths_grid)
        .def_readwrite("candidate_paths_mm", &NetCandidateSet::candidate_paths_mm)
        .def_readwrite("candidate_via_counts", &NetCandidateSet::candidate_via_counts)
        .def_readwrite("candidate_bend_counts", &NetCandidateSet::candidate_bend_counts)
        .def_readwrite("candidate_lengths_mm", &NetCandidateSet::candidate_lengths_mm)
        .def_readwrite("candidate_boundary_vertices", &NetCandidateSet::candidate_boundary_vertices)
        .def_readwrite("candidate_cover_vertices", &NetCandidateSet::candidate_cover_vertices)
        .def_readwrite("candidate_terminal_coords", &NetCandidateSet::candidate_terminal_coords)
        .def_readwrite("candidate_terminal_groups", &NetCandidateSet::candidate_terminal_groups)
        .def_readwrite("candidate_boundary_vertex_ids", &NetCandidateSet::candidate_boundary_vertex_ids)
        .def_readwrite("candidate_cover_vertex_ids", &NetCandidateSet::candidate_cover_vertex_ids)
        .def_readwrite("candidate_terminal_coord_ids", &NetCandidateSet::candidate_terminal_coord_ids)
        .def_readwrite("candidate_terminal_group_ids", &NetCandidateSet::candidate_terminal_group_ids);

    py::class_<ForbiddenSelectionItem>(m, "ForbiddenSelectionItem")
        .def(py::init<>())
        .def_readwrite("net_id", &ForbiddenSelectionItem::net_id)
        .def_readwrite("candidate_index", &ForbiddenSelectionItem::candidate_index);

    py::class_<ForbiddenSelection>(m, "ForbiddenSelection")
        .def(py::init<>())
        .def_readwrite("items", &ForbiddenSelection::items);

    py::class_<ForbiddenPair>(m, "ForbiddenPair")
        .def(py::init<>())
        .def_readwrite("items", &ForbiddenPair::items);

    py::class_<SelectionRequest>(m, "SelectionRequest")
        .def(py::init<>())
        .def_readwrite("nets", &SelectionRequest::nets)
        .def_readwrite("forbidden_selections", &SelectionRequest::forbidden_selections)
        .def_readwrite("forbidden_pairs", &SelectionRequest::forbidden_pairs)
        .def_readwrite("max_paths_per_net", &SelectionRequest::max_paths_per_net)
        .def_readwrite("prefer_gurobi", &SelectionRequest::prefer_gurobi)
        .def_readwrite("allow_fallback", &SelectionRequest::allow_fallback);

    py::class_<NetSelection>(m, "NetSelection")
        .def(py::init<>())
        .def_readonly("net_id", &NetSelection::net_id)
        .def_readonly("selected_candidate_indices", &NetSelection::selected_candidate_indices)
        .def_readonly("solver", &NetSelection::solver)
        .def_readonly("objective", &NetSelection::objective)
        .def_readonly("has_objective", &NetSelection::has_objective);

    py::class_<SelectionResult>(m, "SelectionResult")
        .def(py::init<>())
        .def_readonly("ok", &SelectionResult::ok)
        .def_readonly("solver", &SelectionResult::solver)
        .def_readonly("message", &SelectionResult::message)
        .def_readonly("selections", &SelectionResult::selections);

    m.def("run_dijkstra_test", &runDijkstraTest, py::arg("request"));
    m.def("select_candidate_paths", &selectCandidatePaths, py::arg("request"));
    m.def("rasterize_selector_geometry", &rasterizeSelectorGeometry, py::arg("request"));
    m.def("build_pad_boundary_groups", &buildPadBoundaryGroups, py::arg("request"));
    m.def("analyze_pad_coverage", &analyzePadCoverage, py::arg("request"));
    m.def("analyze_selector_geometry_batch", &analyzeSelectorGeometryBatch, py::arg("requests"));
    m.def("analyze_selector_geometry_candidate_batch", &analyzeSelectorGeometryCandidateBatch, py::arg("request"));
    m.def(
        "random_grid_astar",
        &randomGridAstar,
        py::arg("nx"),
        py::arg("ny"),
        py::arg("nz"),
        py::arg("pitch"),
        py::arg("sources"),
        py::arg("target_vertices"),
        py::arg("target_indices"),
        py::arg("target_grid_xy"),
        py::arg("trace_blocked"),
        py::arg("via_blocked"),
        py::arg("used_penalty_vertices"),
        py::arg("bounds"),
        py::arg("randomize"),
        py::arg("seed"),
        py::arg("max_expanded")
    );
}
