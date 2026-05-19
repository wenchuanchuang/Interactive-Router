#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "include/geometry.h"
#include "include/router.h"

namespace py = pybind11;
using namespace interactive_router;

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
        .def_readwrite("candidate_boundary_vertices", &NetCandidateSet::candidate_boundary_vertices)
        .def_readwrite("candidate_cover_vertices", &NetCandidateSet::candidate_cover_vertices)
        .def_readwrite("candidate_terminal_coords", &NetCandidateSet::candidate_terminal_coords)
        .def_readwrite("candidate_terminal_groups", &NetCandidateSet::candidate_terminal_groups);

    py::class_<SelectionRequest>(m, "SelectionRequest")
        .def(py::init<>())
        .def_readwrite("nets", &SelectionRequest::nets)
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
}
