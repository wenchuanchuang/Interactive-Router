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

    m.def("run_dijkstra_test", &runDijkstraTest, py::arg("request"));
}
