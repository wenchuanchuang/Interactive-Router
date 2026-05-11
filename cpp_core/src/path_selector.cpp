#include "router.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#if defined(ROUTER_HAS_GUROBI)
#include "gurobi_c++.h"
#endif

namespace interactive_router {
namespace {

struct VertexKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VertexKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator<(const VertexKey& other) const {
        if (x != other.x) {
            return x < other.x;
        }
        if (y != other.y) {
            return y < other.y;
        }
        return z < other.z;
    }
};

struct VertexKeyHash {
    std::size_t operator()(const VertexKey& key) const {
        std::size_t hx = static_cast<std::size_t>(key.x) * 73856093u;
        std::size_t hy = static_cast<std::size_t>(key.y) * 19349663u;
        std::size_t hz = static_cast<std::size_t>(key.z) * 83492791u;
        return hx ^ hy ^ hz;
    }
};

struct CandidateRecord {
    int candidate_index = -1;
    int via_count = 0;
    double length_mm = 0.0;
    std::vector<VertexKey> occupied_vertices;
    std::vector<VertexKey> cover_vertices;
    std::vector<VertexKey> terminal_coords;
    std::vector<std::vector<VertexKey>> terminal_groups;
};

struct NetRecord {
    int net_id = 0;
    int original_net_index = -1;
    std::vector<CandidateRecord> candidates;
};

VertexKey toVertex(const GridPoint& point) {
    return VertexKey{point.x, point.y, point.z};
}

double pathLengthMm(const std::vector<Point2D>& path) {
    if (path.size() < 2) {
        return 0.0;
    }
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i - 1].x;
        double dy = path[i].y - path[i - 1].y;
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

std::size_t candidateCount(const NetCandidateSet& net) {
    return std::max({
        net.candidate_paths_grid.size(),
        net.candidate_paths_mm.size(),
        net.candidate_via_counts.size(),
        net.candidate_boundary_vertices.size(),
        net.candidate_cover_vertices.size(),
        net.candidate_terminal_coords.size(),
        net.candidate_terminal_groups.size(),
    });
}

int countPathVias(const std::vector<GridPoint>& path) {
    if (path.size() < 2) {
        return 0;
    }
    int via_count = 0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (path[i].z != path[i - 1].z) {
            ++via_count;
        }
    }
    return via_count;
}

std::vector<VertexKey> uniqueVerticesFromGridPoints(const std::vector<GridPoint>& points) {
    std::vector<VertexKey> vertices;
    vertices.reserve(points.size());
    std::unordered_set<VertexKey, VertexKeyHash> seen;
    seen.reserve(points.size() * 2 + 1);
    for (const auto& point : points) {
        VertexKey key = toVertex(point);
        if (seen.insert(key).second) {
            vertices.push_back(key);
        }
    }
    return vertices;
}

SelectionResult fallbackShortestSelection(const SelectionRequest& request, const std::string& reason) {
    SelectionResult result;
    result.ok = true;
    result.solver = "cpp-fallback-shortest";
    result.message = reason;

    const int keep_count = std::max(1, request.max_paths_per_net);
    for (const auto& net : request.nets) {
        NetSelection selection;
        selection.net_id = net.net_id;
        selection.solver = result.solver;

        const std::size_t candidate_count = candidateCount(net);
        if (candidate_count == 0) {
            selection.has_objective = false;
            result.selections.push_back(std::move(selection));
            continue;
        }

        std::vector<int> ranked(static_cast<int>(candidate_count));
        std::iota(ranked.begin(), ranked.end(), 0);
        std::sort(ranked.begin(), ranked.end(), [&](int a, int b) {
            int via_a = static_cast<std::size_t>(a) < net.candidate_via_counts.size()
                ? net.candidate_via_counts[static_cast<std::size_t>(a)]
                : 0;
            int via_b = static_cast<std::size_t>(b) < net.candidate_via_counts.size()
                ? net.candidate_via_counts[static_cast<std::size_t>(b)]
                : 0;
            if (via_a != via_b) {
                return via_a < via_b;
            }
            std::size_t boundary_a = static_cast<std::size_t>(a) < net.candidate_boundary_vertices.size()
                ? net.candidate_boundary_vertices[static_cast<std::size_t>(a)].size()
                : 0u;
            std::size_t boundary_b = static_cast<std::size_t>(b) < net.candidate_boundary_vertices.size()
                ? net.candidate_boundary_vertices[static_cast<std::size_t>(b)].size()
                : 0u;
            if (boundary_a != boundary_b) {
                return boundary_a < boundary_b;
            }
            return a < b;
        });

        int bounded_keep = std::min(keep_count, static_cast<int>(ranked.size()));
        selection.selected_candidate_indices.assign(ranked.begin(), ranked.begin() + bounded_keep);
        selection.objective = 0.0;
        selection.has_objective = true;
        result.selections.push_back(std::move(selection));
    }
    return result;
}

std::vector<NetRecord> buildNetRecords(const SelectionRequest& request, bool* truncated_candidates) {
    std::vector<NetRecord> records;
    records.reserve(request.nets.size());

    for (std::size_t net_idx = 0; net_idx < request.nets.size(); ++net_idx) {
        const auto& net = request.nets[net_idx];
        NetRecord record;
        record.net_id = net.net_id;
        record.original_net_index = static_cast<int>(net_idx);

        const std::size_t candidate_count = candidateCount(net);
        std::vector<CandidateRecord> candidates;
        candidates.reserve(candidate_count);
        for (std::size_t candidate_idx = 0; candidate_idx < candidate_count; ++candidate_idx) {
            const std::vector<GridPoint> empty_path;
            const auto& path_grid = candidate_idx < net.candidate_paths_grid.size()
                ? net.candidate_paths_grid[candidate_idx]
                : empty_path;

            CandidateRecord candidate;
            candidate.candidate_index = static_cast<int>(candidate_idx);
            candidate.via_count = candidate_idx < net.candidate_via_counts.size()
                ? net.candidate_via_counts[candidate_idx]
                : 0;
            candidate.length_mm = 0.0;

            if (candidate_idx < net.candidate_boundary_vertices.size() &&
                !net.candidate_boundary_vertices[candidate_idx].empty()) {
                candidate.occupied_vertices = uniqueVerticesFromGridPoints(net.candidate_boundary_vertices[candidate_idx]);
            } else if (!path_grid.empty()) {
                candidate.occupied_vertices = uniqueVerticesFromGridPoints(path_grid);
            }

            if (candidate_idx < net.candidate_cover_vertices.size() &&
                !net.candidate_cover_vertices[candidate_idx].empty()) {
                candidate.cover_vertices = uniqueVerticesFromGridPoints(net.candidate_cover_vertices[candidate_idx]);
            } else {
                candidate.cover_vertices = candidate.occupied_vertices;
            }

            if (candidate_idx < net.candidate_terminal_coords.size() &&
                !net.candidate_terminal_coords[candidate_idx].empty()) {
                candidate.terminal_coords = uniqueVerticesFromGridPoints(net.candidate_terminal_coords[candidate_idx]);
            } else if (!path_grid.empty()) {
                candidate.terminal_coords = {
                    toVertex(path_grid.front()),
                    toVertex(path_grid.back()),
                };
            }

            if (candidate_idx < net.candidate_terminal_groups.size() &&
                !net.candidate_terminal_groups[candidate_idx].empty()) {
                for (const auto& group_points : net.candidate_terminal_groups[candidate_idx]) {
                    auto group_vertices = uniqueVerticesFromGridPoints(group_points);
                    if (!group_vertices.empty()) {
                        candidate.terminal_groups.push_back(std::move(group_vertices));
                    }
                }
            }
            if (candidate.terminal_groups.empty()) {
                for (const auto& terminal : candidate.terminal_coords) {
                    candidate.terminal_groups.push_back({terminal});
                }
            }

            if (candidate.terminal_coords.empty() && !candidate.terminal_groups.empty()) {
                for (const auto& group : candidate.terminal_groups) {
                    if (!group.empty()) {
                        candidate.terminal_coords.push_back(group.front());
                    }
                }
            }

            if (!candidate.occupied_vertices.empty() && !candidate.terminal_coords.empty()) {
                candidates.push_back(std::move(candidate));
            }
        }

        std::sort(candidates.begin(), candidates.end(), [](const CandidateRecord& a, const CandidateRecord& b) {
            if (a.via_count != b.via_count) {
                return a.via_count < b.via_count;
            }
            if (a.occupied_vertices.size() != b.occupied_vertices.size()) {
                return a.occupied_vertices.size() < b.occupied_vertices.size();
            }
            return a.candidate_index < b.candidate_index;
        });

        (void)truncated_candidates;

        record.candidates = std::move(candidates);
        records.push_back(std::move(record));
    }

    return records;
}

SelectionResult buildSelectionResultFromChoice(
    const SelectionRequest& request,
    const std::vector<NetRecord>& records,
    const std::vector<int>& chosen_candidate_index,
    const std::string& solver,
    double objective,
    const std::string& message
) {
    SelectionResult result;
    result.ok = true;
    result.solver = solver;
    result.message = message;

    std::vector<NetSelection> ordered(request.nets.size());
    for (std::size_t i = 0; i < request.nets.size(); ++i) {
        ordered[i].net_id = request.nets[i].net_id;
        ordered[i].solver = solver;
        ordered[i].has_objective = true;
        ordered[i].objective = objective;
    }
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto& record = records[i];
        int chosen = (i < chosen_candidate_index.size()) ? chosen_candidate_index[i] : -1;
        if (chosen >= 0) {
            ordered[static_cast<std::size_t>(record.original_net_index)].selected_candidate_indices = {chosen};
        }
    }
    for (auto& sel : ordered) {
        result.selections.push_back(std::move(sel));
    }
    return result;
}

#if defined(ROUTER_HAS_GUROBI)
SelectionResult solveWithGurobi(const SelectionRequest& request, const std::vector<NetRecord>& records) {
    // This solver follows tmp.cpp SolveNetwork style:
    // - path choice binary variables
    // - terminal vertex == 1
    // - shared non-terminal vertex <= 1 (only if touched by >= 2 nets)
    // - objective: minimize via_count * pathChoice
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    model.set(GRB_IntParam_Threads, 20);
    model.set(GRB_DoubleParam_MIPGap, 0.1);

    std::vector<std::vector<GRBVar>> path_choice;
    path_choice.resize(records.size());
    for (std::size_t g = 0; g < records.size(); ++g) {
        path_choice[g].reserve(records[g].candidates.size());
        for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
            path_choice[g].push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY));
        }
    }

    std::set<VertexKey> terminal_vertices;
    std::vector<std::vector<std::vector<VertexKey>>> terminal_groups_by_net(records.size());
    std::map<VertexKey, GRBLinExpr> v_exprs;
    std::map<VertexKey, std::set<int>> v_groups;
    std::size_t total_candidate_count = 0;

    for (std::size_t g = 0; g < records.size(); ++g) {
        total_candidate_count += records[g].candidates.size();
        if (!records[g].candidates.empty()) {
            std::size_t max_group_count = 0;
            for (const auto& candidate : records[g].candidates) {
                max_group_count = std::max(max_group_count, candidate.terminal_groups.size());
            }

            std::vector<std::unordered_set<VertexKey, VertexKeyHash>> union_groups(max_group_count);
            for (const auto& candidate : records[g].candidates) {
                for (std::size_t gi = 0; gi < candidate.terminal_groups.size(); ++gi) {
                    for (const auto& vertex : candidate.terminal_groups[gi]) {
                        union_groups[gi].insert(vertex);
                    }
                }
            }

            terminal_groups_by_net[g].clear();
            terminal_groups_by_net[g].reserve(union_groups.size());
            for (auto& group_set : union_groups) {
                if (group_set.empty()) {
                    continue;
                }
                std::vector<VertexKey> group_vertices;
                group_vertices.reserve(group_set.size());
                for (const auto& vertex : group_set) {
                    group_vertices.push_back(vertex);
                    terminal_vertices.insert(vertex);
                }
                terminal_groups_by_net[g].push_back(std::move(group_vertices));
            }
        }
        for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
            const auto& candidate = records[g].candidates[p];
            for (const auto& vertex : candidate.occupied_vertices) {
                v_exprs[vertex] += path_choice[g][p];
                v_groups[vertex].insert(static_cast<int>(g));
            }
        }
    }

    std::size_t total_terminal_group_rows = 0;
    std::size_t total_terminal_group_nonzeros = 0;
    for (std::size_t g = 0; g < terminal_groups_by_net.size(); ++g) {
        total_terminal_group_rows += terminal_groups_by_net[g].size();
        for (std::size_t gi = 0; gi < terminal_groups_by_net[g].size(); ++gi) {
            const auto& group_vertices = terminal_groups_by_net[g][gi];
            std::unordered_set<VertexKey, VertexKeyHash> group_vertex_set;
            group_vertex_set.reserve(group_vertices.size() * 2 + 1);
            for (const auto& vertex : group_vertices) {
                group_vertex_set.insert(vertex);
            }
            std::size_t hit_count = 0;
            for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
                const auto& candidate = records[g].candidates[p];
                bool hit_group = false;
                for (const auto& occupied : candidate.cover_vertices) {
                    if (group_vertex_set.find(occupied) != group_vertex_set.end()) {
                        hit_group = true;
                        break;
                    }
                }
                if (hit_group) {
                    ++hit_count;
                }
            }
            total_terminal_group_nonzeros += hit_count;
        }
    }

    std::size_t total_capacity_rows = 0;
    std::size_t total_capacity_nonzeros = 0;
    for (const auto& entry : v_groups) {
        const VertexKey& vertex = entry.first;
        const bool is_terminal_vertex = terminal_vertices.find(vertex) != terminal_vertices.end();
        if (is_terminal_vertex) {
            continue;
        }
        if (entry.second.size() > 1) {
            ++total_capacity_rows;
            auto expr_it = v_exprs.find(vertex);
            if (expr_it != v_exprs.end()) {
                total_capacity_nonzeros += entry.second.size();
            }
        }
    }

    std::cout
        << "selector_gurobi_var_count = " << total_candidate_count << '\n'
        << "selector_gurobi_terminal_group_row_count = " << total_terminal_group_rows << '\n'
        << "selector_gurobi_capacity_row_count = " << total_capacity_rows << '\n'
        << "selector_gurobi_total_row_count_estimated = " << (total_terminal_group_rows + total_capacity_rows) << '\n'
        << "selector_gurobi_terminal_group_nonzeros_estimated = " << total_terminal_group_nonzeros << '\n'
        << "selector_gurobi_capacity_nonzeros_estimated = " << total_capacity_nonzeros << '\n'
        << "selector_gurobi_total_nonzeros_estimated = " << (total_terminal_group_nonzeros + total_capacity_nonzeros)
        << std::endl;

    for (const auto& record : records) {
        std::size_t union_group_count = 0;
        if (record.original_net_index >= 0 &&
            static_cast<std::size_t>(record.original_net_index) < terminal_groups_by_net.size()) {
            union_group_count = terminal_groups_by_net[static_cast<std::size_t>(record.original_net_index)].size();
        }
        std::cout
            << "selector_gurobi_net_summary "
            << "net=" << record.net_id
            << " candidates=" << record.candidates.size()
            << " terminal_groups=" << union_group_count
            << std::endl;
    }

    for (std::size_t g = 0; g < terminal_groups_by_net.size(); ++g) {
        for (std::size_t gi = 0; gi < terminal_groups_by_net[g].size(); ++gi) {
            GRBLinExpr group_expr = 0.0;
            const auto& group_vertices = terminal_groups_by_net[g][gi];
            std::unordered_set<VertexKey, VertexKeyHash> group_vertex_set;
            group_vertex_set.reserve(group_vertices.size() * 2 + 1);
            for (const auto& vertex : group_vertices) {
                group_vertex_set.insert(vertex);
            }
            for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
                const auto& candidate = records[g].candidates[p];
                bool hit_group = false;
                for (const auto& occupied : candidate.cover_vertices) {
                    if (group_vertex_set.find(occupied) != group_vertex_set.end()) {
                        hit_group = true;
                        break;
                    }
                }
                if (hit_group) {
                    group_expr += path_choice[g][p];
                }
            }
            model.addConstr(
                group_expr == 1.0,
                "TerminalGroup_net" + std::to_string(records[g].net_id) + "_g" + std::to_string(gi)
            );
        }
    }

    for (const auto& entry : v_exprs) {
        const VertexKey& vertex = entry.first;
        const GRBLinExpr& expr = entry.second;
        const bool is_terminal_vertex = terminal_vertices.find(vertex) != terminal_vertices.end();
        if (is_terminal_vertex) {
            continue;
        }
        auto it = v_groups.find(vertex);
        if (it != v_groups.end() && it->second.size() > 1) {
            model.addConstr(
                expr <= 1.0,
                "Capacity_x" + std::to_string(vertex.x) + "_y" + std::to_string(vertex.y) + "_z" + std::to_string(vertex.z)
            );
        }
    }

    GRBLinExpr obj = 0.0;
    for (std::size_t g = 0; g < records.size(); ++g) {
        for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
            obj += static_cast<double>(records[g].candidates[p].via_count) * path_choice[g][p];
        }
    }
    model.setObjective(obj, GRB_MINIMIZE);
    model.optimize();

    const int status = model.get(GRB_IntAttr_Status);
    if (status == GRB_INFEASIBLE) {
        SelectionResult fail;
        fail.ok = false;
        fail.solver = "gurobi-cpp-via-min";
        fail.message = "Gurobi model is infeasible.";
        return fail;
    }
    if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL && status != GRB_TIME_LIMIT && status != GRB_INTERRUPTED) {
        SelectionResult fail;
        fail.ok = false;
        fail.solver = "gurobi-cpp-via-min";
        fail.message = "Gurobi solve failed with status " + std::to_string(status) + ".";
        return fail;
    }

    std::vector<int> chosen(records.size(), -1);
    for (std::size_t g = 0; g < records.size(); ++g) {
        double best_x = -1.0;
        int best_p = -1;
        for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
            const double x = path_choice[g][p].get(GRB_DoubleAttr_X);
            if (x > best_x) {
                best_x = x;
                best_p = static_cast<int>(p);
            }
            if (x > 0.5) {
                best_p = static_cast<int>(p);
                break;
            }
        }
        if (best_p >= 0 && best_p < static_cast<int>(records[g].candidates.size())) {
            chosen[g] = records[g].candidates[static_cast<std::size_t>(best_p)].candidate_index;
        }
    }

    const double objective = model.get(GRB_DoubleAttr_ObjVal);
    return buildSelectionResultFromChoice(
        request,
        records,
        chosen,
        "gurobi-cpp-via-min",
        objective,
        "Solved by Gurobi with terminal/collision constraints and via-min objective."
    );
}
#endif

}  // namespace

SelectionResult selectCandidatePaths(const SelectionRequest& request) {
    if (request.nets.empty()) {
        SelectionResult result;
        result.ok = true;
        result.solver = "none";
        result.message = "No nets provided.";
        return result;
    }

    if (request.max_paths_per_net != 1) {
        return fallbackShortestSelection(
            request,
            "Only max_paths_per_net=1 is supported in current selector; fallback used."
        );
    }

    bool truncated_candidates = false;
    std::vector<NetRecord> records = buildNetRecords(request, &truncated_candidates);
    for (const auto& record : records) {
        if (record.candidates.empty()) {
            return fallbackShortestSelection(
                request,
                "At least one net has no valid candidates for selector; fallback used."
            );
        }
    }

#if defined(ROUTER_HAS_GUROBI)
    if (request.prefer_gurobi) {
        try {
            SelectionResult gurobi_result = solveWithGurobi(request, records);
            if (gurobi_result.ok) {
                return gurobi_result;
            }
            if (!request.allow_fallback) {
                return gurobi_result;
            }
            return fallbackShortestSelection(
                request,
                "Gurobi was unavailable/infeasible; fallback shortest selector used. " + gurobi_result.message
            );
        } catch (const GRBException& ex) {
            if (!request.allow_fallback) {
                SelectionResult fail;
                fail.ok = false;
                fail.solver = "gurobi-cpp-via-min";
                fail.message = "Gurobi exception: " + ex.getMessage();
                return fail;
            }
            return fallbackShortestSelection(
                request,
                "Gurobi exception, fallback shortest selector used. " + std::string(ex.getMessage())
            );
        } catch (...) {
            if (!request.allow_fallback) {
                SelectionResult fail;
                fail.ok = false;
                fail.solver = "gurobi-cpp-via-min";
                fail.message = "Unknown Gurobi exception.";
                return fail;
            }
            return fallbackShortestSelection(
                request,
                "Unknown Gurobi exception, fallback shortest selector used."
            );
        }
    }
#else
    if (request.prefer_gurobi && !request.allow_fallback) {
        SelectionResult fail;
        fail.ok = false;
        fail.solver = "none";
        fail.message = "This build does not include Gurobi support.";
        return fail;
    }
#endif

    std::string message = "Using fallback shortest selector.";
    if (truncated_candidates) {
        message += " Candidate cap applied.";
    }
    return fallbackShortestSelection(request, message);
}

}  // namespace interactive_router
