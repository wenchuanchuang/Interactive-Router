#include "router.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
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
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;

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

using PackedVertexId = std::uint64_t;

// Keep coordinate arithmetic signed to avoid x-1 / y-1 underflow at grid boundaries.
// Large vertex collections are stored as packed ids to reduce memory and hashing overhead.
constexpr std::uint64_t kPackedXBits = 28;
constexpr std::uint64_t kPackedYBits = 28;
constexpr std::uint64_t kPackedZBits = 8;
constexpr std::uint64_t kPackedYShift = kPackedXBits;
constexpr std::uint64_t kPackedZShift = kPackedXBits + kPackedYBits;
constexpr std::uint64_t kPackedXMask = (1ULL << kPackedXBits) - 1ULL;
constexpr std::uint64_t kPackedYMask = (1ULL << kPackedYBits) - 1ULL;
constexpr std::uint64_t kPackedZMask = (1ULL << kPackedZBits) - 1ULL;

struct CandidateRecord {
    int candidate_index = -1;
    int via_count = 0;
    int bend_count = 0;
    double length_mm = 0.0;
    std::vector<PackedVertexId> occupied_vertices;
    std::vector<PackedVertexId> cover_vertices;
    std::vector<PackedVertexId> terminal_coords;
    std::vector<std::vector<PackedVertexId>> terminal_groups;
};

struct NetRecord {
    int net_id = 0;
    int original_net_index = -1;
    std::vector<CandidateRecord> candidates;
};

struct GroupEdgeVar {
    int group_a = -1;
    int group_b = -1;
    GRBVar var;
};

VertexKey toVertex(const GridPoint& point) {
    return VertexKey{
        static_cast<std::int32_t>(point.x),
        static_cast<std::int32_t>(point.y),
        static_cast<std::int32_t>(point.z),
    };
}

PackedVertexId packVertex(const VertexKey& vertex) {
    if (vertex.x < 0 || vertex.y < 0 || vertex.z < 0) {
        throw std::overflow_error("Negative grid coordinate cannot be packed into unsigned vertex id.");
    }
    if (static_cast<std::uint64_t>(vertex.x) > kPackedXMask ||
        static_cast<std::uint64_t>(vertex.y) > kPackedYMask ||
        static_cast<std::uint64_t>(vertex.z) > kPackedZMask) {
        throw std::overflow_error("Grid coordinate exceeds packed vertex id bit budget.");
    }
    return (static_cast<std::uint64_t>(vertex.z) << kPackedZShift) |
           (static_cast<std::uint64_t>(vertex.y) << kPackedYShift) |
           static_cast<std::uint64_t>(vertex.x);
}

VertexKey unpackVertex(PackedVertexId packed) {
    return VertexKey{
        static_cast<std::int32_t>(packed & kPackedXMask),
        static_cast<std::int32_t>((packed >> kPackedYShift) & kPackedYMask),
        static_cast<std::int32_t>((packed >> kPackedZShift) & kPackedZMask),
    };
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
        net.candidate_bend_counts.size(),
        net.candidate_lengths_mm.size(),
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

std::size_t sharedPadCount(const std::vector<int>& a, const std::vector<int>& b) {
    std::size_t count = 0;
    std::size_t ia = 0;
    std::size_t ib = 0;
    while (ia < a.size() && ib < b.size()) {
        if (a[ia] == b[ib]) {
            ++count;
            ++ia;
            ++ib;
        } else if (a[ia] < b[ib]) {
            ++ia;
        } else {
            ++ib;
        }
    }
    return count;
}

std::vector<PackedVertexId> uniqueVerticesFromGridPoints(const std::vector<GridPoint>& points) {
    std::vector<PackedVertexId> vertices;
    vertices.reserve(points.size());
    std::unordered_set<PackedVertexId> seen;
    seen.reserve(points.size() * 2 + 1);
    for (const auto& point : points) {
        PackedVertexId key = packVertex(toVertex(point));
        if (seen.insert(key).second) {
            vertices.push_back(key);
        }
    }
    return vertices;
}

void appendPackedVertices(std::string& key, std::vector<PackedVertexId> vertices) {
    std::sort(vertices.begin(), vertices.end());
    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
    key += "[";
    for (PackedVertexId vertex : vertices) {
        key += std::to_string(vertex);
        key += ",";
    }
    key += "]";
}

std::string candidateRecordGeometryKey(const CandidateRecord& candidate) {
    // Deduplicate candidates only when their selector-visible geometry and
    // terminal coverage are identical. This catches duplicate stable/aggressive
    // final routes without merging genuinely different alternatives.
    std::string key;
    key.reserve(
        (candidate.occupied_vertices.size() + candidate.cover_vertices.size() +
         candidate.terminal_coords.size()) * 24
    );
    key += "occ=";
    appendPackedVertices(key, candidate.occupied_vertices);
    key += "cov=";
    appendPackedVertices(key, candidate.cover_vertices);
    key += "term=";
    appendPackedVertices(key, candidate.terminal_coords);
    key += "groups=";
    std::vector<std::vector<PackedVertexId>> groups = candidate.terminal_groups;
    for (auto& group : groups) {
        std::sort(group.begin(), group.end());
        group.erase(std::unique(group.begin(), group.end()), group.end());
    }
    std::sort(groups.begin(), groups.end());
    for (const auto& group : groups) {
        appendPackedVertices(key, group);
    }
    return key;
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
            int bend_a = static_cast<std::size_t>(a) < net.candidate_bend_counts.size()
                ? net.candidate_bend_counts[static_cast<std::size_t>(a)]
                : 0;
            int bend_b = static_cast<std::size_t>(b) < net.candidate_bend_counts.size()
                ? net.candidate_bend_counts[static_cast<std::size_t>(b)]
                : 0;
            if (bend_a != bend_b) {
                return bend_a < bend_b;
            }
            double length_a = static_cast<std::size_t>(a) < net.candidate_lengths_mm.size()
                ? net.candidate_lengths_mm[static_cast<std::size_t>(a)]
                : static_cast<std::size_t>(a) < net.candidate_paths_mm.size()
                ? pathLengthMm(net.candidate_paths_mm[static_cast<std::size_t>(a)])
                : 0.0;
            double length_b = static_cast<std::size_t>(b) < net.candidate_lengths_mm.size()
                ? net.candidate_lengths_mm[static_cast<std::size_t>(b)]
                : static_cast<std::size_t>(b) < net.candidate_paths_mm.size()
                ? pathLengthMm(net.candidate_paths_mm[static_cast<std::size_t>(b)])
                : 0.0;
            if (length_a != length_b) {
                return length_a < length_b;
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
            candidate.bend_count = candidate_idx < net.candidate_bend_counts.size()
                ? net.candidate_bend_counts[candidate_idx]
                : 0;
            // Store the physical path length for the final lexicographic
            // objective. Explicit lengths cover external-router candidates
            // that are represented by primitives instead of ordered paths.
            candidate.length_mm = candidate_idx < net.candidate_lengths_mm.size()
                ? net.candidate_lengths_mm[candidate_idx]
                : candidate_idx < net.candidate_paths_mm.size()
                ? pathLengthMm(net.candidate_paths_mm[candidate_idx])
                : 0.0;

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
                    packVertex(toVertex(path_grid.front())),
                    packVertex(toVertex(path_grid.back())),
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
            if (a.bend_count != b.bend_count) {
                return a.bend_count < b.bend_count;
            }
            if (a.length_mm != b.length_mm) {
                return a.length_mm < b.length_mm;
            }
            if (a.occupied_vertices.size() != b.occupied_vertices.size()) {
                return a.occupied_vertices.size() < b.occupied_vertices.size();
            }
            return a.candidate_index < b.candidate_index;
        });

        std::vector<CandidateRecord> deduped_candidates;
        deduped_candidates.reserve(candidates.size());
        std::unordered_set<std::string> seen_candidate_geometry;
        seen_candidate_geometry.reserve(candidates.size() * 2 + 1);
        for (auto& candidate : candidates) {
            const std::string key = candidateRecordGeometryKey(candidate);
            if (!seen_candidate_geometry.insert(key).second) {
                continue;
            }
            deduped_candidates.push_back(std::move(candidate));
        }

        (void)truncated_candidates;

        record.candidates = std::move(deduped_candidates);
        records.push_back(std::move(record));
    }

    return records;
}

SelectionResult buildSelectionResultFromChoice(
    const SelectionRequest& request,
    const std::vector<NetRecord>& records,
    const std::vector<std::vector<int>>& chosen_candidate_indices,
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
        if (i < chosen_candidate_indices.size()) {
            ordered[static_cast<std::size_t>(record.original_net_index)].selected_candidate_indices =
                chosen_candidate_indices[i];
        }
    }
    for (auto& sel : ordered) {
        result.selections.push_back(std::move(sel));
    }
    return result;
}

#if defined(ROUTER_HAS_GUROBI)
class GroupConnectivityLazyCallback : public GRBCallback {
public:
    GroupConnectivityLazyCallback(
        const std::vector<bool>& multi_group_net,
        const std::vector<std::vector<GRBVar>>& group_choice,
        const std::vector<std::vector<GroupEdgeVar>>& group_edge_vars
    )
        : multi_group_net_(multi_group_net),
          group_choice_(group_choice),
          group_edge_vars_(group_edge_vars) {}

protected:
    void callback() override {
        if (where != GRB_CB_MIPSOL) {
            return;
        }

        try {
            constexpr double kSelectedThreshold = 0.5;
            for (std::size_t g = 0; g < multi_group_net_.size(); ++g) {
                if (!multi_group_net_[g] || group_choice_[g].size() <= 1) {
                    continue;
                }

                std::vector<int> selected_groups;
                selected_groups.reserve(group_choice_[g].size());
                for (std::size_t group_index = 0; group_index < group_choice_[g].size(); ++group_index) {
                    if (getSolution(group_choice_[g][group_index]) > kSelectedThreshold) {
                        selected_groups.push_back(static_cast<int>(group_index));
                    }
                }
                if (selected_groups.size() <= 1) {
                    continue;
                }

                std::unordered_set<int> selected_set(selected_groups.begin(), selected_groups.end());
                std::unordered_map<int, std::vector<int>> active_adjacency;
                for (int group_index : selected_groups) {
                    active_adjacency[group_index] = {};
                }
                for (const auto& edge : group_edge_vars_[g]) {
                    if (selected_set.find(edge.group_a) == selected_set.end() ||
                        selected_set.find(edge.group_b) == selected_set.end()) {
                        continue;
                    }
                    if (getSolution(edge.var) > kSelectedThreshold) {
                        active_adjacency[edge.group_a].push_back(edge.group_b);
                        active_adjacency[edge.group_b].push_back(edge.group_a);
                    }
                }

                std::unordered_set<int> unvisited(selected_groups.begin(), selected_groups.end());
                std::vector<std::vector<int>> connected_components;
                while (!unvisited.empty()) {
                    const int start = *unvisited.begin();
                    std::vector<int> component;
                    std::vector<int> stack = {start};
                    unvisited.erase(start);
                    while (!stack.empty()) {
                        const int current = stack.back();
                        stack.pop_back();
                        component.push_back(current);
                        auto adjacency_it = active_adjacency.find(current);
                        if (adjacency_it == active_adjacency.end()) {
                            continue;
                        }
                        for (int neighbor : adjacency_it->second) {
                            auto unvisited_it = unvisited.find(neighbor);
                            if (unvisited_it != unvisited.end()) {
                                unvisited.erase(unvisited_it);
                                stack.push_back(neighbor);
                            }
                        }
                    }
                    connected_components.push_back(std::move(component));
                }

                if (connected_components.size() <= 1) {
                    continue;
                }

                const int anchor_group = connected_components.front().front();
                for (std::size_t component_index = 1; component_index < connected_components.size(); ++component_index) {
                    const auto& component = connected_components[component_index];
                    std::unordered_set<int> component_set(component.begin(), component.end());
                    GRBLinExpr cut_expr = 0.0;
                    for (const auto& edge : group_edge_vars_[g]) {
                        const bool a_in = component_set.find(edge.group_a) != component_set.end();
                        const bool b_in = component_set.find(edge.group_b) != component_set.end();
                        if (a_in != b_in) {
                            cut_expr += edge.var;
                        }
                    }

                    // If one selected group sits inside this component and another selected group
                    // sits outside it, at least one active group-graph edge must cross the cut.
                    GRBLinExpr lazy_expr = cut_expr
                        - group_choice_[g][static_cast<std::size_t>(anchor_group)]
                        - group_choice_[g][static_cast<std::size_t>(component.front())];
                    addLazy(lazy_expr, GRB_GREATER_EQUAL, -1.0);
                }
            }
        } catch (const GRBException&) {
            throw;
        } catch (...) {
            // Let Gurobi surface unknown callback failures through the outer solve path.
            throw;
        }
    }

private:
    const std::vector<bool>& multi_group_net_;
    const std::vector<std::vector<GRBVar>>& group_choice_;
    const std::vector<std::vector<GroupEdgeVar>>& group_edge_vars_;
};

SelectionResult solveWithGurobi(const SelectionRequest& request, const std::vector<NetRecord>& records) {
    // Candidate selection uses binary variables per path.
    // Terminal coverage stays exact for single-group nets, while multi-group nets
    // allow several partial candidates as long as each pad is covered and each
    // candidate group contributes at most one selected path.
    // Shared non-terminal vertices remain mutually exclusive across different nets.
    // Objective: lexicographically minimize via count, bend count, then wire length.
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    model.set(GRB_IntParam_Threads, 20);
    //model.set(GRB_DoubleParam_MIPGap, 0.01);
    model.set(GRB_IntParam_LazyConstraints, 1);

    std::vector<std::vector<GRBVar>> path_choice;
    path_choice.resize(records.size());
    for (std::size_t g = 0; g < records.size(); ++g) {
        path_choice[g].reserve(records[g].candidates.size());
        for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
            path_choice[g].push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY));
        }
    }

    std::vector<std::vector<GRBVar>> group_choice(records.size());
    std::vector<std::vector<GroupEdgeVar>> group_edge_vars(records.size());

    std::set<PackedVertexId> terminal_vertices;
    std::vector<std::vector<std::vector<PackedVertexId>>> terminal_groups_by_net(records.size());
    std::vector<std::vector<std::unordered_set<PackedVertexId>>> terminal_group_sets_by_net(records.size());
    std::vector<std::vector<std::vector<int>>> candidate_hit_terminal_groups_by_net(records.size());
    std::vector<std::vector<std::vector<int>>> candidate_groups_by_net(records.size());
    std::vector<std::vector<std::vector<int>>> group_pad_sets_by_net(records.size());
    std::vector<std::vector<int>> zero_hit_candidates_by_net(records.size());
    std::vector<bool> multi_group_net(records.size(), false);
    std::map<PackedVertexId, GRBLinExpr> v_exprs;
    std::map<PackedVertexId, std::set<int>> v_groups;
    std::size_t total_candidate_count = 0;

    for (std::size_t g = 0; g < records.size(); ++g) {
        total_candidate_count += records[g].candidates.size();
        if (!records[g].candidates.empty()) {
            std::size_t max_group_count = 0;
            for (const auto& candidate : records[g].candidates) {
                max_group_count = std::max(max_group_count, candidate.terminal_groups.size());
            }

            std::vector<std::unordered_set<PackedVertexId>> union_groups(max_group_count);
            for (const auto& candidate : records[g].candidates) {
                for (std::size_t gi = 0; gi < candidate.terminal_groups.size(); ++gi) {
                    for (const auto& vertex : candidate.terminal_groups[gi]) {
                        union_groups[gi].insert(vertex);
                    }
                }
            }

            terminal_groups_by_net[g].clear();
            terminal_group_sets_by_net[g].clear();
            terminal_groups_by_net[g].reserve(union_groups.size());
            terminal_group_sets_by_net[g].reserve(union_groups.size());
            for (auto& group_set : union_groups) {
                if (group_set.empty()) {
                    continue;
                }
                std::vector<PackedVertexId> group_vertices;
                group_vertices.reserve(group_set.size());
                for (const auto& vertex : group_set) {
                    group_vertices.push_back(vertex);
                    terminal_vertices.insert(vertex);
                }
                terminal_groups_by_net[g].push_back(std::move(group_vertices));
                terminal_group_sets_by_net[g].push_back(std::move(group_set));
            }

            candidate_hit_terminal_groups_by_net[g].resize(records[g].candidates.size());
            std::map<std::vector<int>, int> signature_to_group;
            for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
                const auto& candidate = records[g].candidates[p];
                std::vector<int> hit_groups;
                hit_groups.reserve(terminal_group_sets_by_net[g].size());
                for (std::size_t gi = 0; gi < terminal_group_sets_by_net[g].size(); ++gi) {
                    bool hit_group = false;
                    for (const auto& occupied : candidate.cover_vertices) {
                        if (terminal_group_sets_by_net[g][gi].find(occupied) != terminal_group_sets_by_net[g][gi].end()) {
                            hit_group = true;
                            break;
                        }
                    }
                    if (hit_group) {
                        hit_groups.push_back(static_cast<int>(gi));
                    }
                }
                candidate_hit_terminal_groups_by_net[g][p] = hit_groups;
                if (hit_groups.empty()) {
                    zero_hit_candidates_by_net[g].push_back(static_cast<int>(p));
                    continue;
                }
                auto signature_it = signature_to_group.find(hit_groups);
                int group_index = -1;
                if (signature_it == signature_to_group.end()) {
                    group_index = static_cast<int>(group_pad_sets_by_net[g].size());
                    signature_to_group.emplace(hit_groups, group_index);
                    group_pad_sets_by_net[g].push_back(hit_groups);
                    candidate_groups_by_net[g].push_back({});
                } else {
                    group_index = signature_it->second;
                }
                candidate_groups_by_net[g][static_cast<std::size_t>(group_index)].push_back(static_cast<int>(p));
            }
            multi_group_net[g] = group_pad_sets_by_net[g].size() >= 2;
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
    std::size_t total_candidate_group_rows = 0;
    std::size_t total_group_conflict_rows = 0;
    std::size_t total_group_connectivity_edge_count = 0;
    std::size_t total_group_selection_rows = 0;
    std::size_t total_group_edge_link_rows = 0;
    std::size_t total_zero_hit_rows = 0;
    for (std::size_t g = 0; g < terminal_groups_by_net.size(); ++g) {
        total_terminal_group_rows += terminal_groups_by_net[g].size();
        total_zero_hit_rows += zero_hit_candidates_by_net[g].size();
        if (multi_group_net[g]) {
            total_candidate_group_rows += candidate_groups_by_net[g].size();
            total_group_selection_rows += candidate_groups_by_net[g].size();
            for (std::size_t ga = 0; ga < group_pad_sets_by_net[g].size(); ++ga) {
                for (std::size_t gb = ga + 1; gb < group_pad_sets_by_net[g].size(); ++gb) {
                    if (sharedPadCount(group_pad_sets_by_net[g][ga], group_pad_sets_by_net[g][gb]) > 0) {
                        ++total_group_connectivity_edge_count;
                        total_group_edge_link_rows += 2;
                    }
                }
            }
        }
        for (std::size_t gi = 0; gi < terminal_groups_by_net[g].size(); ++gi) {
            std::size_t hit_count = 0;
            for (std::size_t p = 0; p < candidate_hit_terminal_groups_by_net[g].size(); ++p) {
                const auto& hit_groups = candidate_hit_terminal_groups_by_net[g][p];
                if (std::find(hit_groups.begin(), hit_groups.end(), static_cast<int>(gi)) != hit_groups.end()) {
                    ++hit_count;
                }
            }
            total_terminal_group_nonzeros += hit_count;
        }
        if (multi_group_net[g]) {
            for (std::size_t ga = 0; ga < group_pad_sets_by_net[g].size(); ++ga) {
                for (std::size_t gb = ga + 1; gb < group_pad_sets_by_net[g].size(); ++gb) {
                    const std::size_t shared_pad_count =
                        sharedPadCount(group_pad_sets_by_net[g][ga], group_pad_sets_by_net[g][gb]);
                    if (shared_pad_count >= 2) {
                        ++total_group_conflict_rows;
                    }
                }
            }
        }
    }

    std::size_t total_capacity_rows = 0;
    std::size_t total_capacity_nonzeros = 0;
    for (const auto& entry : v_groups) {
        const PackedVertexId& vertex = entry.first;
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
        << "selector_gurobi_candidate_group_row_count = " << total_candidate_group_rows << '\n'
        << "selector_gurobi_group_conflict_row_count = " << total_group_conflict_rows << '\n'
        << "selector_gurobi_group_connectivity_edge_count = " << total_group_connectivity_edge_count << '\n'
        << "selector_gurobi_group_selection_row_count = " << total_group_selection_rows << '\n'
        << "selector_gurobi_group_edge_link_row_count = " << total_group_edge_link_rows << '\n'
        << "selector_gurobi_zero_hit_row_count = " << total_zero_hit_rows << '\n'
        << "selector_gurobi_capacity_row_count = " << total_capacity_rows << '\n'
        << "selector_gurobi_total_row_count_estimated = "
        << (total_terminal_group_rows
            + total_candidate_group_rows
            + total_group_conflict_rows
            + total_group_selection_rows
            + total_group_edge_link_rows
            + total_zero_hit_rows
            + total_capacity_rows) << '\n'
        << "selector_gurobi_terminal_group_nonzeros_estimated = " << total_terminal_group_nonzeros << '\n'
        << "selector_gurobi_capacity_nonzeros_estimated = " << total_capacity_nonzeros << '\n'
        << "selector_gurobi_total_nonzeros_estimated = " << (total_terminal_group_nonzeros + total_capacity_nonzeros)
        << std::endl;

    for (const auto& record : records) {
        std::size_t union_group_count = 0;
        std::size_t candidate_group_count = 0;
        if (record.original_net_index >= 0 &&
            static_cast<std::size_t>(record.original_net_index) < terminal_groups_by_net.size()) {
            union_group_count = terminal_groups_by_net[static_cast<std::size_t>(record.original_net_index)].size();
            candidate_group_count = group_pad_sets_by_net[static_cast<std::size_t>(record.original_net_index)].size();
        }
        std::cout
            << "selector_gurobi_net_summary "
            << "net=" << record.net_id
            << " candidates=" << record.candidates.size()
            << " terminal_groups=" << union_group_count
            << " candidate_groups=" << candidate_group_count
            << std::endl;
    }

    for (std::size_t g = 0; g < zero_hit_candidates_by_net.size(); ++g) {
        for (int candidate_index : zero_hit_candidates_by_net[g]) {
            model.addConstr(
                path_choice[g][static_cast<std::size_t>(candidate_index)] == 0.0,
                "ZeroHit_net" + std::to_string(records[g].net_id) + "_c" + std::to_string(candidate_index)
            );
        }
    }

    for (std::size_t g = 0; g < candidate_groups_by_net.size(); ++g) {
        if (!multi_group_net[g]) {
            continue;
        }
        group_choice[g].reserve(candidate_groups_by_net[g].size());
        for (std::size_t group_index = 0; group_index < candidate_groups_by_net[g].size(); ++group_index) {
            group_choice[g].push_back(
                model.addVar(
                    0.0,
                    1.0,
                    0.0,
                    GRB_BINARY,
                    "GroupSelect_net" + std::to_string(records[g].net_id) + "_g" + std::to_string(group_index)
                )
            );
        }
        for (std::size_t group_index = 0; group_index < candidate_groups_by_net[g].size(); ++group_index) {
            GRBLinExpr group_expr = 0.0;
            for (int candidate_index : candidate_groups_by_net[g][group_index]) {
                group_expr += path_choice[g][static_cast<std::size_t>(candidate_index)];
            }
            model.addConstr(
                group_choice[g][group_index] == group_expr,
                "GroupChoice_net" + std::to_string(records[g].net_id) + "_g" + std::to_string(group_index)
            );
            model.addConstr(
                group_expr <= 1.0,
                "CandidateGroup_net" + std::to_string(records[g].net_id) + "_g" + std::to_string(group_index)
            );
        }
        for (std::size_t ga = 0; ga < group_pad_sets_by_net[g].size(); ++ga) {
            for (std::size_t gb = ga + 1; gb < group_pad_sets_by_net[g].size(); ++gb) {
                if (sharedPadCount(group_pad_sets_by_net[g][ga], group_pad_sets_by_net[g][gb]) == 0) {
                    continue;
                }
                GroupEdgeVar edge_var;
                edge_var.group_a = static_cast<int>(ga);
                edge_var.group_b = static_cast<int>(gb);
                edge_var.var = model.addVar(
                    0.0,
                    1.0,
                    0.0,
                    GRB_BINARY,
                    "GroupEdge_net" + std::to_string(records[g].net_id)
                        + "_a" + std::to_string(ga)
                        + "_b" + std::to_string(gb)
                );
                model.addConstr(
                    edge_var.var <= group_choice[g][ga],
                    "GroupEdgeLinkA_net" + std::to_string(records[g].net_id)
                        + "_a" + std::to_string(ga)
                        + "_b" + std::to_string(gb)
                );
                model.addConstr(
                    edge_var.var <= group_choice[g][gb],
                    "GroupEdgeLinkB_net" + std::to_string(records[g].net_id)
                        + "_a" + std::to_string(ga)
                        + "_b" + std::to_string(gb)
                );
                group_edge_vars[g].push_back(std::move(edge_var));
            }
        }
        for (std::size_t ga = 0; ga < group_pad_sets_by_net[g].size(); ++ga) {
            for (std::size_t gb = ga + 1; gb < group_pad_sets_by_net[g].size(); ++gb) {
                const std::size_t shared_pad_count =
                    sharedPadCount(group_pad_sets_by_net[g][ga], group_pad_sets_by_net[g][gb]);
                if (shared_pad_count < 2) {
                    continue;
                }
                GRBLinExpr conflict_expr = 0.0;
                for (int candidate_index : candidate_groups_by_net[g][ga]) {
                    conflict_expr += path_choice[g][static_cast<std::size_t>(candidate_index)];
                }
                for (int candidate_index : candidate_groups_by_net[g][gb]) {
                    conflict_expr += path_choice[g][static_cast<std::size_t>(candidate_index)];
                }
                model.addConstr(
                    conflict_expr <= 1.0,
                    "GroupConflict_net" + std::to_string(records[g].net_id)
                        + "_a" + std::to_string(ga)
                        + "_b" + std::to_string(gb)
                );
            }
        }
    }

    for (std::size_t g = 0; g < terminal_groups_by_net.size(); ++g) {
        for (std::size_t gi = 0; gi < terminal_groups_by_net[g].size(); ++gi) {
            GRBLinExpr group_expr = 0.0;
            for (std::size_t p = 0; p < candidate_hit_terminal_groups_by_net[g].size(); ++p) {
                const auto& hit_groups = candidate_hit_terminal_groups_by_net[g][p];
                if (std::find(hit_groups.begin(), hit_groups.end(), static_cast<int>(gi)) != hit_groups.end()) {
                    group_expr += path_choice[g][p];
                }
            }
            if (multi_group_net[g]) {
                model.addConstr(
                    group_expr >= 1.0,
                    "TerminalGroup_net" + std::to_string(records[g].net_id) + "_g" + std::to_string(gi)
                );
            } else {
                model.addConstr(
                    group_expr == 1.0,
                    "TerminalGroup_net" + std::to_string(records[g].net_id) + "_g" + std::to_string(gi)
                );
            }
        }
    }

    for (const auto& entry : v_exprs) {
        const PackedVertexId& vertex = entry.first;
        const GRBLinExpr& expr = entry.second;
        const bool is_terminal_vertex = terminal_vertices.find(vertex) != terminal_vertices.end();
        if (is_terminal_vertex) {
            continue;
        }
        auto it = v_groups.find(vertex);
        if (it != v_groups.end() && it->second.size() > 1) {
            const VertexKey unpacked = unpackVertex(vertex);
            model.addConstr(
                expr <= 1.0,
                "Capacity_x" + std::to_string(unpacked.x) + "_y" + std::to_string(unpacked.y) + "_z" + std::to_string(unpacked.z)
            );
        }
    }

    GRBLinExpr via_obj = 0.0;
    GRBLinExpr bend_obj = 0.0;
    GRBLinExpr length_obj = 0.0;
    for (std::size_t g = 0; g < records.size(); ++g) {
        for (std::size_t p = 0; p < records[g].candidates.size(); ++p) {
            via_obj += static_cast<double>(records[g].candidates[p].via_count) * path_choice[g][p];
            bend_obj += static_cast<double>(records[g].candidates[p].bend_count) * path_choice[g][p];
            length_obj += records[g].candidates[p].length_mm * path_choice[g][p];
        }
    }
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.setObjectiveN(via_obj, 0, 3, 1.0, 0.0, 0.0, "MinViaCount");
    model.setObjectiveN(bend_obj, 1, 2, 1.0, 0.0, 0.0, "MinBendCount");
    model.setObjectiveN(length_obj, 2, 1, 1.0, 0.0, 0.0, "MinWireLengthMm");
    GroupConnectivityLazyCallback connectivity_callback(multi_group_net, group_choice, group_edge_vars);
    model.setCallback(&connectivity_callback);
    model.optimize();

    const int status = model.get(GRB_IntAttr_Status);
    if (status == GRB_INFEASIBLE) {
        SelectionResult fail;
        fail.ok = false;
        fail.solver = "gurobi-cpp-via-bend-length-lex";
        fail.message = "Gurobi model is infeasible.";
        return fail;
    }
    if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL && status != GRB_TIME_LIMIT && status != GRB_INTERRUPTED) {
        SelectionResult fail;
        fail.ok = false;
        fail.solver = "gurobi-cpp-via-bend-length-lex";
        fail.message = "Gurobi solve failed with status " + std::to_string(status) + ".";
        return fail;
    }

    std::vector<std::vector<int>> chosen(records.size());
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
                chosen[g].push_back(records[g].candidates[p].candidate_index);
            }
        }
        if (chosen[g].empty() && best_p >= 0 && best_p < static_cast<int>(records[g].candidates.size())) {
            chosen[g].push_back(records[g].candidates[static_cast<std::size_t>(best_p)].candidate_index);
        }
    }

    model.set(GRB_IntParam_ObjNumber, 0);
    const double via_objective = model.get(GRB_DoubleAttr_ObjNVal);
    model.set(GRB_IntParam_ObjNumber, 1);
    const double bend_objective = model.get(GRB_DoubleAttr_ObjNVal);
    model.set(GRB_IntParam_ObjNumber, 2);
    const double length_objective = model.get(GRB_DoubleAttr_ObjNVal);
    std::cout
        << "selector_gurobi_objective_mode = lexicographic(via_count, bend_count, wire_length_mm)" << '\n'
        << "selector_gurobi_objective_via_count = " << via_objective << '\n'
        << "selector_gurobi_objective_bend_count = " << bend_objective << '\n'
        << "selector_gurobi_objective_wire_length_mm = " << length_objective
        << std::endl;
    return buildSelectionResultFromChoice(
        request,
        records,
        chosen,
        "gurobi-cpp-via-bend-length-lex",
        via_objective,
        "Solved by Gurobi with terminal/collision constraints and lexicographic via-count/bend-count/wire-length objective."
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
                fail.solver = "gurobi-cpp-via-bend-length-lex";
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
                fail.solver = "gurobi-cpp-via-bend-length-lex";
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
