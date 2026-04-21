#include "router.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <tuple>
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

double clearanceForNet(const RouteRequest& request, int net_id) {
    for (const auto& track : request.tracks) {
        if (track.net_id == net_id && track.clearance > 0.0) {
            return track.clearance;
        }
    }
    return request.min_clearance;
}

bool layerMatchesPad(const PadGeometry& pad, const std::string& layer) {
    return std::find(pad.layers.begin(), pad.layers.end(), layer) != pad.layers.end()
        || std::find(pad.layers.begin(), pad.layers.end(), "*.Cu") != pad.layers.end();
}

//某個實體座標 point，有沒有落在 pad 幾何範圍裡
bool pointInsidePad(const PadGeometry& pad, const Point2D& point, double bloat = 0.0) {
    constexpr double kPi = 3.14159265358979323846;
    double dx = point.x - pad.center.x;
    double dy = point.y - pad.center.y;

    if (pad.shape == "circle") {
        double radius = std::max(pad.size_x, pad.size_y) * 0.5 + bloat;
        return dx * dx + dy * dy <= radius * radius;
    }

    double angle = pad.rotation_degrees * kPi / 180.0;
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);
    double local_x = dx * cos_a + dy * sin_a;
    double local_y = -dx * sin_a + dy * cos_a;

    if (pad.shape == "oval") {
        double radius_x = pad.size_x * 0.5 + bloat;
        double radius_y = pad.size_y * 0.5 + bloat;
        if (radius_x <= 0.0 || radius_y <= 0.0) {
            return false;
        }
        double norm_x = local_x / radius_x;
        double norm_y = local_y / radius_y;
        return norm_x * norm_x + norm_y * norm_y <= 1.0;
    }

    return std::abs(local_x) <= pad.size_x * 0.5 + bloat
        && std::abs(local_y) <= pad.size_y * 0.5 + bloat;
}

//如果這顆起點 pad 有多層 center 可以當起點，那應該選哪一層當 start center
// 如果起點 pad 有多層 center
// 只取 一個 center_vertex當起點
// 優先選「原本這個 net 的 track endpoint 落在 pad 裡的那一層」
// 如果原本繞線不存在或判斷不出來，就退回 最上層可用 layer 的 center
int preferredPadStartLayer(
    const RouteRequest& request,
    const PadGeometry& pad,
    int net_id,
    const Grid3D& grid
) {
    for (const auto& track : request.tracks) {
        if (track.net_id != net_id) {
            continue;
        }
        //檢查這條 track 的 layer 是不是 pad 支援的 layer
        if (!layerMatchesPad(pad, track.layer)) {
            continue;
        }
        //檢查這條 track 的 start/end 有沒有落在 pad 裡
        if (pointInsidePad(pad, track.start) || pointInsidePad(pad, track.end)) {
            // 原本這顆 pad 的連線是在這個 layer 上接入/接出的
            int z = grid.layerIndex(track.layer);
            if (z >= 0) {
                return z;
            }
        }
    }

    //如果整個 net 都找不到 track endpoint 落在 pad 裡, 取 pad 的最上層可用 layer
    for (int z = 0; z < grid.nz(); ++z) {
        if (layerMatchesPad(pad, grid.layers()[z])) {
            return z;
        }
    }
    return -1;
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

constexpr double kViaPenalty = 50.0;
constexpr double kMinDiscount = 0.2;
constexpr double kDefaultGridStepsPerMm = 10.0;
constexpr int kBaseStepLimit = 1000;
constexpr int kDijkstraStepBase = 15000;
constexpr unsigned int kRandomSeed = 88;
constexpr unsigned int kShuffleSeed = 777;

const int kPlanarRayDirs[8][2] = {
    {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1},
};

const int kDijkstraDeltas[10][3] = {
    {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0},
    {0, 0, -1}, {0, 0, 1},
    {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0},
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

bool samePoint(const GridPoint& a, const GridPoint& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool pointInPath(const std::vector<GridPoint>& path, const GridPoint& point) {
    return std::any_of(path.begin(), path.end(), [&point](const GridPoint& existing) {
        return samePoint(existing, point);
    });
}

bool isAngleValid(int prev_dx, int prev_dy, int prev_dz, int curr_dx, int curr_dy, int curr_dz) {
    if (prev_dx == 0 && prev_dy == 0 && prev_dz == 0) {
        return true;
    }
    if (prev_dz != 0 || curr_dz != 0) {
        return !(prev_dz != 0 && curr_dz != 0);
    }

    double dot = static_cast<double>(prev_dx * curr_dx + prev_dy * curr_dy);
    double mag_prev = std::sqrt(static_cast<double>(prev_dx * prev_dx + prev_dy * prev_dy));
    double mag_curr = std::sqrt(static_cast<double>(curr_dx * curr_dx + curr_dy * curr_dy));
    if (mag_prev <= 0.0 || mag_curr <= 0.0) {
        return false;
    }

    double cos_theta = dot / (mag_prev * mag_curr);
    bool is_straight = std::abs(cos_theta - 1.0) < 1e-5;
    bool is_45_deg = std::abs(cos_theta - 0.707106781) < 1e-5;
    return is_straight || is_45_deg;
}

int orientation(const GridPoint& p, const GridPoint& q, const GridPoint& r) {
    long long val = 1LL * (q.y - p.y) * (r.x - q.x) - 1LL * (q.x - p.x) * (r.y - q.y);
    if (val == 0) {
        return 0;
    }
    return val > 0 ? 1 : 2;
}

bool onSegment(const GridPoint& p, const GridPoint& q, const GridPoint& r) {
    return r.x <= std::max(p.x, q.x) && r.x >= std::min(p.x, q.x)
        && r.y <= std::max(p.y, q.y) && r.y >= std::min(p.y, q.y);
}

bool doSegmentsIntersect(const GridPoint& p1, const GridPoint& q1, const GridPoint& p2, const GridPoint& q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) {
        return true;
    }
    if (o1 == 0 && onSegment(p1, q1, p2)) {
        return true;
    }
    if (o2 == 0 && onSegment(p1, q1, q2)) {
        return true;
    }
    if (o3 == 0 && onSegment(p2, q2, p1)) {
        return true;
    }
    if (o4 == 0 && onSegment(p2, q2, q1)) {
        return true;
    }
    return false;
}

bool isGoalPoint(const Grid3D& grid, const GridPoint& point, const std::unordered_set<std::size_t>& goal_indices) {
    return grid.inBounds(point) && goal_indices.find(grid.flatten(point)) != goal_indices.end();
}

bool isObstacleForPath(const Grid3D& grid, const GridPoint& point, const std::unordered_set<std::size_t>& goal_indices) {
    if (!grid.inBounds(point)) {
        return true;
    }
    if (isGoalPoint(grid, point, goal_indices)) {
        return false;
    }
    return grid.isBlocked(point);
}

double gridSegmentLength(const GridPoint& a, const GridPoint& b) {
    int dx = b.x - a.x;
    int dy = b.y - a.y;
    int dz = b.z - a.z;
    return std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
}

double dijkstraShortestDistanceToAnyGoal(
    const Grid3D& grid,
    const GridPoint& start,
    const std::vector<GridPoint>& goal_vertices
) {
    if (!grid.inBounds(start) || goal_vertices.empty()) {
        return -1.0;
    }

    std::unordered_set<std::size_t> goal_indices;
    for (const auto& goal : goal_vertices) {
        if (grid.inBounds(goal)) {
            goal_indices.insert(grid.flatten(goal));
        }
    }
    if (goal_indices.empty()) {
        return -1.0;
    }
    if (isGoalPoint(grid, start, goal_indices)) {
        return 0.0;
    }

    std::size_t grid_size = static_cast<std::size_t>(grid.nx()) * grid.ny() * grid.nz();
    std::vector<double> dist(grid_size, std::numeric_limits<double>::infinity());
    std::vector<GridPoint> prev(grid_size, {-1, -1, -1});
    std::vector<std::uint8_t> visited(grid_size, 0);

    using QueueItem = std::tuple<double, int, int, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> queue;

    std::size_t start_index = grid.flatten(start);
    dist[start_index] = 0.0;
    queue.push({0.0, start.x, start.y, start.z});

    while (!queue.empty()) {
        auto [current_dist, x, y, z] = queue.top();
        queue.pop();

        GridPoint current{x, y, z};
        std::size_t current_index = grid.flatten(current);
        if (visited[current_index]) {
            continue;
        }
        visited[current_index] = 1;

        if (isGoalPoint(grid, current, goal_indices)) {
            return current_dist;
        }

        for (const auto& delta : kDijkstraDeltas) {
            GridPoint next{x + delta[0], y + delta[1], z + delta[2]};
            if (!grid.inBounds(next)) {
                continue;
            }
            if (isObstacleForPath(grid, next, goal_indices)) {
                continue;
            }

            GridPoint previous = prev[current_index];
            if (previous.x >= 0 && previous.z == current.z && current.z == next.z) {
                int dot_xy = (previous.x - current.x) * (next.x - current.x)
                    + (previous.y - current.y) * (next.y - current.y);
                if (dot_xy >= 0) {
                    continue;
                }
            }

            std::size_t next_index = grid.flatten(next);
            double new_dist = current_dist + gridSegmentLength(current, next);
            if (new_dist < dist[next_index]) {
                dist[next_index] = new_dist;
                prev[next_index] = current;
                queue.push({new_dist, next.x, next.y, next.z});
            }
        }
    }

    return -1.0;
}

int directionIndexForDelta(int dx, int dy, int dz) {
    for (int i = 0; i < 10; ++i) {
        if (kDijkstraDeltas[i][0] == dx && kDijkstraDeltas[i][1] == dy && kDijkstraDeltas[i][2] == dz) {
            return i;
        }
    }
    return -1;
}

bool isViaClear(
    const Grid3D& grid,
    int x,
    int y,
    int z1,
    int z2,
    const std::unordered_set<std::size_t>& goal_indices
) {
    int step = z2 > z1 ? 1 : -1;
    for (int z = z1; z != z2 + step; z += step) {
        GridPoint point{x, y, z};
        if (isObstacleForPath(grid, point, goal_indices)) {
            return false;
        }
    }
    return true;
}

bool isLineOfSightClear(
    const Grid3D& grid,
    int ox,
    int oy,
    int oz,
    int tx,
    int ty,
    const std::unordered_set<std::size_t>& goal_indices
) {
    int dx = tx - ox;
    int dy = ty - oy;
    int dir_x = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    int dir_y = dy > 0 ? 1 : (dy < 0 ? -1 : 0);

    auto is_obs = [&](int x, int y) {
        return isObstacleForPath(grid, {x, y, oz}, goal_indices);
    };

    if (is_obs(tx, ty)) {
        return false;
    }
    if (dir_y == 0) {
        for (int x = ox + dir_x; x != tx; x += dir_x) {
            if (is_obs(x, oy)) {
                return false;
            }
        }
        return true;
    }
    if (dir_x == 0) {
        for (int y = oy + dir_y; y != ty; y += dir_y) {
            if (is_obs(ox, y)) {
                return false;
            }
        }
        return true;
    }

    int cx = dir_x >= 0 ? ox : ox - 1;
    int cy = dir_y >= 0 ? oy : oy - 1;
    double t_delta_x = std::abs(1.0 / dx);
    double t_delta_y = std::abs(1.0 / dy);
    double t_max_x = t_delta_x;
    double t_max_y = t_delta_y;
    double slope_yx = static_cast<double>(dy) / dx;
    double slope_xy = static_cast<double>(dx) / dy;

    while (true) {
        double t = std::min(t_max_x, t_max_y);
        if (std::abs(t - 1.0) < 1e-7 || t > 1.0) {
            return true;
        }

        if (std::abs(t_max_x - t_max_y) < 1e-7) {
            cx += dir_x;
            cy += dir_y;
            t_max_x += t_delta_x;
            t_max_y += t_delta_y;
            int vx = dir_x > 0 ? cx : cx + 1;
            int vy = dir_y > 0 ? cy : cy + 1;
            if (is_obs(vx, vy)) {
                return false;
            }
        } else if (t_max_x < t_max_y) {
            cx += dir_x;
            t_max_x += t_delta_x;
            int vx = dir_x > 0 ? cx : cx + 1;
            int vy_bottom = cy;
            int vy_top = cy + 1;
            if (is_obs(vx, vy_bottom) && is_obs(vx, vy_top)) {
                return false;
            }
            double intersect_y = oy + (vx - ox) * slope_yx;
            if (is_obs(vx, vy_bottom) && (intersect_y - vy_bottom) < 0.5 - 1e-6) {
                return false;
            }
            if (is_obs(vx, vy_top) && (vy_top - intersect_y) < 0.5 - 1e-6) {
                return false;
            }
        } else {
            cy += dir_y;
            t_max_y += t_delta_y;
            int vy = dir_y > 0 ? cy : cy + 1;
            int vx_left = cx;
            int vx_right = cx + 1;
            if (is_obs(vx_left, vy) && is_obs(vx_right, vy)) {
                return false;
            }
            double intersect_x = ox + (vy - oy) * slope_xy;
            if (is_obs(vx_left, vy) && (intersect_x - vx_left) < 0.5 - 1e-6) {
                return false;
            }
            if (is_obs(vx_right, vy) && (vx_right - intersect_x) < 0.5 - 1e-6) {
                return false;
            }
        }
    }
}

int minimumSegmentCountToAnyGoal(
    const Grid3D& grid,
    const GridPoint& start,
    const std::vector<GridPoint>& goal_vertices
) {
    if (!grid.inBounds(start) || goal_vertices.empty()) {
        return -1;
    }

    std::unordered_set<std::size_t> goal_indices;
    for (const auto& goal : goal_vertices) {
        if (grid.inBounds(goal)) {
            goal_indices.insert(grid.flatten(goal));
        }
    }
    if (goal_indices.empty()) {
        return -1;
    }
    if (isGoalPoint(grid, start, goal_indices)) {
        return 0;
    }

    constexpr int kNoDirection = 10;
    constexpr int kDirectionStates = 11;
    constexpr int kInfSegments = std::numeric_limits<int>::max() / 4;

    auto stateIndex = [&](const GridPoint& point, int dir_index) {
        return grid.flatten(point) * kDirectionStates + static_cast<std::size_t>(dir_index);
    };

    std::size_t grid_size = static_cast<std::size_t>(grid.nx()) * grid.ny() * grid.nz();
    std::vector<int> dist(grid_size * kDirectionStates, kInfSegments);
    std::deque<std::pair<GridPoint, int>> queue;

    std::size_t start_state = stateIndex(start, kNoDirection);
    dist[start_state] = 0;
    queue.push_front({start, kNoDirection});

    while (!queue.empty()) {
        auto [current, prev_dir_index] = queue.front();
        queue.pop_front();
        int current_segments = dist[stateIndex(current, prev_dir_index)];

        if (isGoalPoint(grid, current, goal_indices)) {
            return current_segments;
        }

        for (const auto& delta : kDijkstraDeltas) {
            GridPoint next{current.x + delta[0], current.y + delta[1], current.z + delta[2]};
            if (isObstacleForPath(grid, next, goal_indices)) {
                continue;
            }
            if (delta[2] == 0) {
                if (!isLineOfSightClear(grid, current.x, current.y, current.z, next.x, next.y, goal_indices)) {
                    continue;
                }
            } else if (!isViaClear(grid, current.x, current.y, current.z, next.z, goal_indices)) {
                continue;
            }

            int next_dir_index = directionIndexForDelta(delta[0], delta[1], delta[2]);
            if (next_dir_index < 0) {
                continue;
            }

            if (prev_dir_index != kNoDirection) {
                const auto& prev_delta = kDijkstraDeltas[prev_dir_index];
                if (!isAngleValid(prev_delta[0], prev_delta[1], prev_delta[2], delta[0], delta[1], delta[2])) {
                    continue;
                }
            }

            int turn_cost = (prev_dir_index == next_dir_index) ? 0 : 1;
            int next_segments = current_segments + turn_cost;
            std::size_t next_state = stateIndex(next, next_dir_index);
            if (next_segments >= dist[next_state]) {
                continue;
            }

            dist[next_state] = next_segments;
            if (turn_cost == 0) {
                queue.push_front({next, next_dir_index});
            } else {
                queue.push_back({next, next_dir_index});
            }
        }
    }

    return -1;
}

std::vector<GridPoint> castRays360(
    const Grid3D& grid,
    const GridPoint& origin,
    const std::unordered_set<std::size_t>& goal_indices,
    int prev_dx = 0,
    int prev_dy = 0,
    int prev_dz = 0
) {
    std::vector<GridPoint> candidates;
    for (const auto& dir : kPlanarRayDirs) {
        int dx = dir[0];
        int dy = dir[1];
        if (!isAngleValid(prev_dx, prev_dy, prev_dz, dx, dy, 0)) {
            continue;
        }
        int curr_x = origin.x + dx;
        int curr_y = origin.y + dy;
        while (grid.inBounds({curr_x, curr_y, origin.z})) {
            if (!isLineOfSightClear(grid, origin.x, origin.y, origin.z, curr_x, curr_y, goal_indices)) {
                break;
            }
            candidates.push_back({curr_x, curr_y, origin.z});
            curr_x += dx;
            curr_y += dy;
        }
    }

    for (int target_z = 0; target_z < grid.nz(); ++target_z) {
        if (target_z == origin.z) {
            continue;
        }
        int dz = target_z - origin.z;
        if (isAngleValid(prev_dx, prev_dy, prev_dz, 0, 0, dz)
            && isViaClear(grid, origin.x, origin.y, origin.z, target_z, goal_indices)) {
            candidates.push_back({origin.x, origin.y, target_z});
        }
    }
    return candidates;
}

double pathSegmentLength(const GridPoint& a, const GridPoint& b) {
    int dx = b.x - a.x;
    int dy = b.y - a.y;
    int dz = b.z - a.z;
    return std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
}

unsigned int spatialHash(int x, int y, int z, unsigned int seed) {
    unsigned int h = (static_cast<unsigned int>(x) * 73856093U)
        ^ (static_cast<unsigned int>(y) * 19349663U)
        ^ (static_cast<unsigned int>(z) * 83492791U)
        ^ seed;
    h ^= h >> 16;
    h *= 0x85ebca6bU;
    h ^= h >> 13;
    h *= 0xc2b2ae35U;
    h ^= h >> 16;
    return h;
}

double spatialNoise(int x, int y, int z, unsigned int seed) {
    return ((spatialHash(x, y, z, seed) % 10000) / 10000.0 - 0.5) * 0.1;
}

bool isValidFinalMove(const GridPoint& curr, const GridPoint& goal, int prev_dx, int prev_dy, int prev_dz) {
    int final_dx = goal.x - curr.x;
    int final_dy = goal.y - curr.y;
    int final_dz = goal.z - curr.z;
    if (final_dz != 0) {
        if (final_dx != 0 || final_dy != 0) {
            return false;
        }
    } else {
        int abs_dx = std::abs(final_dx);
        int abs_dy = std::abs(final_dy);
        if (abs_dx != 0 && abs_dy != 0 && abs_dx != abs_dy) {
            return false;
        }
    }
    return isAngleValid(prev_dx, prev_dy, prev_dz, final_dx, final_dy, final_dz);
}

bool isClearFinalMove(
    const Grid3D& grid,
    const GridPoint& curr,
    const GridPoint& goal,
    const std::unordered_set<std::size_t>& goal_indices
) {
    int final_dx = goal.x - curr.x;
    int final_dy = goal.y - curr.y;
    int final_dz = goal.z - curr.z;
    if (final_dz == 0) {
        return isLineOfSightClear(grid, curr.x, curr.y, curr.z, goal.x, goal.y, goal_indices);
    }
    if (final_dx == 0 && final_dy == 0) {
        return isViaClear(grid, curr.x, curr.y, curr.z, goal.z, goal_indices);
    }
    return false;
}

bool segmentIntersectsPath(const std::vector<GridPoint>& path, const GridPoint& curr, const GridPoint& next) {
    if (path.size() < 2 || curr.z != next.z) {
        return false;
    }
    for (std::size_t i = 0; i + 2 < path.size(); ++i) {
        if (path[i].z == path[i + 1].z && path[i].z == curr.z
            && doSegmentsIntersect(path[i], path[i + 1], curr, next)) {
            return true;
        }
    }
    return false;
}

std::vector<GridPoint> sortedGoalVertices(const std::vector<GridPoint>& goals, const GridPoint& curr) {
    std::vector<GridPoint> sorted = goals;
    std::sort(sorted.begin(), sorted.end(), [&curr](const GridPoint& a, const GridPoint& b) {
        int da = std::abs(a.x - curr.x) + std::abs(a.y - curr.y) + std::abs(a.z - curr.z) * 8;
        int db = std::abs(b.x - curr.x) + std::abs(b.y - curr.y) + std::abs(b.z - curr.z) * 8;
        if (da != db) {
            return da < db;
        }
        if (a.x != b.x) {
            return a.x < b.x;
        }
        if (a.y != b.y) {
            return a.y < b.y;
        }
        return a.z < b.z;
    });
    if (sorted.size() > 64) {
        sorted.resize(64);
    }
    return sorted;
}

void dfsAllExactSegmentsToAnyGoal(
    const Grid3D& grid,
    const GridPoint& initial_curr,
    const std::vector<GridPoint>& goal_vertices,
    const std::unordered_set<std::size_t>& goal_indices,
    int initial_depth,
    int target_segments,
    std::vector<GridPoint> initial_path,
    std::vector<std::vector<GridPoint>>& all_paths,
    std::size_t max_results,
    int max_steps,
    bool uniform_heuristic,
    unsigned int child_seed
) {
    const int full_hp = max_steps;
    struct DfsState {
        GridPoint curr;
        int depth;
        std::vector<GridPoint> path;
    };

    std::vector<DfsState> stack;
    stack.reserve(1000);
    stack.push_back({initial_curr, initial_depth, std::move(initial_path)});

    while (!stack.empty()) {
        if (all_paths.size() >= max_results) {
            break;
        }
        if (--max_steps <= 0) {
            break;
        }

        DfsState state = std::move(stack.back());
        stack.pop_back();
        GridPoint curr = state.curr;
        std::vector<GridPoint>& path = state.path;

        int prev_dx = 0;
        int prev_dy = 0;
        int prev_dz = 0;
        if (path.size() >= 2) {
            prev_dx = curr.x - path[path.size() - 2].x;
            prev_dy = curr.y - path[path.size() - 2].y;
            prev_dz = curr.z - path[path.size() - 2].z;
            if (prev_dz == 0) {
                int gcd = std::gcd(std::abs(prev_dx), std::abs(prev_dy));
                if (gcd > 0) {
                    prev_dx /= gcd;
                    prev_dy /= gcd;
                }
            } else {
                prev_dx = 0;
                prev_dy = 0;
            }
        }

        if (state.depth == target_segments - 1) {
            for (const auto& goal : sortedGoalVertices(goal_vertices, curr)) {
                if (!isValidFinalMove(curr, goal, prev_dx, prev_dy, prev_dz)) {
                    continue;
                }
                if (!isClearFinalMove(grid, curr, goal, goal_indices)) {
                    continue;
                }
                if (segmentIntersectsPath(path, curr, goal)) {
                    continue;
                }
                auto finished = path;
                finished.push_back(goal);
                all_paths.push_back(std::move(finished));
                max_steps = full_hp;
                if (all_paths.size() >= max_results) {
                    break;
                }
            }
            continue;
        }

        auto next_points = castRays360(grid, curr, goal_indices, prev_dx, prev_dy, prev_dz);
        double progress = static_cast<double>(state.depth) / std::max(1, target_segments - 1);
        double curve_factor = 4.0 * (progress - 0.5) * (progress - 0.5);
        double dynamic_via_penalty = kViaPenalty * (curve_factor * (1.0 - kMinDiscount) + kMinDiscount);
        GridPoint guide_goal = sortedGoalVertices(goal_vertices, curr).front();

        auto score = [&](const GridPoint& p) {
            double tx = 0.0;
            double ty = 0.0;
            double tz = 0.0;
            if (uniform_heuristic) {
                int remaining = std::max(1, target_segments - state.depth);
                tx = curr.x + (guide_goal.x - curr.x) / static_cast<double>(remaining);
                ty = curr.y + (guide_goal.y - curr.y) / static_cast<double>(remaining);
                tz = path.front().z + (guide_goal.z - path.front().z) * progress;
            } else {
                tx = guide_goal.x;
                ty = guide_goal.y;
                tz = guide_goal.z;
            }
            double dx = p.x - tx;
            double dy = p.y - ty;
            double dz = p.z - tz;
            double base = dx * dx + dy * dy + dz * dz * dynamic_via_penalty;
            return base + (base + 1.0) * spatialNoise(p.x, p.y, p.z, child_seed);
        };

        std::sort(next_points.begin(), next_points.end(), [&](const GridPoint& a, const GridPoint& b) {
            double score_a = score(a);
            double score_b = score(b);
            long long scaled_a = std::llround(score_a * 1000000.0);
            long long scaled_b = std::llround(score_b * 1000000.0);
            if (scaled_a != scaled_b) {
                return scaled_a > scaled_b;
            }
            if (a.x != b.x) {
                return a.x > b.x;
            }
            if (a.y != b.y) {
                return a.y > b.y;
            }
            return a.z > b.z;
        });

        for (const auto& next : next_points) {
            if (isGoalPoint(grid, next, goal_indices) || pointInPath(path, next)) {
                continue;
            }

            int next_dx = next.x - curr.x;
            int next_dy = next.y - curr.y;
            int next_dz = next.z - curr.z;
            if (next_dz == 0) {
                int gcd = std::gcd(std::abs(next_dx), std::abs(next_dy));
                if (gcd > 0) {
                    int base_dx = next_dx / gcd;
                    int base_dy = next_dy / gcd;
                    if (prev_dz == 0 && prev_dx == base_dx && prev_dy == base_dy) {
                        continue;
                    }
                }
            }
            if (segmentIntersectsPath(path, curr, next)) {
                continue;
            }

            DfsState next_state;
            next_state.curr = next;
            next_state.depth = state.depth + 1;
            next_state.path = path;
            next_state.path.push_back(next);
            stack.push_back(std::move(next_state));
        }
    }
}

std::vector<std::vector<GridPoint>> findAllExactSegmentPathsToAnyGoal(
    const Grid3D& grid,
    const GridPoint& start,
    const std::vector<GridPoint>& goal_vertices,
    int target_segments,
    int dynamic_step_limit,
    std::size_t max_results,
    bool uniform_heuristic
) {
    if (target_segments <= 0 || goal_vertices.empty()) {
        return {};
    }

    std::unordered_set<std::size_t> goal_indices;
    for (const auto& goal : goal_vertices) {
        if (grid.inBounds(goal)) {
            goal_indices.insert(grid.flatten(goal));
        }
    }

    std::vector<std::vector<GridPoint>> paths;
    if (target_segments == 1) {
        for (const auto& goal : sortedGoalVertices(goal_vertices, start)) {
            if (isValidFinalMove(start, goal, 0, 0, 0) && isClearFinalMove(grid, start, goal, goal_indices)) {
                paths.push_back({start, goal});
                if (paths.size() >= max_results) {
                    break;
                }
            }
        }
        return paths;
    }

    auto first_points = castRays360(grid, start, goal_indices, 0, 0, 0);
    GridPoint guide_goal = sortedGoalVertices(goal_vertices, start).front();
    auto score = [&](const GridPoint& p) {
        double tx = start.x + (guide_goal.x - start.x) / static_cast<double>(target_segments);
        double ty = start.y + (guide_goal.y - start.y) / static_cast<double>(target_segments);
        double dz = p.z - start.z;
        double base = (p.x - tx) * (p.x - tx) + (p.y - ty) * (p.y - ty) + dz * dz * kViaPenalty;
        if (!uniform_heuristic) {
            base = (p.x - guide_goal.x) * (p.x - guide_goal.x)
                + (p.y - guide_goal.y) * (p.y - guide_goal.y)
                + (p.z - guide_goal.z) * (p.z - guide_goal.z) * kViaPenalty;
        }
        return base + (base + 1.0) * spatialNoise(p.x, p.y, p.z, kRandomSeed);
    };
    std::sort(first_points.begin(), first_points.end(), [&](const GridPoint& a, const GridPoint& b) {
        double score_a = score(a);
        double score_b = score(b);
        if (std::abs(score_a - score_b) > 1e-9) {
            return score_a < score_b;
        }
        if (a.x != b.x) {
            return a.x < b.x;
        }
        if (a.y != b.y) {
            return a.y < b.y;
        }
        return a.z < b.z;
    });

    std::size_t branch_limit = std::min<std::size_t>(first_points.size(), 48);
    std::size_t local_max = std::max<std::size_t>(2, max_results / std::max<std::size_t>(1, branch_limit) + 2);
    std::vector<std::future<std::vector<std::vector<GridPoint>>>> futures;
    futures.reserve(branch_limit);

    for (std::size_t i = 0; i < branch_limit; ++i) {
        GridPoint next = first_points[i];
        if (isGoalPoint(grid, next, goal_indices)) {
            continue;
        }

        unsigned int child_seed = kRandomSeed ^ static_cast<unsigned int>((i + 1) * 1234567U);
        futures.push_back(std::async(std::launch::async, [&, next, child_seed]() {
            std::vector<std::vector<GridPoint>> local_paths;
            dfsAllExactSegmentsToAnyGoal(
                grid,
                next,
                goal_vertices,
                goal_indices,
                1,
                target_segments,
                {start, next},
                local_paths,
                local_max,
                dynamic_step_limit,
                uniform_heuristic,
                child_seed
            );
            return local_paths;
        }));
    }

    for (auto& future : futures) {
        auto local_paths = future.get();
        for (auto& path : local_paths) {
            paths.push_back(std::move(path));
            if (paths.size() >= max_results) {
                break;
            }
        }
    }

    std::mt19937 rng(kShuffleSeed);
    std::shuffle(paths.begin(), paths.end(), rng);
    return paths;
}

double routeLength(const std::vector<GridPoint>& path) {
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += pathSegmentLength(path[i - 1], path[i]);
    }
    return length;
}

std::vector<std::vector<GridPoint>> generateCandidatePaths(
    const Grid3D& grid,
    const GridPoint& start,
    const std::vector<GridPoint>& goals,
    std::size_t max_results,
    bool backward_search = false
) {
    std::vector<std::vector<GridPoint>> all_paths;
    int nearest = std::numeric_limits<int>::max();
    for (const auto& goal : goals) {
        nearest = std::min(nearest, std::abs(goal.x - start.x) + std::abs(goal.y - start.y) + std::abs(goal.z - start.z));
    }
    double real_dist = dijkstraShortestDistanceToAnyGoal(grid, start, goals);
    int dynamic_step_limit = 0;
    if (real_dist < 0.0) {
        dynamic_step_limit = kBaseStepLimit + (nearest == std::numeric_limits<int>::max() ? 0 : nearest * 60);
        std::cout << "  dynamic_step_limit use Manhattan " << std::endl;
    } else {
        dynamic_step_limit = kDijkstraStepBase + static_cast<int>(real_dist) * 60;
        std::cout << "  dynamic_step_limit use dijkstraShortest " << std::endl;
    }
    std::cout << "  dynamic_step_limit " << dynamic_step_limit << std::endl;

    int minimum_segments = minimumSegmentCountToAnyGoal(grid, start, goals);
    if (minimum_segments < 0) {
        std::cout << "  minimum_segment_presearch found no reachable grid path" << std::endl;
        return {};
    }

    constexpr int kMaxCandidateSegments = 20;
    std::cout << "  minimum_segment_presearch " << minimum_segments << " segments" << std::endl;
    if (minimum_segments > kMaxCandidateSegments) {
        std::cout << "  minimum segments exceed max candidate segments " << kMaxCandidateSegments << std::endl;
        return {};
    }

    for (int segments = std::max(1, minimum_segments); segments <= kMaxCandidateSegments; ++segments) {
        std::cout << "      Searching for " << segments << "-segment paths..." << std::endl;

        std::size_t quota = std::max<std::size_t>(2, max_results / 2);
        auto uniform = findAllExactSegmentPathsToAnyGoal(
            grid,
            start,
            goals,
            segments,
            dynamic_step_limit,
            quota,
            true
        );
        auto greedy = findAllExactSegmentPathsToAnyGoal(
            grid,
            start,
            goals,
            segments,
            dynamic_step_limit,
            quota,
            false
        );

        std::size_t paths_f_uni = backward_search ? 0 : uniform.size();
        std::size_t paths_f_gre = backward_search ? 0 : greedy.size();
        std::size_t paths_b_uni = backward_search ? uniform.size() : 0;
        std::size_t paths_b_gre = backward_search ? greedy.size() : 0;
        std::size_t total_f = paths_f_uni + paths_f_gre;
        std::size_t total_b = paths_b_uni + paths_b_gre;
        std::cout << "          Found " << (total_f + total_b) << " paths in total\n"
                  << "              (forward-uniform:" << paths_f_uni << ", forward-greedy:" << paths_f_gre
                  << " | backward-uniform:" << paths_b_uni << ", backward-greedy:" << paths_b_gre << ")" << std::endl;

        all_paths.insert(all_paths.end(), uniform.begin(), uniform.end());
        all_paths.insert(all_paths.end(), greedy.begin(), greedy.end());
    }

    auto path_less = [](const auto& a, const auto& b) {
        if (a.size() != b.size()) {
            return a.size() < b.size();
        }
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (a[i].x != b[i].x) {
                return a[i].x < b[i].x;
            }
            if (a[i].y != b[i].y) {
                return a[i].y < b[i].y;
            }
            if (a[i].z != b[i].z) {
                return a[i].z < b[i].z;
            }
        }
        return false;
    };
    auto path_equal = [](const auto& a, const auto& b) {
        if (a.size() != b.size()) {
            return false;
        }
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (!samePoint(a[i], b[i])) {
                return false;
            }
        }
        return true;
    };
    std::sort(all_paths.begin(), all_paths.end(), path_less);
    all_paths.erase(std::unique(all_paths.begin(), all_paths.end(), path_equal), all_paths.end());
    std::sort(all_paths.begin(), all_paths.end(), [](const auto& a, const auto& b) {
        return routeLength(a) < routeLength(b);
    });
    if (all_paths.size() > max_results) {
        all_paths.resize(max_results);
    }
    return all_paths;
}

}  // namespace

Grid3D buildObstacleGridForNet(const RouteRequest& request, int net_id, double net_width, double clearance) {
    double grid_steps_per_mm = request.grid_steps_per_mm > 0.0 ? request.grid_steps_per_mm : kDefaultGridStepsPerMm;
    double pitch = 1.0 / grid_steps_per_mm;
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
        double track_clearance = std::max(clearance, track.clearance);
        markTrack(grid, track, net_width * 0.5 + track_clearance);
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
        grid.markPad(pad, bloat);
    }

    // Stage 2: open all grid vertices inside this net's pads plus this net's
    // clearance. This prevents the terminal area from being trapped by the
    // conservative bloat applied above.
    for (const auto& pad : request.pads) {
        if (pad.net_id == net_id) {
            grid.clearPad(pad, clearance);
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
            grid.markPad(pad, one_grid_guard);
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
    std::cout << "Rerouting net " << net_id << "..." << std::endl;
    double net_width = widthForNet(request, net_id);
    double clearance = clearanceForNet(request, net_id);
    Grid3D grid = buildObstacleGridForNet(request, net_id, net_width, clearance);

    result.net_id = net_id;
    result.grid_pitch = grid.pitch();
    result.origin_x = grid.origin_x();
    result.origin_y = grid.origin_y();
    result.nx = grid.nx();
    result.ny = grid.ny();
    result.nz = grid.nz();

    struct PadTerminals {
        std::vector<GridPoint> center_vertices;
        std::vector<GridPoint> goal_vertices;
    };
    std::vector<PadTerminals> pad_terminal_groups;
    for (const auto& pad : request.pads) {
        //只處理這條 net 的 pad
        if (pad.net_id != net_id) {
            continue;
        }
        PadTerminals terminals;
        //先決定這顆 pad 的 center 要取哪一層
        int preferred_z = preferredPadStartLayer(request, pad, net_id, grid);
        for (int z = 0; z < grid.nz(); ++z) {
            if (!layerMatchesPad(pad, grid.layers()[z])) {
                continue;
            }
            //如果這一層是 preferred_z，就把 pad center 放進 start 用的 center_vertices
            if (z == preferred_z) {
                auto center = grid.physicalToGrid(pad.center, z);
                if (grid.inBounds(center)) {
                    terminals.center_vertices.push_back(center);
                }
            }
            auto vertices = grid.verticesInsidePad(pad, std::max(grid.pitch() * 0.25, net_width * 0.25), z);
            for (const auto& vertex : vertices) {
                if (!grid.isBlocked(vertex)) {
                    terminals.goal_vertices.push_back(vertex);
                }
            }
        }
        result.terminal_group_sizes.push_back(static_cast<int>(terminals.goal_vertices.size()));
        if (!terminals.center_vertices.empty() && !terminals.goal_vertices.empty()) {
            pad_terminal_groups.push_back(std::move(terminals));
        }
    }

    if (pad_terminal_groups.size() < 2) {
        result.failure_reason = "Fewer than two pads have usable terminal grid vertices.";
        return result;
    }

    result.start_vertices = pad_terminal_groups[0].center_vertices;
    result.goal_vertices = pad_terminal_groups[1].goal_vertices;

    std::vector<std::vector<GridPoint>> candidate_paths;
    for (const auto& start : result.start_vertices) { // if center of start has multi layer
        auto paths = generateCandidatePaths(grid, start, result.goal_vertices, 12);
        candidate_paths.insert(candidate_paths.end(), paths.begin(), paths.end());
    }
    for (const auto& backward_start : pad_terminal_groups[1].center_vertices) {
        auto paths = generateCandidatePaths(grid, backward_start, pad_terminal_groups[0].goal_vertices, 12, true);
        for (auto& path : paths) {
            std::reverse(path.begin(), path.end());
            candidate_paths.push_back(std::move(path));
        }
    }

    auto path_equal = [](const auto& a, const auto& b) {
        if (a.size() != b.size()) {
            return false;
        }
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (!samePoint(a[i], b[i])) {
                return false;
            }
        }
        return true;
    };
    std::sort(candidate_paths.begin(), candidate_paths.end(), [](const auto& a, const auto& b) {
        return routeLength(a) < routeLength(b);
    });
    candidate_paths.erase(std::unique(candidate_paths.begin(), candidate_paths.end(), path_equal), candidate_paths.end());
    if (candidate_paths.size() > 1000) {
        candidate_paths.resize(1000);
    }

    if (candidate_paths.empty()) {
        result.failure_reason = "The exact-segment candidate router did not find a path to the goal pad.";
        return result;
    }

    result.found = true;
    result.failure_reason.clear();
    result.candidate_paths_grid = candidate_paths;
    for (const auto& path : candidate_paths) {
        std::vector<Point2D> physical_path;
        physical_path.reserve(path.size());
        for (const auto& point : path) {
            physical_path.push_back(grid.gridToPhysical(point));
        }
        result.candidate_paths_mm.push_back(std::move(physical_path));
    }
    result.path_grid = result.candidate_paths_grid.front();
    result.path_mm = result.candidate_paths_mm.front();
    return result;
}

}  // namespace interactive_router
