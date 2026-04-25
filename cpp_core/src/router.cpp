#include "router.h"
#include "net_analysis.h"


#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <map>
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


bool isValidFinalMove(const GridPoint& curr, const GridPoint& goal, int prev_dx, int prev_dy, int prev_dz);


bool isClearFinalMove(
   const Grid3D& grid,
   const GridPoint& curr,
   const GridPoint& goal,
   const std::unordered_set<std::size_t>& goal_indices
);


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
   int best_goal_segments = kInfSegments;


   std::size_t start_state = stateIndex(start, kNoDirection);
   dist[start_state] = 0;
   queue.push_front({start, kNoDirection});


   while (!queue.empty()) {
       auto [current, prev_dir_index] = queue.front();
       queue.pop_front();
       int current_segments = dist[stateIndex(current, prev_dir_index)];


       if (isGoalPoint(grid, current, goal_indices)) {
           best_goal_segments = std::min(best_goal_segments, current_segments);
           continue;
       }
       if (current_segments >= best_goal_segments) {
           continue;
       }


       int prev_dx = 0;
       int prev_dy = 0;
       int prev_dz = 0;
       if (prev_dir_index != kNoDirection) {
           prev_dx = kDijkstraDeltas[prev_dir_index][0];
           prev_dy = kDijkstraDeltas[prev_dir_index][1];
           prev_dz = kDijkstraDeltas[prev_dir_index][2];
       }


       for (const auto& goal : goal_vertices) {
           if (!isValidFinalMove(current, goal, prev_dx, prev_dy, prev_dz)) {
               continue;
           }
           if (!isClearFinalMove(grid, current, goal, goal_indices)) {
               continue;
           }


           int final_dir_index = kNoDirection;
           int final_dx = goal.x - current.x;
           int final_dy = goal.y - current.y;
           int final_dz = goal.z - current.z;
           if (final_dz != 0) {
               final_dir_index = directionIndexForDelta(0, 0, final_dz > 0 ? 1 : -1);
           } else if (final_dx != 0 || final_dy != 0) {
               int gcd = std::gcd(std::abs(final_dx), std::abs(final_dy));
               final_dir_index = directionIndexForDelta(final_dx / gcd, final_dy / gcd, 0);
           }


           int extra_segment = (prev_dir_index != kNoDirection && prev_dir_index == final_dir_index) ? 0 : 1;
           best_goal_segments = std::min(best_goal_segments, current_segments + extra_segment);
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


   return best_goal_segments == kInfSegments ? -1 : best_goal_segments;
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
        return false;
    }
    int abs_dx = std::abs(final_dx);
    int abs_dy = std::abs(final_dy);
    if (abs_dx != 0 && abs_dy != 0 && abs_dx != abs_dy) {
        return false;
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
    if (final_dz != 0) {
        return false;
    }
    return isLineOfSightClear(grid, curr.x, curr.y, curr.z, goal.x, goal.y, goal_indices);
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


struct DfsState {
   GridPoint curr;
   int depth;
   std::vector<GridPoint> path;
};


struct SegmentBucketState {
   int segment = 0;
   std::size_t found_count = 0;
   double hp = 0.0;
};


double clampValue(double value, double low, double high) {
   return std::max(low, std::min(high, value));
}


std::vector<int> activeBucketIndices(
   const std::vector<SegmentBucketState>& buckets,
   std::size_t max_results,
   int min_segment
) {
   std::vector<int> active;
   for (std::size_t index = 0; index < buckets.size(); ++index) {
       if (buckets[index].segment < min_segment) {
           continue;
       }
       if (buckets[index].found_count >= max_results) {
           continue;
       }
       if (buckets[index].hp <= 0.0) {
           continue;
       }
       active.push_back(static_cast<int>(index));
   }
   return active;
}


double alphaForActiveBuckets(const std::vector<SegmentBucketState>& buckets, const std::vector<int>& active, std::size_t max_results) {
   constexpr double kInitialAlphaBias = 0.7;
   constexpr double kSmallSegmentBiasPower = 1.0;
   if (active.empty()) {
       return kInitialAlphaBias;
   }
   int min_target = buckets[active.front()].segment;
   int max_target = buckets[active.back()].segment;
   if (min_target == max_target) {
       return 1.0;
   }


   double weighted_sum = 0.0;
   double total_weight = 0.0;
   for (int index : active) {
       double need = static_cast<double>(max_results - buckets[index].found_count);
       if (need <= 0.0) {
           continue;
       }
       double relative_index = static_cast<double>(buckets[index].segment - min_target + 1);
       double bias = 1.0 / std::pow(relative_index, kSmallSegmentBiasPower);
       double weight = need * bias;
       weighted_sum += static_cast<double>(buckets[index].segment) * weight;
       total_weight += weight;
   }
   if (total_weight <= 0.0) {
       return kInitialAlphaBias;
   }


   double center = weighted_sum / total_weight;
   double alpha_raw = static_cast<double>(max_target) - center;
   alpha_raw /= static_cast<double>(max_target - min_target);
   double alpha = kInitialAlphaBias + (alpha_raw - 0.5);
   return clampValue(alpha, 0.15, 0.85);
}


void applyCostShare(
   std::vector<SegmentBucketState>& buckets,
   const std::vector<int>& active,
   std::size_t max_results
) {
   double total_weight = 0.0;
   for (int index : active) {
       total_weight += static_cast<double>(max_results - buckets[index].found_count);
   }
   if (total_weight <= 0.0) {
       return;
   }
   for (int index : active) {
       double weight = static_cast<double>(max_results - buckets[index].found_count);
       buckets[index].hp -= weight / total_weight;
   }
}


bool hasRemainingDemand(const std::vector<SegmentBucketState>& buckets, std::size_t max_results) {
   return std::any_of(buckets.begin(), buckets.end(), [max_results](const SegmentBucketState& bucket) {
       return bucket.found_count < max_results;
   });
}


bool pathAlreadyCollected(
   const std::vector<std::vector<GridPoint>>& bucket_paths,
   const std::vector<GridPoint>& candidate
) {
   for (const auto& existing : bucket_paths) {
       if (existing.size() != candidate.size()) {
           continue;
       }
       bool same = true;
       for (std::size_t i = 0; i < existing.size(); ++i) {
           if (!samePoint(existing[i], candidate[i])) {
               same = false;
               break;
           }
       }
       if (same) {
           return true;
       }
   }
   return false;
}


std::vector<std::vector<GridPoint>> dfsRangeSegmentPathsToAnyGoal(
   const Grid3D& grid,
   const GridPoint& initial_curr,
   const std::vector<GridPoint>& goal_vertices,
   const std::unordered_set<std::size_t>& goal_indices,
   int initial_depth,
   std::vector<GridPoint> initial_path,
   int min_target_segments,
   int max_target_segments,
   int dynamic_step_limit,
   std::size_t max_results,
   bool uniform_heuristic,
   unsigned int heuristic_seed_salt,
   bool debug_logs
) {
   if (min_target_segments <= 0 || min_target_segments > max_target_segments || goal_vertices.empty()) {
       return {};
   }


   std::vector<SegmentBucketState> buckets;
   std::vector<std::vector<std::vector<GridPoint>>> bucket_paths;
   buckets.reserve(static_cast<std::size_t>(max_target_segments - min_target_segments + 1));
   bucket_paths.resize(static_cast<std::size_t>(max_target_segments - min_target_segments + 1));
   for (int segment = min_target_segments; segment <= max_target_segments; ++segment) {
       buckets.push_back({segment, 0, static_cast<double>(dynamic_step_limit)});
   }


   const int full_hp = dynamic_step_limit;
   int remaining_steps = dynamic_step_limit;
   int last_logged_min_target = -1;
   int last_logged_max_target = -1;
   int debug_log_budget = debug_logs ? 24 : 0;
   std::vector<DfsState> stack;
   stack.reserve(1000);
   stack.push_back({initial_curr, initial_depth, std::move(initial_path)});


   while (!stack.empty()) {
       if (!hasRemainingDemand(buckets, max_results)) {
           break;
       }
       if (--remaining_steps <= 0) {
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


       int completed_segments = state.depth + 1;
       if (completed_segments >= min_target_segments && completed_segments <= max_target_segments) {
           std::size_t bucket_index = static_cast<std::size_t>(completed_segments - min_target_segments);
           if (buckets[bucket_index].found_count < max_results) {
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
                   if (pathAlreadyCollected(bucket_paths[bucket_index], finished)) {
                       continue;
                   }
                   bucket_paths[bucket_index].push_back(finished);
                   ++buckets[bucket_index].found_count;
                   buckets[bucket_index].hp = static_cast<double>(full_hp);
                   remaining_steps = full_hp;
                   if (debug_log_budget > 0) {
                       std::cout << "            bucket " << buckets[bucket_index].segment
                                 << " found path " << buckets[bucket_index].found_count
                                 << "/" << max_results << " hp reset" << std::endl;
                       --debug_log_budget;
                   }
                   if (buckets[bucket_index].found_count >= max_results) {
                       break;
                   }
               }
           }
       }


       if (state.depth >= max_target_segments - 1) {
           continue;
       }


       auto active = activeBucketIndices(buckets, max_results, state.depth + 2);
       if (active.empty()) {
           continue;
       }
       std::vector<double> hp_before(buckets.size(), 0.0);
       for (int index : active) {
           hp_before[static_cast<std::size_t>(index)] = buckets[static_cast<std::size_t>(index)].hp;
       }
       applyCostShare(buckets, active, max_results);
       std::vector<int> drained_indices;
       for (int index : active) {
           std::size_t bucket_index = static_cast<std::size_t>(index);
           if (hp_before[bucket_index] > 0.0 && buckets[bucket_index].hp <= 0.0
               && buckets[bucket_index].found_count < max_results) {
               drained_indices.push_back(index);
           }
       }
       if (!drained_indices.empty()) {
           int removed_index = *std::min_element(
               drained_indices.begin(),
               drained_indices.end(),
               [&](int a, int b) {
                   return buckets[static_cast<std::size_t>(a)].segment < buckets[static_cast<std::size_t>(b)].segment;
               }
           );
           for (std::size_t index = 0; index < buckets.size(); ++index) {
               if (static_cast<int>(index) == removed_index) {
                   continue;
               }
               if (buckets[index].found_count < max_results) {
                   buckets[index].hp = static_cast<double>(full_hp);
               }
           }
           remaining_steps = full_hp;
           if (debug_log_budget > 0) {
               std::cout << "            bucket " << buckets[static_cast<std::size_t>(removed_index)].segment
                         << " became inactive after hp drain; restored all other unfinished buckets" << std::endl;
               --debug_log_budget;
           }
       }
       active = activeBucketIndices(buckets, max_results, state.depth + 2);
       if (active.empty()) {
           if (debug_log_budget > 0) {
               std::cout << "            all active buckets drained at depth " << state.depth << std::endl;
               --debug_log_budget;
           }
           continue;
       }


       int target_segments = buckets[active.front()].segment;
       int max_target = buckets[active.back()].segment;
       if (debug_log_budget > 0 && (target_segments != last_logged_min_target || max_target != last_logged_max_target)) {
           std::cout << "            active range " << target_segments << ".." << max_target
                     << " target=" << target_segments << " depth=" << state.depth << std::endl;
           last_logged_min_target = target_segments;
           last_logged_max_target = max_target;
           --debug_log_budget;
       }
       if (debug_log_budget > 0 && drained_indices.empty()) {
           for (std::size_t index = 0; index < buckets.size(); ++index) {
               if (hp_before[index] > 0.0 && buckets[index].hp <= 0.0 && buckets[index].found_count < max_results) {
                   std::cout << "            bucket " << buckets[index].segment
                             << " became inactive after hp drain at depth " << state.depth << std::endl;
                   --debug_log_budget;
                   if (debug_log_budget <= 0) {
                       break;
                   }
               }
           }
       }
       GridPoint guide_goal = sortedGoalVertices(goal_vertices, curr).front();
       std::vector<int> score_targets;
       score_targets.reserve(active.size());
       for (int index : active) {
           score_targets.push_back(buckets[static_cast<std::size_t>(index)].segment);
       }


       auto next_points = castRays360(grid, curr, goal_indices, prev_dx, prev_dy, prev_dz);
       auto score = [&](const GridPoint& p) {
           double base = std::numeric_limits<double>::infinity();
           for (int target : score_targets) {
               double progress = static_cast<double>(state.depth) / std::max(1, target - 1);
               double curve_factor = 4.0 * (progress - 0.5) * (progress - 0.5);
               double dynamic_via_penalty = kViaPenalty * (curve_factor * (1.0 - kMinDiscount) + kMinDiscount);
               double tx = 0.0;
               double ty = 0.0;
               double tz = 0.0;
               if (uniform_heuristic) {
                   int remaining_segments = std::max(1, target - state.depth);
                   tx = curr.x + (guide_goal.x - curr.x) / static_cast<double>(remaining_segments);
                   ty = curr.y + (guide_goal.y - curr.y) / static_cast<double>(remaining_segments);
                   tz = path.front().z + (guide_goal.z - path.front().z) * progress;
               } else {
                   tx = guide_goal.x;
                   ty = guide_goal.y;
                   tz = guide_goal.z;
               }
               double dx = p.x - tx;
               double dy = p.y - ty;
               double dz = p.z - tz;
               double target_base = dx * dx + dy * dy + dz * dz * dynamic_via_penalty;
               if (target_base < base) {
                   base = target_base;
               }
           }
           unsigned int noise_seed = kRandomSeed
               ^ heuristic_seed_salt
               ^ static_cast<unsigned int>((state.depth + 1) * 1234567U)
               ^ (uniform_heuristic ? 0x13579BDFU : 0x9e3779b9U);
           return base + (base + 1.0) * spatialNoise(p.x, p.y, p.z, noise_seed);
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


       std::size_t branch_limit = state.depth == 0 ? std::min<std::size_t>(next_points.size(), 48) : next_points.size();
       for (std::size_t i = 0; i < branch_limit; ++i) {
           const auto& next = next_points[i];
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


   std::vector<std::vector<GridPoint>> paths;
   for (const auto& bucket : bucket_paths) {
       paths.insert(paths.end(), bucket.begin(), bucket.end());
   }
   std::mt19937 rng(kShuffleSeed ^ static_cast<unsigned int>(uniform_heuristic ? 1U : 2U));
   std::shuffle(paths.begin(), paths.end(), rng);
   return paths;
}


std::vector<std::vector<GridPoint>> findAllExactSegmentPathsToAnyGoal(
   const Grid3D& grid,
   const GridPoint& start,
   const std::vector<GridPoint>& goal_vertices,
   int min_target_segments,
   int max_target_segments,
   int dynamic_step_limit,
   std::size_t max_results,
   bool uniform_heuristic,
   unsigned int heuristic_seed_salt
) {
   if (min_target_segments <= 0 || min_target_segments > max_target_segments || goal_vertices.empty()) {
       return {};
   }


   std::unordered_set<std::size_t> goal_indices;
   for (const auto& goal : goal_vertices) {
       if (grid.inBounds(goal)) {
           goal_indices.insert(grid.flatten(goal));
       }
   }


   std::vector<std::vector<GridPoint>> paths;
   if (min_target_segments <= 1) {
       for (const auto& goal : sortedGoalVertices(goal_vertices, start)) {
           if (isValidFinalMove(start, goal, 0, 0, 0) && isClearFinalMove(grid, start, goal, goal_indices)) {
               paths.push_back({start, goal});
               if (paths.size() >= max_results) {
                   break;
               }
           }
       }
   }


   auto first_points = castRays360(grid, start, goal_indices, 0, 0, 0);
   GridPoint guide_goal = sortedGoalVertices(goal_vertices, start).front();
   auto score = [&](const GridPoint& p) {
       double base = std::numeric_limits<double>::infinity();
       for (int target_segment = std::max(1, min_target_segments); target_segment <= max_target_segments; ++target_segment) {
           double progress = 0.0;
           int remaining = std::max(1, target_segment);
           double tx = 0.0;
           double ty = 0.0;
           double tz = 0.0;
           if (uniform_heuristic) {
               tx = start.x + (guide_goal.x - start.x) / static_cast<double>(remaining);
               ty = start.y + (guide_goal.y - start.y) / static_cast<double>(remaining);
               tz = start.z + (guide_goal.z - start.z) * progress;
           } else {
               tx = guide_goal.x;
               ty = guide_goal.y;
               tz = guide_goal.z;
           }
           double dx = p.x - tx;
           double dy = p.y - ty;
           double dz = p.z - tz;
           double target_base = dx * dx + dy * dy + dz * dz * kViaPenalty;
           if (target_base < base) {
               base = target_base;
           }
       }
       unsigned int base_seed = uniform_heuristic ? kRandomSeed : (kRandomSeed ^ 0x9e3779b9U);
       return base + (base + 1.0) * spatialNoise(p.x, p.y, p.z, base_seed ^ heuristic_seed_salt);
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
   std::vector<std::future<std::vector<std::vector<GridPoint>>>> futures;
   futures.reserve(branch_limit);
   for (std::size_t i = 0; i < branch_limit; ++i) {
       GridPoint next = first_points[i];
       if (isGoalPoint(grid, next, goal_indices)) {
           continue;
       }
       bool debug_branch = i == 0;
       futures.push_back(std::async(std::launch::async, [&, next, debug_branch]() {
           return dfsRangeSegmentPathsToAnyGoal(
               grid,
               next,
               goal_vertices,
               goal_indices,
               1,
               {start, next},
               min_target_segments,
               max_target_segments,
               dynamic_step_limit,
               max_results,
               uniform_heuristic,
               heuristic_seed_salt,
               debug_branch
           );
       }));
   }


   for (auto& future : futures) {
       auto branch_paths = future.get();
       paths.insert(paths.end(), branch_paths.begin(), branch_paths.end());
   }


   std::mt19937 rng(kShuffleSeed ^ static_cast<unsigned int>(uniform_heuristic ? 1U : 2U));
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


void printSegmentPathHistogram(const std::vector<std::vector<GridPoint>>& paths, const char* label) {
   std::map<int, int> histogram;
   for (const auto& path : paths) {
       int segments = path.size() > 1 ? static_cast<int>(path.size()) - 1 : 0;
       ++histogram[segments];
   }

   std::cout << "  " << label << " segment/path counts:" << std::endl;
   if (histogram.empty()) {
       std::cout << "      (none)" << std::endl;
       return;
   }
   for (const auto& [segments, count] : histogram) {
       std::cout << "      " << segments << " segments: " << count << " paths" << std::endl;
   }
}


std::vector<std::vector<GridPoint>> generateCandidatePaths(
   const Grid3D& grid,
   const GridPoint& start,
   const std::vector<GridPoint>& goals,
   std::size_t max_results,
   int max_candidate_segments,
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


   std::cout << "  minimum_segment_presearch " << minimum_segments << " segments" << std::endl;
   std::cout << "  max_candidate_segments " << max_candidate_segments << std::endl;
   if (minimum_segments > max_candidate_segments) {
       std::cout << "  minimum segments exceed max candidate segments " << max_candidate_segments << std::endl;
       return {};
   }


   std::cout << "      Searching range " << std::max(1, minimum_segments)
             << ".." << max_candidate_segments << " segments..." << std::endl;


   std::vector<std::vector<GridPoint>> uniform;
   std::vector<std::vector<GridPoint>> greedy;
   constexpr int kHeuristicSeedRounds = 4;
   for (int round = 0; round < kHeuristicSeedRounds; ++round) {
       unsigned int seed_salt = 0x9e3779b9U * static_cast<unsigned int>(round + 1);
       auto uniform_round = findAllExactSegmentPathsToAnyGoal(
           grid,
           start,
           goals,
           std::max(1, minimum_segments),
           max_candidate_segments,
           dynamic_step_limit,
           max_results,
           true,
           seed_salt
       );
       auto greedy_round = findAllExactSegmentPathsToAnyGoal(
           grid,
           start,
           goals,
           std::max(1, minimum_segments),
           max_candidate_segments,
           dynamic_step_limit,
           max_results,
           false,
           seed_salt
       );
       uniform.insert(uniform.end(), uniform_round.begin(), uniform_round.end());
       greedy.insert(greedy.end(), greedy_round.begin(), greedy_round.end());
       if (!uniform.empty() || !greedy.empty()) {
           break;
       }
   }


   std::size_t paths_f_uni = backward_search ? 0 : uniform.size();
   std::size_t paths_f_gre = backward_search ? 0 : greedy.size();
   std::size_t paths_b_uni = backward_search ? uniform.size() : 0;
   std::size_t paths_b_gre = backward_search ? greedy.size() : 0;
   std::size_t total_f = paths_f_uni + paths_f_gre;
   std::size_t total_b = paths_b_uni + paths_b_gre;
   std::map<int, int> segment_histogram_raw;
   auto collect_segment_histogram = [&](const std::vector<std::vector<GridPoint>>& paths) {
       for (const auto& path : paths) {
           int segments = path.size() > 1 ? static_cast<int>(path.size()) - 1 : 0;
           ++segment_histogram_raw[segments];
       }
   };
   collect_segment_histogram(uniform);
   collect_segment_histogram(greedy);
   std::cout << "          Found " << (total_f + total_b) << " paths in total\n"
             << "              (forward-uniform:" << paths_f_uni << ", forward-greedy:" << paths_f_gre
             << " | backward-uniform:" << paths_b_uni << ", backward-greedy:" << paths_b_gre << ")" << std::endl;
   std::cout << "          Raw segment/path counts:" << std::endl;
   if (segment_histogram_raw.empty()) {
       std::cout << "              (none)" << std::endl;
   } else {
       for (const auto& [segments, count] : segment_histogram_raw) {
           std::cout << "              " << segments << " segments: " << count << " paths" << std::endl;
       }
   }


   all_paths.insert(all_paths.end(), uniform.begin(), uniform.end());
   all_paths.insert(all_paths.end(), greedy.begin(), greedy.end());


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
   std::size_t total_result_cap = static_cast<std::size_t>(max_candidate_segments - std::max(1, minimum_segments) + 1) * max_results;
   if (all_paths.size() > total_result_cap) {
       all_paths.resize(total_result_cap);
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
           auto vertices = grid.verticesOnPadBoundary(pad, pad_entry_bloat, z);
           terminals.insert(terminals.end(), vertices.begin(), vertices.end());
       }
   }
   return terminals;
}

void blockGoalPadInteriorExceptBoundary(
   Grid3D& grid,
   const PadGeometry& goal_pad,
   const std::vector<GridPoint>& goal_boundary_vertices,
   double pad_entry_bloat
) {
   std::unordered_set<std::size_t> boundary_indices;
   boundary_indices.reserve(goal_boundary_vertices.size());
   for (const auto& vertex : goal_boundary_vertices) {
       if (grid.inBounds(vertex)) {
           boundary_indices.insert(grid.flatten(vertex));
       }
   }

   for (int z = 0; z < grid.nz(); ++z) {
       if (!layerMatchesPad(goal_pad, grid.layers()[z])) {
           continue;
       }
       auto inside_vertices = grid.verticesInsidePad(goal_pad, pad_entry_bloat, z);
       for (const auto& vertex : inside_vertices) {
           if (boundary_indices.find(grid.flatten(vertex)) == boundary_indices.end()) {
               grid.setBlocked(vertex, true);
           }
       }
   }
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
   // 最大segment數 ： net原本的繞線的segment數＋ min(1+ board layer數, 5)
   int original_segment_count = originalRouteSegmentCount(request, net_id);
   int segment_margin = std::min(1 + grid.nz(), 5);
   int max_candidate_segments = std::max(1, original_segment_count + segment_margin);
   std::cout << "  original_route_segments " << original_segment_count << std::endl;
   std::cout << "  segment_margin " << segment_margin << std::endl;
   std::cout << "  dynamic_max_candidate_segments " << max_candidate_segments << std::endl;


   result.net_id = net_id;
   result.grid_pitch = grid.pitch();
   result.origin_x = grid.origin_x();
   result.origin_y = grid.origin_y();
   result.nx = grid.nx();
   result.ny = grid.ny();
   result.nz = grid.nz();


   struct PadTerminals {
       const PadGeometry* pad = nullptr;
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
       terminals.pad = &pad;
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
           auto vertices = grid.verticesOnPadBoundary(pad, std::max(grid.pitch() * 0.25, net_width * 0.25), z);
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
   double pad_entry_bloat = std::max(grid.pitch() * 0.25, net_width * 0.25);
   for (const auto& start : result.start_vertices) { // if center of start has multi layer
       Grid3D search_grid = grid;
       blockGoalPadInteriorExceptBoundary(
           search_grid,
           *pad_terminal_groups[1].pad,
           result.goal_vertices,
           pad_entry_bloat
       );
       auto paths = generateCandidatePaths(search_grid, start, result.goal_vertices, 100, max_candidate_segments);
       candidate_paths.insert(candidate_paths.end(), paths.begin(), paths.end());
   }
   for (const auto& backward_start : pad_terminal_groups[1].center_vertices) {
       Grid3D search_grid = grid;
       blockGoalPadInteriorExceptBoundary(
           search_grid,
           *pad_terminal_groups[0].pad,
           pad_terminal_groups[0].goal_vertices,
           pad_entry_bloat
       );
       auto paths = generateCandidatePaths(
           search_grid,
           backward_start,
           pad_terminal_groups[0].goal_vertices,
           100,
           max_candidate_segments,
           true
       );
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
       candidate_paths.resize(1000);//
   }
   printSegmentPathHistogram(candidate_paths, "Final candidate");


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
