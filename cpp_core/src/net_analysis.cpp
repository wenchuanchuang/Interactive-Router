#include "net_analysis.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace interactive_router {
namespace {

struct PointKey {
    std::int64_t x = 0;
    std::int64_t y = 0;

    bool operator==(const PointKey& other) const {
        return x == other.x && y == other.y;
    }
};

struct PointKeyHasher {
    std::size_t operator()(const PointKey& key) const {
        return std::hash<std::int64_t>{}(key.x) ^ (std::hash<std::int64_t>{}(key.y) << 1);
    }
};

PointKey makePointKey(const Point2D& point) {
    constexpr double kCoordinateScale = 1'000'000.0;
    return {
        static_cast<std::int64_t>(std::llround(point.x * kCoordinateScale)),
        static_cast<std::int64_t>(std::llround(point.y * kCoordinateScale)),
    };
}

bool samePhysicalPoint(const Point2D& a, const Point2D& b, double epsilon = 1e-9) {
    return std::abs(a.x - b.x) <= epsilon && std::abs(a.y - b.y) <= epsilon;
}

bool sameScalar(double a, double b, double epsilon = 1e-9) {
    return std::abs(a - b) <= epsilon;
}

bool isStraightThroughAtJunction(const TrackGeometry& first, const TrackGeometry& second, const Point2D& junction) {
    Point2D first_other = samePhysicalPoint(first.start, junction) ? first.end : first.start;
    Point2D second_other = samePhysicalPoint(second.start, junction) ? second.end : second.start;
    double v1x = first_other.x - junction.x;
    double v1y = first_other.y - junction.y;
    double v2x = second_other.x - junction.x;
    double v2y = second_other.y - junction.y;
    double cross = v1x * v2y - v1y * v2x;
    double dot = v1x * v2x + v1y * v2y;
    return std::abs(cross) <= 1e-9 && dot < 0.0;
}

}  // namespace

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

int originalRouteSegmentCount(const RouteRequest& request, int net_id) {
    std::vector<const TrackGeometry*> net_tracks;
    net_tracks.reserve(request.tracks.size());
    for (const auto& track : request.tracks) {
        if (track.net_id != net_id) {
            continue;
        }
        if (samePhysicalPoint(track.start, track.end)) {
            continue;
        }
        net_tracks.push_back(&track);
    }

    int count = static_cast<int>(net_tracks.size());
    std::unordered_map<PointKey, std::vector<int>, PointKeyHasher> incident_tracks_by_point;
    incident_tracks_by_point.reserve(net_tracks.size() * 2);

    for (std::size_t index = 0; index < net_tracks.size(); ++index) {
        incident_tracks_by_point[makePointKey(net_tracks[index]->start)].push_back(static_cast<int>(index));
        incident_tracks_by_point[makePointKey(net_tracks[index]->end)].push_back(static_cast<int>(index));
    }

    for (const auto& [junction_key, incident_indices] : incident_tracks_by_point) {
        (void) junction_key;
        //某個接點剛好只接兩條 track
        if (incident_indices.size() != 2) {
            continue;
        }
        const TrackGeometry& first = *net_tracks[incident_indices[0]];
        const TrackGeometry& second = *net_tracks[incident_indices[1]];
        if (first.layer != second.layer || !sameScalar(first.width, second.width)) {
            continue;
        }

        Point2D junction = samePhysicalPoint(first.start, second.start) || samePhysicalPoint(first.start, second.end)
            ? first.start
            : first.end;
        //在這個接點是直通、不是轉彎
        if (!isStraightThroughAtJunction(first, second, junction)) {
            continue;
        }
        //把原本兩條 track 視為同一個幾何 segment，所以 count 減 1
        --count;
    }

    for (const auto& via : request.vias) {
        if (via.net_id == net_id) {
            ++count;
        }
    }
    return count;
}

bool layerMatchesPad(const PadGeometry& pad, const std::string& layer) {
    return std::find(pad.layers.begin(), pad.layers.end(), layer) != pad.layers.end()
        || std::find(pad.layers.begin(), pad.layers.end(), "*.Cu") != pad.layers.end();
}

//某個實體座標 point，有沒有落在 pad 幾何範圍裡
bool pointInsidePad(const PadGeometry& pad, const Point2D& point, double bloat) {
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

}  // namespace interactive_router
