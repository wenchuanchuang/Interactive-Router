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
        //?暺?憟賢?亙璇?track
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
        //?券暺?湧??航?敶?
        if (!isStraightThroughAtJunction(first, second, junction)) {
            continue;
        }
        //???砍璇?track 閬???嗾雿?segment嚗?隞?count 皜?1
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

// Check whether a physical point falls inside the pad geometry.
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

//憒???韏琿? pad ??撅?center ?臭誑?嗉絲暺????閰脤?芯?撅斤 start center
// 憒?韏琿? pad ??撅?center
// ?芸? 銝??center_vertex?嗉絲暺?
// ?芸??詻??祇?net ??track endpoint ?賢 pad 鋆∠????撅扎?
// 憒??蝜?銝??冽??斗銝靘?撠梢???銝惜?舐 layer ??center
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
        //瑼Ｘ?? track ??layer ?臭???pad ?舀??layer
        if (!layerMatchesPad(pad, track.layer)) {
            continue;
        }
        //瑼Ｘ?? track ??start/end ?????pad 鋆?
        if (pointInsidePad(pad, track.start) || pointInsidePad(pad, track.end)) {
            // ??? pad ????臬??layer 銝???亙??
            int z = grid.layerIndex(track.layer);
            if (z >= 0) {
                return z;
            }
        }
    }

    //憒??游?net ?賣銝 track endpoint ?賢 pad 鋆? ??pad ??銝惜?舐 layer
    for (int z = 0; z < grid.nz(); ++z) {
        if (layerMatchesPad(pad, grid.layers()[z])) {
            return z;
        }
    }
    return -1;
}

}  // namespace interactive_router
