#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <vector>

namespace interactive_router {

// Compact 64-bit bitmap with Roaring-style high/low containers.
//
// The implementation intentionally keeps only sorted array containers. That is
// enough for the selector workload because vertices are sparse and mostly used
// for append, membership, ordered iteration, and size checks. The high 32 bits
// select a container and the low 32 bits are stored once inside that container.
class Roaring64Bitmap {
public:
    using Value = std::uint64_t;

    void add(Value value) {
        const std::uint32_t high = highBits(value);
        const std::uint32_t low = lowBits(value);
        containers_[high].push_back(low);
        normalized_ = false;
    }

    bool contains(Value value) const {
        normalize();
        auto found = containers_.find(highBits(value));
        if (found == containers_.end()) {
            return false;
        }
        const auto& values = found->second;
        return std::binary_search(values.begin(), values.end(), lowBits(value));
    }

    bool empty() const {
        normalize();
        return size_ == 0;
    }

    std::size_t size() const {
        normalize();
        return size_;
    }

    void clear() {
        containers_.clear();
        size_ = 0;
        normalized_ = true;
    }

    std::vector<Value> values() const {
        normalize();
        std::vector<Value> out;
        out.reserve(size_);
        forEach([&out](Value value) {
            out.push_back(value);
        });
        return out;
    }

    template <typename Callback>
    void forEach(Callback&& callback) const {
        normalize();
        for (const auto& entry : containers_) {
            const std::uint64_t high = static_cast<std::uint64_t>(entry.first) << 32;
            for (std::uint32_t low : entry.second) {
                callback(high | static_cast<std::uint64_t>(low));
            }
        }
    }

private:
    static std::uint32_t highBits(Value value) {
        return static_cast<std::uint32_t>(value >> 32);
    }

    static std::uint32_t lowBits(Value value) {
        return static_cast<std::uint32_t>(value & 0xffffffffULL);
    }

    void normalize() const {
        if (normalized_) {
            return;
        }
        size_ = 0;
        for (auto& entry : containers_) {
            auto& values = entry.second;
            std::sort(values.begin(), values.end());
            values.erase(std::unique(values.begin(), values.end()), values.end());
            size_ += values.size();
        }
        normalized_ = true;
    }

    mutable std::map<std::uint32_t, std::vector<std::uint32_t>> containers_;
    mutable std::size_t size_ = 0;
    mutable bool normalized_ = true;
};

}  // namespace interactive_router
