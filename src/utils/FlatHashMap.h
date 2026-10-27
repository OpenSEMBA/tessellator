#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace meshlib {
namespace utils {

// Simple open-addressing hash map with linear probing. It offers the same
// find/emplace surface used throughout the meshers while avoiding the per-node
// allocations and pointer chasing of std::unordered_map, which dominates in
// tight loops over grid cells. Keys must be equality comparable and default
// constructible.
template<class Key, class Value, class Hash>
class FlatHashMap {
public:
    FlatHashMap() = default;

    explicit FlatHashMap(std::size_t expectedElements)
    {
        reserve(expectedElements);
    }

    void reserve(std::size_t expectedElements)
    {
        std::size_t capacity = 16;
        while (capacity < expectedElements * 2) {
            capacity <<= 1;
        }
        rehash(capacity);
    }

    Value* find(const Key& key)
    {
        if (capacity_ == 0) {
            return nullptr;
        }
        std::size_t index = bucket(key);
        while (used_[index] != 0) {
            if (keys_[index] == key) {
                return &values_[index];
            }
            index = (index + 1) & mask_;
        }
        return nullptr;
    }

    const Value* find(const Key& key) const
    {
        return const_cast<FlatHashMap*>(this)->find(key);
    }

    std::size_t size() const
    {
        return count_;
    }

    template<class... Args>
    std::pair<Value*, bool> emplace(const Key& key, Args&&... args)
    {
        if (capacity_ == 0 || count_ + 1 > maxLoad()) {
            rehash(capacity_ == 0 ? 16 : capacity_ * 2);
        }
        std::size_t index = bucket(key);
        while (used_[index] != 0) {
            if (keys_[index] == key) {
                return { &values_[index], false };
            }
            index = (index + 1) & mask_;
        }
        keys_[index] = key;
        values_[index] = Value(std::forward<Args>(args)...);
        used_[index] = 1;
        ++count_;
        return { &values_[index], true };
    }

private:
    std::size_t maxLoad() const
    {
        return capacity_ * 7 / 10;
    }

    // Scrambles the hash so that the low bits used by the power-of-two mask
    // stay well distributed even when the user hash is weak.
    static std::size_t mix(std::size_t h)
    {
        if constexpr (sizeof(std::size_t) >= 8) {
            h ^= h >> 33;
            h *= static_cast<std::size_t>(0xff51afd7ed558ccdULL);
            h ^= h >> 33;
        }
        else {
            h ^= h >> 16;
            h *= static_cast<std::size_t>(0x7feb352dU);
            h ^= h >> 15;
            h *= static_cast<std::size_t>(0x846ca68bU);
            h ^= h >> 16;
        }
        return h;
    }

    std::size_t bucket(const Key& key) const
    {
        return mix(Hash{}(key)) & mask_;
    }

    void rehash(std::size_t capacity)
    {
        std::vector<Key> oldKeys = std::move(keys_);
        std::vector<Value> oldValues = std::move(values_);
        std::vector<char> oldUsed = std::move(used_);
        const std::size_t oldCapacity = capacity_;

        keys_.assign(capacity, Key{});
        values_.assign(capacity, Value{});
        used_.assign(capacity, 0);
        capacity_ = capacity;
        mask_ = capacity - 1;
        count_ = 0;

        for (std::size_t i = 0; i < oldCapacity; ++i) {
            if (oldUsed[i] != 0) {
                emplace(oldKeys[i], std::move(oldValues[i]));
            }
        }
    }

    std::vector<Key> keys_;
    std::vector<Value> values_;
    std::vector<char> used_;
    std::size_t capacity_ = 0;
    std::size_t mask_ = 0;
    std::size_t count_ = 0;
};

} // namespace utils
} // namespace meshlib
