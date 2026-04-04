#pragma once

#include <atomic>
#include <vector>
#include <cstddef>

template<typename T>
class SPSCQueue {
public:
    explicit SPSCQueue(size_t capacity) : capacity_(capacity + 1), buffer_(capacity + 1) {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    bool push(const T& item) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t next_h = (h + 1) % capacity_;
        if (next_h == tail_.load(std::memory_order_acquire)) {
            return false; // Queue is full
        }
        buffer_[h] = item;
        head_.store(next_h, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) {
            return false; // Queue is empty
        }
        item = buffer_[t];
        tail_.store((t + 1) % capacity_, std::memory_order_release);
        return true;
    }

private:
    size_t capacity_;
    std::vector<T> buffer_;
    // Cache line padding to prevent false sharing
    alignas(64) std::atomic<size_t> head_; // written by producer
    alignas(64) std::atomic<size_t> tail_; // written by consumer
};
