#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/** Bounded FIFO between an asynchronous transport and a non-blocking CommandSource. */
template <typename Request>
class NonLockstepRequestQueue final {
public:
    explicit NonLockstepRequestQueue(std::size_t capacity)
        : capacity_(capacity)
    {
        if (capacity == 0) {
            throw std::invalid_argument("request queue capacity must be positive");
        }
    }

    [[nodiscard]] bool try_push(Request request)
    {
        std::lock_guard lock(mutex_);
        if (requests_.size() == capacity_) return false;
        requests_.push_back(std::move(request));
        return true;
    }

    [[nodiscard]] std::optional<Request> try_take()
    {
        std::lock_guard lock(mutex_);
        if (requests_.empty()) return std::nullopt;
        auto request = std::move(requests_.front());
        requests_.pop_front();
        return request;
    }

    template <typename Predicate>
    [[nodiscard]] std::vector<Request> remove_if(Predicate matches)
    {
        std::lock_guard lock(mutex_);
        std::vector<Request> removed;
        for (auto it = requests_.begin(); it != requests_.end();) {
            if (matches(*it)) {
                removed.push_back(std::move(*it));
                it = requests_.erase(it);
            } else {
                ++it;
            }
        }
        return removed;
    }

    [[nodiscard]] std::vector<Request> drain()
    {
        return remove_if([](const Request&) { return true; });
    }

private:
    const std::size_t capacity_;
    std::mutex mutex_;
    std::deque<Request> requests_;
};

} // namespace hakoniwa::robot_runtime::runtime
