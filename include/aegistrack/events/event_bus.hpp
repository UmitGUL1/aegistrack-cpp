#pragma once

#include "aegistrack/events/event.hpp"

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

namespace aegistrack {

/**
 * @brief Thread-safe publish-subscribe event bus with bounded chronological history.
 */
class EventBus {
public:
    using SubscriptionId = std::uint64_t;
    using EventCallback = std::function<void(const SystemEvent&)>;

    explicit EventBus(std::size_t max_history = 5000) : max_history_(max_history) {}

    SubscriptionId subscribe(EventCallback callback);
    void unsubscribe(SubscriptionId id);
    void publish(SystemEvent event);

    [[nodiscard]] std::vector<SystemEvent> history() const;
    void clearHistory();

private:
    struct Subscriber {
        SubscriptionId id{0};
        EventCallback callback;
    };

    std::size_t max_history_;
    mutable std::mutex mutex_;
    std::uint64_t next_sub_id_{1};
    std::uint64_t next_event_id_{1};
    std::vector<Subscriber> subscribers_;
    std::vector<SystemEvent> history_;
};

}  // namespace aegistrack
