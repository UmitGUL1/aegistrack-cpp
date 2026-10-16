#include "aegistrack/events/event_bus.hpp"

#include <algorithm>

namespace aegistrack {

EventBus::SubscriptionId EventBus::subscribe(EventCallback callback) {
    std::scoped_lock lock(mutex_);
    const SubscriptionId id = next_sub_id_++;
    subscribers_.push_back(Subscriber{id, std::move(callback)});
    return id;
}

void EventBus::unsubscribe(SubscriptionId id) {
    std::scoped_lock lock(mutex_);
    subscribers_.erase(
        std::remove_if(subscribers_.begin(), subscribers_.end(),
                       [id](const Subscriber& s) { return s.id == id; }),
        subscribers_.end());
}

void EventBus::publish(SystemEvent event) {
    std::vector<EventCallback> callbacks_copy;
    {
        std::scoped_lock lock(mutex_);
        event.event_id = next_event_id_++;
        if (history_.size() >= max_history_) {
            history_.erase(history_.begin());
        }
        history_.push_back(event);
        callbacks_copy.reserve(subscribers_.size());
        for (const auto& sub : subscribers_) {
            if (sub.callback) {
                callbacks_copy.push_back(sub.callback);
            }
        }
    }
    for (const auto& cb : callbacks_copy) {
        cb(event);
    }
}

std::vector<SystemEvent> EventBus::history() const {
    std::scoped_lock lock(mutex_);
    return history_;
}

void EventBus::clearHistory() {
    std::scoped_lock lock(mutex_);
    history_.clear();
}

}  // namespace aegistrack
