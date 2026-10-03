#include "priority_request_table.hpp"
#include <vanetza/facilities/spat_functions.hpp>

using namespace vanetza;

PriorityRequestTable::PriorityRequestTable(std::size_t capacity) : capacity_(capacity)
{
}

void PriorityRequestTable::expire(Clock::time_point now)
{
    for (auto it = entries_.begin(); it != entries_.end();) {
        if (it->second.expiry < now) {
            it = entries_.erase(it);
            changed_ = true;
        } else {
            ++it;
        }
    }
}

bool PriorityRequestTable::update(std::uint32_t requester, long request_id, long status, Clock::time_point expiry)
{
    const auto key = std::make_pair(requester, request_id);
    auto found = entries_.find(key);
    if (found == entries_.end()) {
        if (entries_.size() >= capacity_) {
            return false;
        }
        entries_.emplace(key, Entry { status, expiry });
        changed_ = true;
    } else {
        changed_ |= found->second.status != status;
        found->second.status = status;
        found->second.expiry = expiry;
    }
    return true;
}

void PriorityRequestTable::cancel(std::uint32_t requester, long request_id)
{
    changed_ |= entries_.erase(std::make_pair(requester, request_id)) > 0;
}

long PriorityRequestTable::commit()
{
    if (changed_) {
        sequence_number_ = facilities::next_msg_count(sequence_number_);
        changed_ = false;
    }
    return sequence_number_;
}

std::size_t PriorityRequestTable::size() const
{
    return entries_.size();
}
