#include <gtest/gtest.h>
#include "priority_request_table.hpp"
#include <chrono>

using namespace vanetza;
using std::chrono::seconds;

namespace
{

const Clock::time_point t0 = Clock::at("2026-10-03 10:00:00");

} // namespace

TEST(PriorityRequestTable, sequence_number_follows_changes)
{
    PriorityRequestTable table(256);
    EXPECT_EQ(0, table.commit());

    EXPECT_TRUE(table.update(20, 1, 2, t0 + seconds(30)));
    EXPECT_EQ(1, table.commit());
    EXPECT_TRUE(table.update(20, 1, 2, t0 + seconds(31))); // repeated, unchanged
    EXPECT_EQ(1, table.commit());
    EXPECT_TRUE(table.update(20, 1, 4, t0 + seconds(32))); // status changed
    EXPECT_EQ(2, table.commit());
}

TEST(PriorityRequestTable, several_requests_of_one_srem_change_once)
{
    PriorityRequestTable table(256);
    table.update(20, 1, 2, t0 + seconds(30));
    table.update(20, 2, 2, t0 + seconds(30));
    table.update(21, 1, 5, t0 + seconds(30));
    EXPECT_EQ(1, table.commit());
    EXPECT_EQ(3u, table.size());
}

TEST(PriorityRequestTable, cancellation)
{
    PriorityRequestTable table(256);
    table.update(20, 1, 2, t0 + seconds(30));
    EXPECT_EQ(1, table.commit());

    table.cancel(20, 7); // unknown request
    EXPECT_EQ(1, table.commit());
    table.cancel(20, 1);
    EXPECT_EQ(2, table.commit());
    EXPECT_EQ(0u, table.size());
}

TEST(PriorityRequestTable, expiry)
{
    PriorityRequestTable table(256);
    table.update(20, 1, 2, t0 + seconds(10));
    table.update(21, 1, 2, t0 + seconds(30));
    EXPECT_EQ(1, table.commit());

    table.expire(t0 + seconds(10)); // expires after this time
    EXPECT_EQ(2u, table.size());
    EXPECT_EQ(1, table.commit());

    // the first request expires while the second one is repeated unchanged
    table.expire(t0 + seconds(11));
    table.update(21, 1, 2, t0 + seconds(41));
    EXPECT_EQ(2, table.commit());
    EXPECT_EQ(1u, table.size());
}

TEST(PriorityRequestTable, capacity)
{
    PriorityRequestTable table(2);
    EXPECT_TRUE(table.update(20, 1, 2, t0));
    EXPECT_TRUE(table.update(21, 1, 2, t0));
    EXPECT_FALSE(table.update(22, 1, 2, t0));
    EXPECT_TRUE(table.update(20, 1, 4, t0)); // known requests are still updated
    EXPECT_EQ(2u, table.size());
}

TEST(PriorityRequestTable, sequence_number_wraps)
{
    PriorityRequestTable table(256);
    for (long i = 1; i <= 128; ++i) {
        table.update(20, 1, i % 2, t0); // status toggles with every SREM
        EXPECT_EQ(i % 128, table.commit());
    }
}
