#pragma once

#include <cstdint>

namespace async_service_lockstep_fixture {

constexpr std::uint64_t kDeltaUsec = 1'000;
constexpr std::int64_t kTimeScale = 1'000'000;
constexpr std::int64_t kPositionIncrement = 1;

constexpr std::int64_t target(unsigned channel, unsigned step)
{
    return 100 * (channel + 1) + step;
}

} // namespace async_service_lockstep_fixture
