#pragma once


#include "runtime/service/async_service.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <span>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

enum class AsyncServiceEvent {
    None,
    Request,
    Cancel,
    Error,
};

enum class AsyncServiceResultCode {
    Ok,
    Invalid,
    Canceled,
    Error,
};

using AsyncServiceResponseWriter = std::function<bool(std::span<std::byte>)>;

/** Nonblocking I/O for an already-created Core service with one client. */
class IAsyncServiceServer {
public:
    virtual ~IAsyncServiceServer() = default;

    virtual AsyncServiceEvent poll(int service_id) noexcept = 0;
    virtual std::optional<AsyncServiceState> status(int service_id) noexcept = 0;
    virtual bool get_request(int service_id, std::vector<std::byte>& request) = 0;
    /**
     * Call the writer synchronously with the prepared packet. A missing writer
     * sends the packet as provided by Core. Writer failure suppresses sending.
     */
    virtual bool reply(
        int service_id, AsyncServiceResultCode code,
        const AsyncServiceResponseWriter& write = {}) = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
