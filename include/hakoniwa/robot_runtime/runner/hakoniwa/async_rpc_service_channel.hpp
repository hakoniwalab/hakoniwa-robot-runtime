#pragma once

#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"

#include <functional>
#include <span>

namespace hakoniwa::robot_runtime::runner::hakoniwa {

/** Decode one complete Core service request packet into step + opaque body. */
using AsyncRpcRequestDecoder = std::function<bool(
    std::span<const std::byte>, AsyncRpcRequest&, std::string& error)>;

/** Encode step + opaque body into a Core-provided response packet buffer. */
using AsyncRpcResponseEncoder = std::function<bool(
    const AsyncRpcResponse&, std::span<std::byte>, std::string& error)>;

/**
 * Thin production adapter over hako_asset_service_server_*.
 *
 * One instance represents one robot channel (and therefore one Core service
 * server).  Keeping a server per robot lets the batch retain all outstanding
 * requests until physics has stepped; Core's server API replies to the current
 * request of one server.  Call hako_asset_service_initialize() before polling.
 */
class CoreAsyncRpcServiceChannel final : public IAsyncRpcChannel {
public:
    CoreAsyncRpcServiceChannel(
        std::string channel_name,
        std::string asset_name,
        std::string service_name,
        AsyncRpcRequestDecoder decoder,
        AsyncRpcResponseEncoder encoder);

    /** Create the Core service server during Start, before asset feedback. */
    bool prepare(std::string& error) noexcept;
    [[nodiscard]] const std::string& name() const noexcept override;
    AsyncRpcPollResult poll(AsyncRpcRequest& request, std::string& error) noexcept override;
    bool reply(const AsyncRpcResponse& response, std::string& error) noexcept override;

private:
    bool ensure_server(std::string& error) noexcept;

    std::string channel_name_;
    std::string asset_name_;
    std::string service_name_;
    AsyncRpcRequestDecoder decoder_;
    AsyncRpcResponseEncoder encoder_;
    int service_id_ {-1};
};

} // namespace hakoniwa::robot_runtime::runner::hakoniwa
