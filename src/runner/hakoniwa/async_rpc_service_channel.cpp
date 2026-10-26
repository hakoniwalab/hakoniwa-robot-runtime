#include "hakoniwa/robot_runtime/runner/hakoniwa/async_rpc_service_channel.hpp"

#include "hako_asset_service.h"

#include <exception>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runner::hakoniwa {

CoreAsyncRpcServiceChannel::CoreAsyncRpcServiceChannel(
    std::string channel_name,
    std::string asset_name,
    std::string service_name,
    AsyncRpcRequestDecoder decoder,
    AsyncRpcResponseEncoder encoder)
    : channel_name_(std::move(channel_name))
    , asset_name_(std::move(asset_name))
    , service_name_(std::move(service_name))
    , decoder_(std::move(decoder))
    , encoder_(std::move(encoder))
{
    if (channel_name_.empty() || asset_name_.empty() || service_name_.empty()
        || !decoder_ || !encoder_) {
        throw std::invalid_argument("Core async RPC channel configuration is incomplete");
    }
}

const std::string& CoreAsyncRpcServiceChannel::name() const noexcept
{
    return channel_name_;
}

bool CoreAsyncRpcServiceChannel::prepare(std::string& error) noexcept
{
    return ensure_server(error);
}

bool CoreAsyncRpcServiceChannel::ensure_server(std::string& error) noexcept
{
    if (service_id_ >= 0) {
        return true;
    }
    service_id_ = hako_asset_service_server_create(asset_name_.c_str(), service_name_.c_str());
    if (service_id_ < 0) {
        error = "hako_asset_service_server_create failed";
        return false;
    }
    return true;
}

AsyncRpcPollResult CoreAsyncRpcServiceChannel::poll(
    AsyncRpcRequest& request,
    std::string& error) noexcept
{
    try {
        if (!ensure_server(error)) {
            return AsyncRpcPollResult::error;
        }
        const int event = hako_asset_service_server_poll(service_id_);
        if (event == HAKO_SERVICE_SERVER_API_EVENT_NONE) {
            return AsyncRpcPollResult::none;
        }
        if (event != HAKO_SERVICE_SERVER_API_REQUEST_IN) {
            error = event == HAKO_SERVICE_SERVER_API_REQUEST_CANCEL
                ? "request was canceled" : "hako_asset_service_server_poll failed";
            return AsyncRpcPollResult::error;
        }
        char* packet = nullptr;
        std::size_t packet_len = 0;
        if (hako_asset_service_server_get_request(service_id_, &packet, &packet_len) != 0
            || packet == nullptr) {
            error = "hako_asset_service_server_get_request failed";
            return AsyncRpcPollResult::error;
        }
        request.channel = channel_name_;
        if (!decoder_(
                std::span<const std::byte>(reinterpret_cast<const std::byte*>(packet), packet_len),
                request,
                error)) {
            return AsyncRpcPollResult::error;
        }
        request.channel = channel_name_;
        return AsyncRpcPollResult::request;
    } catch (const std::exception& exception) {
        error = exception.what();
        return AsyncRpcPollResult::error;
    } catch (...) {
        error = "unknown exception";
        return AsyncRpcPollResult::error;
    }
}

bool CoreAsyncRpcServiceChannel::reply(
    const AsyncRpcResponse& response,
    std::string& error) noexcept
{
    try {
        if (!ensure_server(error)) {
            return false;
        }
        char* packet = nullptr;
        std::size_t packet_len = 0;
        if (hako_asset_service_server_get_response_buffer(
                service_id_, &packet, &packet_len,
                HAKO_SERVICE_API_STATUS_DONE,
                HAKO_SERVICE_API_RESULT_CODE_OK) != 0
            || packet == nullptr) {
            error = "hako_asset_service_server_get_response_buffer failed";
            return false;
        }
        if (!encoder_(
                response,
                std::span<std::byte>(reinterpret_cast<std::byte*>(packet), packet_len),
                error)) {
            return false;
        }
        if (hako_asset_service_server_put_response(service_id_, packet, packet_len) != 0) {
            error = "hako_asset_service_server_put_response failed";
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    } catch (...) {
        error = "unknown exception";
        return false;
    }
}

} // namespace hakoniwa::robot_runtime::runner::hakoniwa
