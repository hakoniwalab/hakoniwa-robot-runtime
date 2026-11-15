// Stage B of docs/design-async-rpc.md: the non-lockstep service input.
// Fakes only (no Core): the channel replays scripted poll events, the service
// finishes after a scripted number of on_step calls.
#undef NDEBUG
#include "hakoniwa/robot_runtime/runner/async_service_input.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime::runner;

namespace {

struct Event {
    AsyncRpcPollResult result {AsyncRpcPollResult::none};
    std::uint64_t step {0};
};

class FakeChannel final : public IAsyncRpcChannel {
public:
    explicit FakeChannel(std::string value) : name_(std::move(value)) {}
    const std::string& name() const noexcept override { return name_; }

    AsyncRpcPollResult poll(AsyncRpcRequest& request, std::string& error) noexcept override
    {
        ++polls;
        if (events.empty()) return AsyncRpcPollResult::none;
        const Event event = events.front();
        events.pop_front();
        request.channel = name_;
        if (event.result == AsyncRpcPollResult::request) {
            request.step = event.step;
            request.payload = {std::byte {static_cast<unsigned char>(event.step)}};
        }
        if (event.result == AsyncRpcPollResult::error) error = "fake poll error";
        return event.result;
    }

    bool reply(const AsyncRpcResponse& response, std::string& error) noexcept override
    {
        if (fail_reply) {
            error = "fake reply error";
            return false;
        }
        responses.push_back(response);
        return true;
    }

    void push(AsyncRpcPollResult result, std::uint64_t step = 0)
    {
        events.push_back(Event {result, step});
    }

    std::string name_;
    std::deque<Event> events;
    std::vector<AsyncRpcResponse> responses;
    int polls {0};
    bool fail_reply {false};
};

class FakeService final : public IAsyncService {
public:
    bool on_request(const AsyncRpcRequest& request, std::string& error) noexcept override
    {
        requests.push_back(request);
        if (refuse) {
            error = "refused by fake";
            return false;
        }
        steps_in_request = 0;
        return true;
    }
    void on_cancel() noexcept override { ++cancels; }
    bool on_step(AsyncServiceState state, AsyncRpcResponse& response) noexcept override
    {
        states.push_back(state);
        ++steps_in_request;
        if (state == AsyncServiceState::canceling) {
            if (steps_in_request < cancel_after) return false;
            response.step = requests.back().step;
            response.result_code = AsyncRpcResultCode::canceled;
            response.payload = {std::byte {0xCC}};
            return true;
        }
        if (steps_in_request < finish_after) return false;
        response.step = requests.back().step;
        response.result_code = finish_code;
        response.payload = {std::byte {0xD0}};
        return true;
    }
    void reset() noexcept override { ++resets; }

    std::vector<AsyncRpcRequest> requests;
    std::vector<AsyncServiceState> states;
    int steps_in_request {0};
    int finish_after {3};
    int cancel_after {1};
    AsyncRpcResultCode finish_code {AsyncRpcResultCode::ok};
    bool refuse {false};
    int cancels {0};
    int resets {0};
};

struct Rig {
    std::shared_ptr<FakeChannel> channel = std::make_shared<FakeChannel>("A");
    std::shared_ptr<FakeService> service = std::make_shared<FakeService>();
    std::unique_ptr<AsyncServiceInput> input =
        std::make_unique<AsyncServiceInput>(std::vector<AsyncServiceBinding> {{channel, service}});

    /** One simulation step; returns whether both halves succeeded. */
    bool step() { return input->before_physics() && input->after_physics(); }
};

} // namespace

int main()
{
    {
        // B1: no request -> every step passes, the channel is polled once per
        // step, no hook runs and nothing is answered.
        Rig rig;
        assert(rig.input->prepare_channels());
        for (int i = 0; i < 5; ++i) assert(rig.step());
        assert(rig.channel->polls == 5);
        assert(rig.service->requests.empty());
        assert(rig.service->states.empty());
        assert(rig.channel->responses.empty());
        assert(!rig.input->state(0).has_value());
        assert(!rig.input->failed());
    }
    {
        // B2: a request is handed over in the step it arrives; on_step starts
        // in that step; the reply goes out after physics of the step in which
        // on_step finished (3rd call), with the service's response; then idle.
        Rig rig;
        rig.channel->push(AsyncRpcPollResult::request, 7);
        assert(rig.input->before_physics());
        assert(rig.service->requests.size() == 1);
        assert(rig.service->requests[0].step == 7);
        assert(rig.service->requests[0].channel == "A");
        assert(rig.service->states.size() == 1);
        assert(rig.input->state(0) == AsyncServiceState::doing);
        assert(rig.input->after_physics());
        assert(rig.channel->responses.empty());

        assert(rig.step());
        assert(rig.channel->responses.empty());
        assert(rig.input->before_physics());
        assert(rig.service->states.size() == 3);
        assert(rig.channel->responses.empty()); // decided, but sent after physics
        assert(rig.input->after_physics());
        assert(rig.channel->responses.size() == 1);
        assert(rig.channel->responses[0].step == 7);
        assert(rig.channel->responses[0].result_code == AsyncRpcResultCode::ok);
        assert(rig.channel->responses[0].payload == std::vector<std::byte> {std::byte {0xD0}});
        assert(!rig.input->state(0).has_value());

        for (int i = 0; i < 3; ++i) assert(rig.step());
        assert(rig.service->states.size() == 3); // no on_step while idle
        assert(rig.channel->responses.size() == 1);
        for (const auto state : rig.service->states) assert(state == AsyncServiceState::doing);
    }
    {
        // B3: a service that finishes in its first on_step answers in the same
        // step it was accepted.
        Rig rig;
        rig.service->finish_after = 1;
        rig.channel->push(AsyncRpcPollResult::request, 1);
        assert(rig.step());
        assert(rig.channel->responses.size() == 1);
        assert(!rig.input->state(0).has_value());
    }
    {
        // B4: a refused request is answered INVALID after physics, with the
        // request's step; on_step never runs and the service stays idle.
        Rig rig;
        rig.service->refuse = true;
        rig.channel->push(AsyncRpcPollResult::request, 4);
        assert(rig.input->before_physics());
        assert(!rig.input->state(0).has_value());
        assert(rig.channel->responses.empty());
        assert(rig.input->after_physics());
        assert(rig.channel->responses.size() == 1);
        assert(rig.channel->responses[0].result_code == AsyncRpcResultCode::invalid);
        assert(rig.channel->responses[0].step == 4);
        assert(rig.service->states.empty());
        assert(!rig.input->failed());
    }
    {
        // B5: a cancel while in progress -> on_cancel once, the next on_step
        // (same step) sees CANCELING; the service needs 2 canceling steps to
        // stop, then CANCELED goes out after physics and the service is idle.
        Rig rig;
        rig.service->finish_after = 100;
        rig.service->cancel_after = 3; // counts steps since accepted
        rig.channel->push(AsyncRpcPollResult::request, 2);
        assert(rig.step()); // accepted, on_step #1 doing
        rig.channel->push(AsyncRpcPollResult::cancel);
        assert(rig.input->before_physics()); // on_step #2 canceling
        assert(rig.service->cancels == 1);
        assert(rig.input->state(0) == AsyncServiceState::canceling);
        assert(rig.service->states.back() == AsyncServiceState::canceling);
        assert(rig.input->after_physics());
        assert(rig.channel->responses.empty());
        assert(rig.step()); // on_step #3 canceling -> stopped
        assert(rig.service->cancels == 1);
        assert(rig.channel->responses.size() == 1);
        assert(rig.channel->responses[0].result_code == AsyncRpcResultCode::canceled);
        assert(rig.channel->responses[0].step == 2);
        assert(!rig.input->state(0).has_value());
        assert(rig.service->states.size() == 3);
        assert(rig.service->states[0] == AsyncServiceState::doing);
        assert(rig.service->states[1] == AsyncServiceState::canceling);
        assert(rig.service->states[2] == AsyncServiceState::canceling);
    }
    {
        // B6: a cancel while idle is ignored (Core drops it; a stray one must
        // not stop the simulation).
        Rig rig;
        rig.channel->push(AsyncRpcPollResult::cancel);
        assert(rig.step());
        assert(rig.service->cancels == 0);
        assert(rig.channel->responses.empty());
        assert(!rig.input->failed());
    }
    {
        // B7: a request while in progress breaks Core's contract -> the input
        // fails, naming the channel; no further hook runs.
        Rig rig;
        rig.service->finish_after = 100;
        rig.channel->push(AsyncRpcPollResult::request, 1);
        assert(rig.step());
        rig.channel->push(AsyncRpcPollResult::request, 2);
        assert(!rig.input->before_physics());
        assert(rig.input->failed());
        assert(rig.input->failure_reason().find("A") != std::string::npos);
        assert(rig.service->requests.size() == 1);
        const auto steps_seen = rig.service->states.size();
        assert(!rig.input->before_physics());
        assert(rig.service->states.size() == steps_seen);
    }
    {
        // B8: an ERROR result from on_step is answered as ERROR.
        Rig rig;
        rig.service->finish_after = 1;
        rig.service->finish_code = AsyncRpcResultCode::error;
        rig.channel->push(AsyncRpcPollResult::request, 3);
        assert(rig.step());
        assert(rig.channel->responses.size() == 1);
        assert(rig.channel->responses[0].result_code == AsyncRpcResultCode::error);
        assert(!rig.input->failed());
    }
    {
        // B9: a channel poll error and a reply error fail the input with a
        // reason that names the channel.
        Rig poll_rig;
        poll_rig.channel->push(AsyncRpcPollResult::error);
        assert(!poll_rig.input->before_physics());
        assert(poll_rig.input->failed());
        assert(poll_rig.input->failure_reason().find("A") != std::string::npos);
        assert(poll_rig.input->failure_reason().find("fake poll error") != std::string::npos);

        Rig reply_rig;
        reply_rig.service->finish_after = 1;
        reply_rig.channel->fail_reply = true;
        reply_rig.channel->push(AsyncRpcPollResult::request, 1);
        assert(reply_rig.input->before_physics());
        assert(!reply_rig.input->after_physics());
        assert(reply_rig.input->failed());
        assert(reply_rig.input->failure_reason().find("fake reply error") != std::string::npos);
    }
    {
        // B10: reset drops the request in progress without a reply and calls
        // reset() on every service; a new request is then accepted.
        Rig rig;
        rig.service->finish_after = 100;
        rig.channel->push(AsyncRpcPollResult::request, 1);
        assert(rig.step());
        rig.input->reset();
        assert(rig.service->resets == 1);
        assert(!rig.input->state(0).has_value());
        assert(rig.step());
        assert(rig.channel->responses.empty());
        rig.channel->push(AsyncRpcPollResult::request, 5);
        assert(rig.step());
        assert(rig.service->requests.size() == 2);
        assert(rig.input->state(0) == AsyncServiceState::doing);
    }
    {
        // B11: two bindings are independent: A finishes while B keeps running
        // and is then canceled.
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        auto sa = std::make_shared<FakeService>();
        auto sb = std::make_shared<FakeService>();
        sa->finish_after = 2;
        sb->finish_after = 100;
        sb->cancel_after = 3;
        AsyncServiceInput input({{a, sa}, {b, sb}});
        a->push(AsyncRpcPollResult::request, 1);
        b->push(AsyncRpcPollResult::request, 2);
        assert(input.before_physics() && input.after_physics());
        assert(input.before_physics() && input.after_physics());
        assert(a->responses.size() == 1 && b->responses.empty());
        assert(!input.state(0).has_value());
        assert(input.state(1) == AsyncServiceState::doing);
        b->push(AsyncRpcPollResult::cancel);
        assert(input.before_physics() && input.after_physics());
        assert(b->responses.size() == 1);
        assert(b->responses[0].result_code == AsyncRpcResultCode::canceled);
        assert(sa->states.size() == 2);
        assert(!input.state(2).has_value()); // out of range reads as idle
    }
    return 0;
}
