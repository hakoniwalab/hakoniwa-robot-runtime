#include "runtime/arbiter/priority_command_arbiter.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

PriorityCommandArbiter::PriorityCommandArbiter(
    std::vector<ControllerPriority> priorities)
{
    std::unordered_map<std::string, std::int32_t> group_priorities;
    policies_.reserve(priorities.size());
    for (auto& priority : priorities) {
        if (priority.controller_id.empty()) {
            throw std::invalid_argument("arbiter controller ID must not be empty");
        }
        const std::string group_id = priority.control_group_id.empty()
            ? priority.controller_id
            : priority.control_group_id;
        const auto [group, inserted] = group_priorities.emplace(
            group_id, priority.priority);
        if (!inserted && group->second != priority.priority) {
            throw std::invalid_argument(
                "controllers in one control group must have the same priority");
        }
        if (!policies_.emplace(
                std::move(priority.controller_id),
                Policy {group_id, priority.priority}).second) {
            throw std::invalid_argument("arbiter controller ID must be unique");
        }
    }
}

ArbitrationResult PriorityCommandArbiter::arbitrate(
    const std::vector<ControllerOutput>& controller_outputs,
    const RobotState& current_state,
    const RuntimeStepContext& context) const
{
    (void)current_state;
    (void)context;

    struct Candidate {
        std::int32_t priority {0};
        std::vector<ActuatorCommand> commands;
    };

    std::unordered_map<std::string, Candidate> candidates;
    std::unordered_set<std::string> rejected_sources;
    for (const auto& output : controller_outputs) {
        if (!output.status.can_produce_commands() || output.commands.empty()) {
            continue;
        }
        const auto policy = policies_.find(output.controller_id);
        if (policy == policies_.end()) {
            for (const auto& command : output.commands) {
                if (!command.source_id.empty()) {
                    rejected_sources.insert(command.source_id);
                }
            }
            continue;
        }
        auto& candidate = candidates[policy->second.control_group_id];
        candidate.priority = policy->second.priority;
        candidate.commands.insert(
            candidate.commands.end(), output.commands.begin(), output.commands.end());
    }

    ArbitrationResult result;
    result.rejected_source_ids.assign(
        rejected_sources.begin(), rejected_sources.end());
    std::sort(result.rejected_source_ids.begin(), result.rejected_source_ids.end());

    const Candidate* selected = nullptr;
    std::optional<std::string> selected_control_id;
    std::int32_t best_priority = std::numeric_limits<std::int32_t>::min();
    bool conflict = false;
    for (const auto& [group_id, candidate] : candidates) {
        if (selected == nullptr || candidate.priority > best_priority) {
            selected = &candidate;
            selected_control_id = group_id;
            best_priority = candidate.priority;
            conflict = false;
        } else if (candidate.priority == best_priority) {
            conflict = true;
        }
    }

    if (conflict) {
        std::unordered_set<std::string> actuator_ids;
        for (const auto& [group_id, candidate] : candidates) {
            (void)group_id;
            if (candidate.priority != best_priority) {
                continue;
            }
            for (const auto& command : candidate.commands) {
                actuator_ids.insert(command.actuator_id);
            }
        }
        result.conflicting_actuator_ids.assign(
            actuator_ids.begin(), actuator_ids.end());
        std::sort(
            result.conflicting_actuator_ids.begin(),
            result.conflicting_actuator_ids.end());
        return result;
    }

    if (selected != nullptr) {
        result.selected_control_id = std::move(selected_control_id);
        result.selected_commands = selected->commands;
        std::sort(
            result.selected_commands.begin(),
            result.selected_commands.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs.actuator_id < rhs.actuator_id;
            });
    }
    return result;
}

} // namespace hakoniwa::robot_runtime::runtime
