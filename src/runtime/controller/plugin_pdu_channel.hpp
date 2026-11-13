#pragma once

#include "runtime/publisher/encoded_pdu_sink.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/**
 * The latest encoded sample of one PDU, written by the PDU's writer (as an
 * IEncodedPduSink) and read by the controller plugins that list the PDU.
 *
 * Writer and readers run in the simulation thread (publish stage, then the
 * controller stage of a later step); the channel takes no lock.
 * version() changes with every store() and clear(), so a reader detects a
 * new sample by comparing the version it saw last.
 */
class PluginPduChannel final : public IEncodedPduSink {
public:
    PluginPduChannel(std::string pdu_name, std::string pdu_type)
        : pdu_name_(std::move(pdu_name))
        , pdu_type_(std::move(pdu_type))
    {
    }

    void store(std::span<const std::byte> data, std::uint64_t sample_time_usec) override
    {
        data_.assign(data.begin(), data.end());
        sample_time_usec_ = sample_time_usec;
        has_sample_ = true;
        ++version_;
    }

    void clear() noexcept override
    {
        data_.clear();
        sample_time_usec_ = 0;
        has_sample_ = false;
        ++version_;
    }

    [[nodiscard]] const std::string& pdu_name() const noexcept { return pdu_name_; }
    [[nodiscard]] const std::string& pdu_type() const noexcept { return pdu_type_; }
    [[nodiscard]] bool has_sample() const noexcept { return has_sample_; }
    [[nodiscard]] std::uint64_t version() const noexcept { return version_; }
    [[nodiscard]] std::uint64_t sample_time_usec() const noexcept { return sample_time_usec_; }
    [[nodiscard]] const std::vector<std::byte>& data() const noexcept { return data_; }

private:
    std::string pdu_name_;
    std::string pdu_type_;
    std::vector<std::byte> data_;
    std::uint64_t sample_time_usec_ {0};
    std::uint64_t version_ {0};
    bool has_sample_ {false};
};

/**
 * The PDU channels of all controller plugins of one Runtime, by PDU name.
 * The composition root makes it from the RuntimeDefinition, gives it to
 * build_runtime() (the plugins read from it) and hands each channel to the
 * writer of that PDU (the writer stores into it).
 */
class PluginPduChannels final {
public:
    /** Returns the channel of pdu_name, adding it on first use. */
    std::shared_ptr<PluginPduChannel> add(const std::string& pdu_name, const std::string& pdu_type)
    {
        auto& channel = channels_[pdu_name];
        if (channel == nullptr) {
            channel = std::make_shared<PluginPduChannel>(pdu_name, pdu_type);
        }
        return channel;
    }

    /** The channel of pdu_name, or null when no plugin reads it. */
    [[nodiscard]] std::shared_ptr<PluginPduChannel> find(const std::string& pdu_name) const
    {
        const auto channel = channels_.find(pdu_name);
        if (channel == channels_.end()) {
            return nullptr;
        }
        return channel->second;
    }

    [[nodiscard]] bool empty() const noexcept { return channels_.empty(); }

private:
    std::map<std::string, std::shared_ptr<PluginPduChannel>> channels_;
};

} // namespace hakoniwa::robot_runtime::runtime
