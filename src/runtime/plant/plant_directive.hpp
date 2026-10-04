#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE

#include "runtime/types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Type-erased, immutable instruction for the Plant on the Plant Directive path.
 *
 * A directive is not an arbitration candidate. It is produced by a
 * non-arbitrated IDirectiveController and handed to
 * IActuatorPlant::apply_directives() after arbitration and right before the
 * physics step. Concrete directives are backend-independent; each Plant
 * applier picks only its own kind with directives_of<T>() and ignores the rest.
 */
class IPlantDirective {
public:
    virtual ~IPlantDirective() = default;
    [[nodiscard]] virtual std::string_view type_name() const noexcept = 0;
};

using PlantDirectiveList = std::vector<std::shared_ptr<const IPlantDirective>>;

/** Picks every directive of one concrete kind out of a mixed directive list. */
template <class T>
[[nodiscard]] std::vector<std::shared_ptr<const T>> directives_of(
    const PlantDirectiveList& directives)
{
    std::vector<std::shared_ptr<const T>> selected;
    for (const auto& directive : directives) {
        if (auto typed = std::dynamic_pointer_cast<const T>(directive)) {
            selected.push_back(std::move(typed));
        }
    }
    return selected;
}

/** Mirror pose / velocity for one externally owned body. */
class MirrorBodyDirective final : public IPlantDirective {
public:
    explicit MirrorBodyDirective(MirrorBodyCommand command)
        : command_(std::move(command))
    {
    }

    [[nodiscard]] std::string_view type_name() const noexcept override
    {
        return "hakoniwa/MirrorBody";
    }

    [[nodiscard]] const MirrorBodyCommand& command() const noexcept
    {
        return command_;
    }

private:
    MirrorBodyCommand command_;
};

/**
 * Sliding friction for a fixed set of geoms.
 *
 * Only the sliding coefficient is carried; the Plant keeps torsional and
 * rolling friction unchanged. The value persists in the Plant until the next
 * directive for the same geoms or a Plant reset.
 */
class GeomFrictionDirective final : public IPlantDirective {
public:
    GeomFrictionDirective(
        std::vector<std::string> geom_names,
        const double sliding_friction,
        const std::uint64_t created_at_usec)
        : geom_names_(std::move(geom_names))
        , sliding_friction_(sliding_friction)
        , created_at_usec_(created_at_usec)
    {
    }

    [[nodiscard]] std::string_view type_name() const noexcept override
    {
        return "hakoniwa/GeomFriction";
    }

    [[nodiscard]] const std::vector<std::string>& geom_names() const noexcept
    {
        return geom_names_;
    }

    [[nodiscard]] double sliding_friction() const noexcept
    {
        return sliding_friction_;
    }

    [[nodiscard]] std::uint64_t created_at_usec() const noexcept
    {
        return created_at_usec_;
    }

private:
    std::vector<std::string> geom_names_;
    double sliding_friction_ {0.0};
    std::uint64_t created_at_usec_ {0};
};

} // namespace hakoniwa::robot_runtime::runtime

#endif
