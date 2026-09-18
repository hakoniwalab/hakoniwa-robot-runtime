#include "hakoniwa/robot_runtime/adapters/endpoint/impulse_collision_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR

#include "hako_msgs/pdu_cpptype_ImpulseCollision.hpp"
#include "hako_msgs/pdu_cpptype_conv_ImpulseCollision.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class ImpulseCollisionPduWriter::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint_value,
        std::string pdu_robot,
        std::string pdu_name)
        : endpoint(endpoint_value,
            ::hakoniwa::pdu::PduKey {
                std::move(pdu_robot), std::move(pdu_name)})
    {
    }

    ::hakoniwa::pdu::TypedEndpoint<
        HakoCpp_ImpulseCollision,
        ::hako::pdu::msgs::hako_msgs::ImpulseCollision> endpoint;
};

ImpulseCollisionPduWriter::ImpulseCollisionPduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

ImpulseCollisionPduWriter::~ImpulseCollisionPduWriter() = default;

bool ImpulseCollisionPduWriter::send(
    const runtime::ImpulseCollisionOutput& output) noexcept
{
    HakoCpp_ImpulseCollision pdu {};
    pdu.collision = output.collision;
    pdu.is_target_static = output.is_target_static;
    pdu.restitution_coefficient = output.restitution_coefficient;
    pdu.self_contact_vector.x = output.self_contact_vector.x;
    pdu.self_contact_vector.y = output.self_contact_vector.y;
    pdu.self_contact_vector.z = output.self_contact_vector.z;
    pdu.normal.x = output.normal.x;
    pdu.normal.y = output.normal.y;
    pdu.normal.z = output.normal.z;
    pdu.target_contact_vector.x = output.target_contact_vector.x;
    pdu.target_contact_vector.y = output.target_contact_vector.y;
    pdu.target_contact_vector.z = output.target_contact_vector.z;
    pdu.target_velocity.x = output.target_velocity.x;
    pdu.target_velocity.y = output.target_velocity.y;
    pdu.target_velocity.z = output.target_velocity.z;
    pdu.target_angular_velocity.x = output.target_angular_velocity.x;
    pdu.target_angular_velocity.y = output.target_angular_velocity.y;
    pdu.target_angular_velocity.z = output.target_angular_velocity.z;
    pdu.target_euler.x = output.target_euler.roll;
    pdu.target_euler.y = output.target_euler.pitch;
    pdu.target_euler.z = output.target_euler.yaw;
    pdu.target_inertia.x = output.target_inertia.x;
    pdu.target_inertia.y = output.target_inertia.y;
    pdu.target_inertia.z = output.target_inertia.z;
    pdu.target_mass = output.target_mass;
    return impl_->endpoint.send(pdu) == HAKO_PDU_ERR_OK;
}

void ImpulseCollisionPduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
