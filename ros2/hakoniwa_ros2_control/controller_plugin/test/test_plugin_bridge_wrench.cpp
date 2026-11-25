#include <array>

#include <gtest/gtest.h>

#include "plugin_bridge.hpp"

namespace hakoniwa_ros2_control
{
namespace
{

constexpr std::array<double, 6> sample_wrench{1.25, -2.5, 3.75, -0.125, 0.25, -0.5};

TEST(PluginBridgeWrench, LooksUpConfiguredNames)
{
  PluginBridge bridge;
  bridge.configure_wrenches({"a", "b"});

  EXPECT_EQ(bridge.wrench_index("a"), 0);
  EXPECT_EQ(bridge.wrench_index("b"), 1);
  EXPECT_EQ(bridge.wrench_index("x"), -1);
}

TEST(PluginBridgeWrench, StoresAllSixAxes)
{
  PluginBridge bridge;
  bridge.configure_wrenches({"a", "b"});

  EXPECT_FALSE(bridge.has_wrench(1));
  bridge.set_wrench(1, sample_wrench);

  EXPECT_TRUE(bridge.has_wrench(1));
  EXPECT_EQ(bridge.wrench(1), sample_wrench);
  EXPECT_FALSE(bridge.has_wrench(0));
}

TEST(PluginBridgeWrench, ReconfigurationInvalidatesStoredWrench)
{
  PluginBridge bridge;
  bridge.configure_wrenches({"a", "b"});
  bridge.set_wrench(1, sample_wrench);
  ASSERT_TRUE(bridge.has_wrench(1));

  bridge.configure_wrenches({"a", "b"});

  EXPECT_FALSE(bridge.has_wrench(1));
}

TEST(PluginBridgeWrench, ClearRemovesSensorAndItsValue)
{
  PluginBridge bridge;
  bridge.configure_wrenches({"a", "b"});
  bridge.set_wrench(1, sample_wrench);
  ASSERT_TRUE(bridge.has_wrench(1));

  bridge.clear();

  EXPECT_EQ(bridge.wrench_index("b"), -1);
  // has_wrench requires a configured index.
  bridge.configure_wrenches({"a", "b"});
  EXPECT_FALSE(bridge.has_wrench(1));
}

TEST(PluginBridgeWrench, JointAndWrenchStorageAreIndependent)
{
  PluginBridge bridge;
  bridge.configure({"joint"});
  bridge.set_state(0, 1.0, -2.0, 3.0);

  bridge.configure_wrenches({"a"});
  bridge.set_wrench(0, sample_wrench);

  EXPECT_EQ(bridge.index("joint"), 0);
  EXPECT_DOUBLE_EQ(bridge.position(0), 1.0);
  EXPECT_DOUBLE_EQ(bridge.velocity(0), -2.0);
  EXPECT_DOUBLE_EQ(bridge.effort(0), 3.0);

  bridge.set_state(0, 4.0, -5.0, 6.0);
  EXPECT_TRUE(bridge.has_wrench(0));
  EXPECT_EQ(bridge.wrench(0), sample_wrench);

  bridge.configure({"new_joint"});

  EXPECT_EQ(bridge.wrench_index("a"), 0);
  EXPECT_TRUE(bridge.has_wrench(0));
  EXPECT_EQ(bridge.wrench(0), sample_wrench);
}

}  // namespace
}  // namespace hakoniwa_ros2_control
