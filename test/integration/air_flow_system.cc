/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gtest/gtest.h>

#include <gz/msgs/air_flow_sensor.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AirFlowSensor.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test AirFlowTest system
class AirFlowTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(AirFlowTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(AirFlow))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "air_flow.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "air_flow_sensor";

  auto topic = "world/air_flow_sensor/model/air_flow_model/link/link/"
      "sensor/air_flow_sensor/air_flow";

  bool updateChecked{false};

  // Create a system that checks sensor topic
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::AirFlowSensor, components::Name>(
            [&](const Entity &_entity,
                const components::AirFlowSensor *,
                const components::Name *_name) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);

              auto sensorComp = _ecm.Component<components::Sensor>(_entity);
              EXPECT_NE(nullptr, sensorComp);

              if (_info.iterations == 1)
                return true;

              // This component is created on the 2nd PreUpdate
              auto topicComp = _ecm.Component<components::SensorTopic>(_entity);
              EXPECT_NE(nullptr, topicComp);
              if (topicComp)
              {
                EXPECT_EQ(topic, topicComp->Data());
              }

              updateChecked = true;

              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // Subscribe to air_flow topic
  bool received{false};
  msgs::AirFlowSensor msg;
  msg.Clear();
  std::function<void(const msgs::AirFlowSensor &)>  cb =
      [&received, &msg](const msgs::AirFlowSensor &_msg)
  {
    // Only need one message
    if (received)
      return;

    msg = _msg;
    received = true;
  };

  transport::Node node;
  node.Subscribe(topic, cb);

  // Run server
  server.Run(true, 100, false);
  EXPECT_TRUE(updateChecked);

  // Wait for message to be received
  for (int sleep = 0; !received && sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(received);

  // check air flow
  EXPECT_TRUE(msg.has_header());
  EXPECT_TRUE(msg.header().has_stamp());
  EXPECT_EQ(0, msg.header().stamp().sec());
  EXPECT_LT(0, msg.header().stamp().nsec());
  EXPECT_DOUBLE_EQ(0, msg.xy_direction());
  EXPECT_DOUBLE_EQ(0, msg.xy_speed());
}
