/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include "test_constants.h"

using namespace std::chrono_literals;

int g_argc;
char** g_argv;

class MapClientTest : public testing::Test
{
  public:
    // A node is needed to make a service call
    rclcpp::Node::SharedPtr n_;

    MapClientTest()
    : n_(nullptr)
    {
      n_.reset(new rclcpp::Node("map_client_test"));
      got_map_ = false;
      got_map_metadata_ = false;
    }

    ~MapClientTest()
    {
      n_.reset();
    }

    bool got_map_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid const> map_;
    void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid const> map)
    {
      map_ = map;
      got_map_ = true;
    }

    bool got_map_metadata_;
    std::shared_ptr<nav_msgs::msg::MapMetaData const> map_metadata_;
    void mapMetaDataCallback(const std::shared_ptr<nav_msgs::msg::MapMetaData const> map_metadata)
    {
      map_metadata_ = map_metadata;
      got_map_metadata_ = true;
    }
};

/* Try to retrieve the map via service, and compare to ground truth */
TEST_F(MapClientTest, call_service)
{
  auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
  nav_msgs::srv::GetMap::Response resp;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client =
    n_->create_client<nav_msgs::srv::GetMap>("static_map");

  ASSERT_TRUE(client->wait_for_service(5s));

  auto result = client->async_send_request(req);
  auto ret = rclcpp::spin_until_future_complete(n_, result, 5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::executor::FutureReturnCode::SUCCESS);
  resp = *result.get().get();

  ASSERT_FLOAT_EQ(resp.map.info.resolution, g_valid_image_res);
  ASSERT_EQ(resp.map.info.width, g_valid_image_width);
  ASSERT_EQ(resp.map.info.height, g_valid_image_height);
  ASSERT_STREQ(resp.map.header.frame_id.c_str(), "map");
  for(unsigned int i=0; i < resp.map.info.width * resp.map.info.height; i++)
    ASSERT_EQ(g_valid_image_content[i], resp.map.data[i]);
}

/* Try to retrieve the map via topic, and compare to ground truth */
TEST_F(MapClientTest, subscribe_topic)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 1;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  rclcpp::SubscriptionBase::SharedPtr sub = n_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map",
    std::bind(&MapClientTest::mapCallback, this, std::placeholders::_1),
    qos);

  // Try a few times, because the server may not be up yet.
  int i=20;
  while(!got_map_ && i > 0)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.spin_node_once(n_, 1ms);
    rclcpp::sleep_for(250ms);
    i--;
  }
  ASSERT_TRUE(got_map_);
  ASSERT_FLOAT_EQ(map_->info.resolution, g_valid_image_res);
  ASSERT_EQ(map_->info.width, g_valid_image_width);
  ASSERT_EQ(map_->info.height, g_valid_image_height);
  ASSERT_STREQ(map_->header.frame_id.c_str(), "map");
  for(unsigned int i=0; i < map_->info.width * map_->info.height; i++)
    ASSERT_EQ(g_valid_image_content[i], map_->data[i]);
}

/* Try to retrieve the metadata via topic, and compare to ground truth */
TEST_F(MapClientTest, subscribe_topic_metadata)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 1;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr sub =
    n_->create_subscription<nav_msgs::msg::MapMetaData>(
      "map_metadata",
      std::bind(&MapClientTest::mapMetaDataCallback, this, std::placeholders::_1),
      qos);

  // Try a few times, because the server may not be up yet.
  int i=20;
  while(!got_map_metadata_ && i > 0)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.spin_node_once(n_, 1ms);
    rclcpp::sleep_for(250ms);
    i--;
  }
  ASSERT_TRUE(got_map_metadata_);
  ASSERT_FLOAT_EQ(map_metadata_->resolution, g_valid_image_res);
  ASSERT_EQ(map_metadata_->width, g_valid_image_width);
  ASSERT_EQ(map_metadata_->height, g_valid_image_height);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  g_argc = argc;
  g_argv = argv;
  rclcpp::init(g_argc, g_argv);

  return RUN_ALL_TESTS();
}
