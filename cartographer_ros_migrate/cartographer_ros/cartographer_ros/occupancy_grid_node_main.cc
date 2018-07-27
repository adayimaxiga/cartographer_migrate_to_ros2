/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "gflags/gflags.h"
#include "nav_msgs/msg/occupancy_grid.h"

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>

#include <builtin_interfaces/msg/time.hpp>

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;
  ::rclcpp::Node::SharedPtr node_handle();
 private:
  void HandleSubmapList(const cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr msg);
  void DrawAndPublish();
  void PublishOccupancyGrid(const std::string& frame_id, const builtin_interfaces::msg::Time& time,
                            const Eigen::Array2f& origin,
                            cairo_surface_t* surface);

  ::rclcpp::Node::SharedPtr node_handle_;
  const double resolution_;

  ::cartographer::common::Mutex mutex_;

  ::rclcpp::Publisher<::nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  ::rclcpp::Subscription<::cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_subscriber_;
  ::rclcpp::Client<::cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client_;

  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  
  ::rclcpp::TimerBase::SharedPtr occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  builtin_interfaces::msg::Time last_timestamp_;
};

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution)
    //  client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
    //      kSubmapQueryServiceName)),
    //  submap_list_subscriber_(node_handle_.subscribe(
    //      kSubmapListTopic, kLatestOnlyPublisherQueueSize,
    //      boost::function<void(
    //          const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
    //         [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
    //            HandleSubmapList(msg);
    //          }))),
    //  occupancy_grid_publisher_(
    //      node_handle_.advertise<::nav_msgs::OccupancyGrid>(
    //          FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
    //          true /* latched */)),
    //  occupancy_grid_publisher_timer_(
    //      node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
    //                                   &Node::DrawAndPublish, this)) 
{
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  custom_qos_profile.depth = 50;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  
  
  node_handle_ = rclcpp::Node::make_shared("occupancy_grid_map");
  client_ = node_handle_->create_client<::cartographer_ros_msgs::srv::SubmapQuery>(
    "submap_query");

  submap_list_subscriber_ = node_handle_->create_subscription<cartographer_ros_msgs::msg::SubmapList>(
      "submap_list",std::bind(&Node::HandleSubmapList, this,std::placeholders::_1));
  occupancy_grid_publisher_ = node_handle_->create_publisher<::nav_msgs::msg::OccupancyGrid>(
    "map",custom_qos_profile);
  occupancy_grid_publisher_timer_ = node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(publish_period_sec * 1000)),
    std::bind(&Node::DrawAndPublish, this));
    LOG(INFO)<< "config_finished";
    rclcpp::spin(node_handle_);
  
}
void Node::HandleSubmapList(
    const cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr msg) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  LOG(INFO)<< "Submap receive";
  // We do not do any work if occupancy_grid_node_main.cc nobody listens.
  if (node_handle_->count_subscribers(FLAGS_occupancy_grid_topic) == 0) {
    return;
  }

  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }

    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, client_,node_handle_);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::DrawAndPublish() {
  LOG(INFO)<< "occu_timer";
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  LOG(INFO)<< "Occupied grid map Publish";
  ::cartographer::common::MutexLocker locker(&mutex_);
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occupancy_grid_publisher_->publish(*msg_ptr);
}

::rclcpp::Node::SharedPtr Node::node_handle() {return node_handle_;}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {

//  google::InitGoogleLogging(argv[0]);
//  google::ParseCommandLineFlags(&argc, &argv, true);
  
  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  ::rclcpp::init(argc, argv);

//  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, 1);
  LOG(INFO)<< "main start2";
  
  ::rclcpp::shutdown();
}
