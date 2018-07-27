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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "glog/logging.h"

#include <nav_msgs/msg/odometry.hpp>
//#include "nav_msgs/Odometry.h"

//#include "ros/serialization.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
//#include "sensor_msgs/PointCloud2.h"

#include "tf2_eigen/tf2_eigen.h"

#include "visualization_msgs/msg/marker_array.hpp"
//#include "visualization_msgs/MarkerArray.h"


#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>







#include "gflags/gflags.h"

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

cartographer_ros_msgs::msg::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::msg::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;
  topics.nav_sat_fix_topic = kNavSatFixTopic;
  topics.landmark_topic = kLandmarkTopic;
  return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr),
    const int trajectory_id, const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, Node* const node,
    rmw_qos_profile_t custom_qos_profile) {
  return node_handle->create_subscription<MessageType>(
      topic,
      [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
            (node->*handler)(trajectory_id, topic, msg);
      },
      custom_qos_profile);
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer, const bool collect_metrics)
    : node_options_(node_options),
      resolution_(FLAGS_resolution),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer) {
  carto::common::MutexLocker lock(&mutex_);
  if (collect_metrics) {
    metrics_registry_ = carto::common::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  custom_qos_profile.depth = 50;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  node_handle_ = rclcpp::Node::make_shared("cartographer_node");



  submap_list_publisher_ =
      node_handle_->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
          kSubmapListTopic, custom_qos_profile);
  //submap_list_publisher_ =
  //    node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
  //        kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kTrajectoryNodeListTopic, custom_qos_profile);
  //trajectory_node_list_publisher_ =
  //    node_handle_.advertise<::visualization_msgs::MarkerArray>(
  //        kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kLandmarkPosesListTopic, custom_qos_profile);
  //landmark_poses_list_publisher_ =
  //    node_handle_.advertise<::visualization_msgs::MarkerArray>(
  //        kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kConstraintListTopic, custom_qos_profile);
  //constraint_list_publisher_ =
  //    node_handle_.advertise<::visualization_msgs::MarkerArray>(
  //        kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  

  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName, std::bind(&Node::HandleSubmapQuery, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName, std::bind(&Node::HandleStartTrajectory, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName, std::bind(&Node::HandleFinishTrajectory, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::WriteState>(
      kWriteStateServiceName, std::bind(&Node::HandleWriteState, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::GetTrajectoryStates>(
      kGetTrajectoryStatesServiceName, std::bind(&Node::HandleGetTrajectoryStates, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::ReadMetrics>(
      kReadMetricsServiceName, std::bind(&Node::HandleReadMetrics, this, std::placeholders::_1, std::placeholders::_2)));
  //service_servers_.push_back(node_handle_.advertiseService(
  //    kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  //service_servers_.push_back(node_handle_.advertiseService(
  //    kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  //service_servers_.push_back(node_handle_.advertiseService(
  //    kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  //service_servers_.push_back(node_handle_.advertiseService(
  //    kWriteStateServiceName, &Node::HandleWriteState, this));
  //service_servers_.push_back(node_handle_.advertiseService(
  //    kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));
  //service_servers_.push_back(node_handle_.advertiseService(
  //    kReadMetricsServiceName, &Node::HandleReadMetrics, this));
  occupancy_grid_publisher_ =
      node_handle_->create_publisher<::nav_msgs::msg::OccupancyGrid>(
  kOccupancyGridTopic, custom_qos_profile);


  scan_matched_point_cloud_publisher_ =
      node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>(
          kScanMatchedPointCloudTopic, custom_qos_profile);
  //scan_matched_point_cloud_publisher_ =
  //    node_handle_.advertise<sensor_msgs::PointCloud2>(
  //        kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);


  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
    std::bind(&Node::PublishSubmapList, this)));
  //wall_timers_.push_back(node_handle_.createWallTimer(
  //    ::ros::WallDuration(node_options_.submap_publish_period_sec),
  //    &Node::PublishSubmapList, this));

  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_->create_wall_timer(
        std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
        std::bind(&Node::PublishLocalTrajectoryData, this));
  }

  //if (node_options_.pose_publish_period_sec > 0) {
  //  publish_local_trajectory_data_timer_ = node_handle_.createTimer(
  //      ::ros::Duration(node_options_.pose_publish_period_sec),
  //      &Node::PublishLocalTrajectoryData, this);
  //}
  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    std::bind(&Node::PublishTrajectoryNodeList, this)));
  //wall_timers_.push_back(node_handle_.createWallTimer(
  //    ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
  //    &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    std::bind(&Node::PublishLandmarkPosesList, this)));

  //wall_timers_.push_back(node_handle_.createWallTimer(
  //    ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
  //    &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
    std::bind(&Node::PublishConstraintList, this)));
  
  //wall_timers_.push_back(node_handle_.createWallTimer(
  //    ::ros::WallDuration(kConstraintPublishPeriodSec),
  //    &Node::PublishConstraintList, this));

  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(1 * 1000)),
    std::bind(&Node::DrawAndPublish, this)));
  




  ts_ = std::make_shared<rclcpp::TimeSource>(node_handle_);
  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts_->attachClock(clock_);
}

Node::~Node() { FinishAllTrajectories(); }

::rclcpp::Node::SharedPtr Node::node_handle() { return node_handle_; }

void Node::HandleSubmapQuery(
    const std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Response> response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);
  return;
}

void Node::PublishSubmapList() {
  carto::common::MutexLocker lock(&mutex_);
  
  LOG(INFO)<< "Submap receive";
  cartographer_ros_msgs::msg::SubmapList msg=map_builder_bridge_.GetSubmapList(clock_);
  
  submap_list_publisher_->publish(msg);

  std::set<SubmapId> submap_ids_to_delete;

  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto& submap_msg : msg.submap) {
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

    auto request = std::make_shared<::cartographer_ros_msgs::srv::SubmapQuery::Request>();
    request -> trajectory_id = id.trajectory_id;
    request -> submap_index = id.submap_index;

    auto result = std::make_shared<::cartographer_ros_msgs::srv::SubmapQuery::Response>();
    map_builder_bridge_.HandleSubmapQuery(request, result);

    if (result.get()->textures.empty()) {
    return ;
    }

    auto response =
      ::cartographer::common::make_unique<::cartographer::io::SubmapTextures>();
    response->version = result.get()->submap_version;
    for (const auto& texture : result.get()->textures) {
      const std::string compressed_cells(texture.cells.begin(),
                                        texture.cells.end());
     response->textures.emplace_back(::cartographer::io::SubmapTexture{
          ::cartographer::io::UnpackTextureData(compressed_cells, texture.width,
                                               texture.height),
          texture.width, texture.height, texture.resolution,
          ToRigid3d(texture.slice_pose)});
    }

    if (response == nullptr) {
      continue;
    }
    CHECK(!response->textures.empty());
    submap_slice.version = response->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = response->textures.begin();
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

  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg.header.stamp;
  last_frame_id_ = msg.header.frame_id;



}

void Node::DrawAndPublish() {
//  LOG(INFO)<< "occu_timer";
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

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Node::PublishLocalTrajectoryData() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_data.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      if (node_handle_->count_subscribers(kScanMatchedPointCloudTopic) > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                                .returns.size());
        for (const Eigen::Vector3f point :
             trajectory_data.local_slam_data->range_data_in_local.returns) {
          Eigen::Vector4f point_time;
          point_time << point, 0.f;
          point_cloud.push_back(point_time);
        }
        scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),
            node_options_.map_frame,
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_data.local_to_map.cast<float>())));
      }
      extrapolator.AddPose(trajectory_data.local_slam_data->time,
                           trajectory_data.local_slam_data->local_pose);
    }

    geometry_msgs::msg::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        FromRos(clock_->now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp =
        node_options_.use_pose_extrapolator
            ? ToRos(now)
            : ToRos(trajectory_data.local_slam_data->time);
    const Rigid3d tracking_to_local_3d =
        node_options_.use_pose_extrapolator
            ? extrapolator.ExtrapolatePose(now)
            : trajectory_data.local_slam_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));
      }
      return tracking_to_local_3d;
    }();

    const Rigid3d tracking_to_map =
        trajectory_data.local_to_map * tracking_to_local;

    if (trajectory_data.published_to_tracking != nullptr) {
      if (trajectory_data.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_data.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_data.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_data.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_data.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_data.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_->sendTransform(stamped_transforms);
      } else {
        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_data.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_data.published_to_tracking));
        tf_broadcaster_->sendTransform(stamped_transform);
      }
    }
  }
}

void Node::PublishTrajectoryNodeList() {
  if (node_handle_->count_subscribers(kTrajectoryNodeListTopic) > 0) {
    carto::common::MutexLocker lock(&mutex_);
    trajectory_node_list_publisher_->publish(
        map_builder_bridge_.GetTrajectoryNodeList(clock_));
  }
}

void Node::PublishLandmarkPosesList() {
  if (node_handle_->count_subscribers(kLandmarkPosesListTopic) > 0) {
    carto::common::MutexLocker lock(&mutex_);
    landmark_poses_list_publisher_->publish(
        map_builder_bridge_.GetLandmarkPosesList(clock_));
  }
}

void Node::PublishConstraintList() {
  if (node_handle_->count_subscribers(kConstraintListTopic) > 0) {
    carto::common::MutexLocker lock(&mutex_);
    constraint_list_publisher_->publish(
      map_builder_bridge_.GetConstraintList(clock_));
  }
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(
    const TrajectoryOptions& options,
    const cartographer_ros_msgs::msg::SensorTopics& topics) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, topics.imu_topic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(
        SensorId{SensorType::ODOMETRY, topics.odometry_topic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, topics.nav_sat_fix_topic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::msg::SensorTopics& topics) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options, topics);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, topics, trajectory_id);

  //wall_timers_.push_back(node_handle_->create_wall_timer(
  //  std::chrono::milliseconds(int(kTopicMismatchCheckDelaySec * 1000)),
  //  std::bind(&Node::MaybeWarnAboutTopicMismatch, this),true));

  //wall_timers_.push_back(node_handle_.createWallTimer(
  //    ::ros::WallDuration(kTopicMismatchCheckDelaySec),
  //    &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::msg::SensorTopics& topics,
                             const int trajectory_id) {
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  custom_qos_profile.depth = 50;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, node_handle_,
             this, custom_qos_profile),
         topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             node_handle_, this,custom_qos_profile),
         topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             node_handle_, this,custom_qos_profile),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    std::string topic = topics.imu_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, topic,
                                                node_handle_, this, custom_qos_profile),
         topic});
  }

  if (options.use_odometry) {
    std::string topic = topics.odometry_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, topic,
                                                  node_handle_, this, custom_qos_profile),
         topic});
  }
  if (options.use_nav_sat) {
    std::string topic = topics.nav_sat_fix_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, topic, node_handle_,
             this, custom_qos_profile),
         topic});
  }
  if (options.use_landmarks) {
    std::string topic = topics.landmark_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, topic, node_handle_,
             this, custom_qos_profile),
         topic});
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(
    const ::cartographer_ros_msgs::msg::SensorTopics& topics,
    const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options, topics)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::msg::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  auto trajectory_states = map_builder_bridge_.GetTrajectoryStates();

  cartographer_ros_msgs::msg::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    const std::string message = "Trajectory " + std::to_string(trajectory_id) +
                                " already pending to finish.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
    status_response.message = message;
    LOG(INFO) << message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  if (!(trajectory_states.count(trajectory_id))) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " doesn't exist.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    status_response.message = error;
    return status_response;
  } else if (trajectory_states.at(trajectory_id) == TrajectoryState::FROZEN) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is frozen.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    status_response.message = error;
    return status_response;
  } else if (trajectory_states.at(trajectory_id) == TrajectoryState::FINISHED) {
    const std::string error = "Trajectory " + std::to_string(trajectory_id) +
                              " has already been finished.";
    LOG(ERROR) << error;
    status_response.code =
        cartographer_ros_msgs::msg::StatusCode::RESOURCE_EXHAUSTED;
    status_response.message = error;
    return status_response;
  } else if (trajectory_states.at(trajectory_id) == TrajectoryState::DELETED) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " has been deleted.";
    LOG(ERROR) << error;
    status_response.code =
        cartographer_ros_msgs::msg::StatusCode::RESOURCE_EXHAUSTED;
    status_response.message = error;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      //entry.subscriber.shutdown();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  const std::string message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
  status_response.message = message;
  return status_response;
}

void Node::HandleStartTrajectory(
    const std::shared_ptr<::cartographer_ros_msgs::srv::StartTrajectory::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::StartTrajectory::Response> response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request->options, &options) ||
      !ValidateTrajectoryOptions(options)) {
    const std::string error = "Invalid trajectory options.";
    LOG(ERROR) << error;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message = error;
  } else if (!ValidateTopicNames(request->topics, options)) {
    const std::string error = "Invalid topics.";
    LOG(ERROR) << error;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message = error;
  } else {
    response->trajectory_id = AddTrajectory(options, request->topics);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message = "Success.";
  }
  return;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options, DefaultSensorTopics());
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id :
         ComputeExpectedSensorIds(bags_options.at(i), DefaultSensorTopics())) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

void Node::HandleGetTrajectoryStates(
    const std::shared_ptr<::cartographer_ros_msgs::srv::GetTrajectoryStates::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::GetTrajectoryStates::Response> response) {
  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  carto::common::MutexLocker lock(&mutex_);
  response->status.code = ::cartographer_ros_msgs::msg::StatusCode::OK;
  response->trajectory_states.header.stamp = clock_->now();
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    response->trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::DELETED);
        break;
    }
  }
  return;
}

void Node::HandleFinishTrajectory(
    const std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Response> response) {
  (void)response;
  carto::common::MutexLocker lock(&mutex_);
  FinishTrajectoryUnderLock(request->trajectory_id);
  return;
}

void Node::HandleWriteState(
    const std::shared_ptr<::cartographer_ros_msgs::srv::WriteState::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::WriteState::Response> response) {
  carto::common::MutexLocker lock(&mutex_);
  if (map_builder_bridge_.SerializeState(request->filename)) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message = "State written to '" + request->filename + "'.";
  } else {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message = "Failed to write '" + request->filename + "'.";
  }
  return;
}

void Node::HandleReadMetrics(
    const std::shared_ptr<::cartographer_ros_msgs::srv::ReadMetrics::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::ReadMetrics::Response> response) {
  carto::common::MutexLocker lock(&mutex_);
  response->timestamp = clock_->now();
  if (!metrics_registry_) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::UNAVAILABLE;
    response->status.message = "Collection of runtime metrics is not activated.";
    return;
  }
  metrics_registry_->ReadMetrics(response);
  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  response->status.message = "Successfully read metrics.";
  return;
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::msg::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  carto::common::MutexLocker lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::msg::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(map_builder_bridge_.SerializeState(filename))
      << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);
}
/*
void Node::MaybeWarnAboutTopicMismatch() {
  ::ros::master::V_TopicInfo ros_topics;
  ::ros::master::getTopics(ros_topics);
  std::set<std::string> published_topics;
  std::stringstream published_topics_string;
  for (const auto& it : ros_topics) {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }
  bool print_topics = false;
  for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
      if (published_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
  if (print_topics) {
    LOG(WARNING) << "Currently available topics are: "
                 << published_topics_string.str();
  }
}
*/

}  // namespace cartographer_ros
