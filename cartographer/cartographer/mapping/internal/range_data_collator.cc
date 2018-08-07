/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
//trajectory调用了这里来接收rangedata
sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) {
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  // TODO(gaschler): These two cases can probably be one.
  //第一次来不进入if
  if (id_to_pending_data_.count(sensor_id) != 0) {
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    auto result = CropAndMerge();
    id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
    return result;
  }
  //增加id
  id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
  //不想等就先不处理。
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }

  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}
//之前提示Dropped xx earlier points 就是这里，看看这伙在干啥
//大概是，有个overlap_before 和 overlap_end 小于overlap_before的数据直接丢掉，
//         大于overlap_end的数据先留着不处理，只处理before与end之间的数据
//这个before与end时间点选取是在current_start_与current_end_对应，
//            这个current_end就是这次数据的时间，这个current_start_就是上次数据的时间。
//          注意这里有两个时间，一个是每个激光点自带的时间，一个是激光数据包的时间。去sensorbridge看看是怎么给的
//          这里的数据包time就是接收数据的时间直接给了，每个激光的time在msg_conversion.cc里面给的
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  //遍历一圈激光雷达。
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    sensor::TimedPointCloudData& data = it->second;
    sensor::TimedPointCloud& ranges = it->second.ranges;
    //sensor::TimedPointCloud 类型里面，第四项是时间，
    auto overlap_begin = ranges.begin();
    //遍历，寻找时间落后的开始点。
    //时间计算，数据的起始time + 数据点的时间。
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin)[3]) <
               current_start_) {
      ++overlap_begin;
    }
    //遍历，寻找时间落后的结束点。
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end)[3]) <= current_end_) {
      ++overlap_end;
    }
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.origin);
      double time_correction = common::ToSeconds(data.time - current_end_);
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{*overlap_it,
                                                                  origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        point.point_time[3] += time_correction;
        result.ranges.push_back(point);
      }
    }

    // Drop buffered points until overlap_end.
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } else if (overlap_end == ranges.begin()) {
      ++it;
    } else {
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end())};
      ++it;
    }
  }

  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time[3] < b.point_time[3];
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
