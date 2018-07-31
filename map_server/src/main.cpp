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

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include "map_server/image_loader.h"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "yaml-cpp/yaml.h"

#include <rcl/rcl.h>

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res, std::shared_ptr<rclcpp::Node> node)
    {
      std::string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      std::string frame_id;
      node->get_parameter_or("frame_id", frame_id, std::string("map"));
      deprecated = (res != 0);
      if (!deprecated) {
        //mapfname = fname + ".pgm";
        //std::ifstream fin((fname + ".yaml").c_str());
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          RCUTILS_LOG_ERROR("Map_server could not open %s.", fname.c_str())
          exit(-1);
        }
#ifdef HAVE_YAMLCPP_GT_0_5_0
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        try {
          doc["resolution"] >> res;
        } catch (YAML::InvalidScalar &) {
          RCUTILS_LOG_ERROR("The map does not contain a resolution tag or it is invalid.")
          exit(-1);
        }
        try {
          doc["negate"] >> negate;
        } catch (YAML::InvalidScalar &) {
          RCUTILS_LOG_ERROR("The map does not contain a negate tag or it is invalid.")
          exit(-1);
        }
        try {
          doc["occupied_thresh"] >> occ_th;
        } catch (YAML::InvalidScalar &) {
          RCUTILS_LOG_ERROR("The map does not contain an occupied_thresh tag or it is invalid.")
          exit(-1);
        }
        try {
          doc["free_thresh"] >> free_th;
        } catch (YAML::InvalidScalar &) {
          RCUTILS_LOG_ERROR("The map does not contain a free_thresh tag or it is invalid.")
          exit(-1);
        }
        try {
          std::string modeS = "";
          doc["mode"] >> modeS;

          if(modeS=="trinary")
            mode = TRINARY;
          else if(modeS=="scale")
            mode = SCALE;
          else if(modeS=="raw")
            mode = RAW;
          else{
            RCUTILS_LOG_ERROR("Invalid mode tag \"%s\".", modeS.c_str())
            exit(-1);
          }
        } catch (YAML::Exception &) {
          RCUTILS_LOG_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary")
          mode = TRINARY;
        }
        try {
          doc["origin"][0] >> origin[0];
          doc["origin"][1] >> origin[1];
          doc["origin"][2] >> origin[2];
        } catch (YAML::InvalidScalar &) {
          RCUTILS_LOG_ERROR("The map does not contain an origin tag or it is invalid.")
          exit(-1);
        }
        try {
          doc["image"] >> mapfname;
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            RCUTILS_LOG_ERROR("The image tag cannot be an empty string.")
            exit(-1);
          }
          if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
        } catch (YAML::InvalidScalar &) {
          RCUTILS_LOG_ERROR("The map does not contain an image tag or it is invalid.")
          exit(-1);
        }
      } else {
        node->get_parameter_or("negate", negate, 0);
        node->get_parameter_or("occupied_thresh", occ_th, 0.65);
        node->get_parameter_or("free_thresh", free_th, 0.196);
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

      RCUTILS_LOG_INFO("Loading map from image \"%s\"", mapfname.c_str())
      map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
      // TODO(wjwwood): use rclcpp version of Time::now()
      uint32_t now_sec = 0;
      uint32_t now_nanosec = 0;
      {
        rcutils_time_point_value_t now = 0;
        rcl_ret_t ret = rcutils_system_time_now(&now);
        if (ret != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR("Could not get current time: %s", rcl_get_error_string_safe())
          exit(-1);
        }
        now_sec = RCL_NS_TO_S(now);
        now_nanosec = now % (1000 * 1000 * 1000);
      }
      map_resp_.map.info.map_load_time.sec = now_sec;
      map_resp_.map.info.map_load_time.nanosec = now_nanosec;
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp.sec = now_sec;
      map_resp_.map.header.stamp.nanosec = now_nanosec;
      RCUTILS_LOG_INFO(
        "Read a %d X %d map @ %.3lf m/cell",
        map_resp_.map.info.width,
        map_resp_.map.info.height,
        map_resp_.map.info.resolution)
      meta_data_message_ = map_resp_.map.info;

      using namespace std::placeholders;
      service = node->create_service<nav_msgs::srv::GetMap>(
        "static_map", std::bind(&MapServer::mapCallback, this, _1, _2));

      // Latched publisher for metadata
      rmw_qos_profile_t qos = rmw_qos_profile_default;
      qos.depth = 1;
      qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      metadata_pub = node->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", qos);
      metadata_pub->publish(meta_data_message_);

      // Latched publisher for data
      qos = rmw_qos_profile_default;
      qos.depth = 1;
      qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
      map_pub->publish(map_resp_.map);
    }

  public:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr metadata_pub;
    rclcpp::ServiceBase::SharedPtr service;
    bool deprecated;

    /** Callback invoked when someone requests our service */
    void mapCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                     std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
    {
      (void)req;

      // = operator is overloaded to make deep copy (tricky!)
      *res.get() = map_resp_;
      RCUTILS_LOG_INFO("Sending map")
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::msg::MapMetaData meta_data_message_;
    nav_msgs::srv::GetMap::Response map_resp_;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // TODO(wjwwood): make this an anonymous node name
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("map_server");

  if(argc != 3 && argc != 2)
  {
    RCUTILS_LOG_ERROR("%s", USAGE)
    exit(-1);
  }
  if (argc != 2) {
    RCUTILS_LOG_WARN("Using deprecated map server interface. Please switch to new interface.")
  }
  std::string fname(argv[1]);
  double res = (argc == 2) ? 0.0 : atof(argv[2]);

  try
  {
    MapServer ms(fname, res, node);
    rclcpp::spin(node);
  }
  catch(std::runtime_error& e)
  {
    RCUTILS_LOG_ERROR("map_server exception: %s", e.what())
    return -1;
  }

  return 0;
}
