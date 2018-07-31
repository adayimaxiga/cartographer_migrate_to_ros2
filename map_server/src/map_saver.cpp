/*
 * map_saver
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include <cstdio>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

/**
 * @brief Map generation node.
 */
class MapGenerator
{

  public:
    MapGenerator(const std::string& mapname, rclcpp::Node::SharedPtr node)
    : mapname_(mapname), saved_map_(false)
    {
      printf("Waiting for the map\n");
      rmw_qos_profile_t qos = rmw_qos_profile_default;
      qos.depth = 1;
      map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        std::bind(&MapGenerator::mapCallback, this, std::placeholders::_1),
        qos
      );
    }

    void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> map)
    {
      printf("Received a %d X %d map @ %.3f m/pix\n",
               map->info.width,
               map->info.height,
               map->info.resolution);


      std::string mapdatafile = mapname_ + ".pgm";
      printf("Writing map occupancy data to %s\n", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        fprintf(stderr, "[ERROR] Couldn't save map file to %s\n", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] == 0) { //occ [0,0.1)
            fputc(254, out);
          } else if (map->data[i] == +100) { //occ (0.65,1]
            fputc(000, out);
          } else { //occ [0.1,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);


      std::string mapmetadatafile = mapname_ + ".yaml";
      printf("Writing map occupancy data to %s\n", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

       */

      geometry_msgs::msg::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml,
        "image: %s\n"
        "resolution: %f\n"
        "origin: [%f, %f, %f]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n\n",
        mapdatafile.c_str(),
        map->info.resolution,
        map->info.origin.position.x,
        map->info.origin.position.y,
        yaw);

      fclose(yaml);

      printf("Done\n");
      saved_map_ = true;
    }

    std::string mapname_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    bool saved_map_;

};

#define USAGE "Usage: \n" \
              "  map_saver -h\n" \
              "  map_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_saver");
  std::string mapname = "map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  MapGenerator mg(mapname, node);

  while(!mg.saved_map_ && rclcpp::ok()) {
    rclcpp::spin_some(node);
  }

  return 0;
}


