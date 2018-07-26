# cartographer_migrate_to_ros2


移植cartographer_ros 2018-07-20版本到ros2环境，cartographer_ros谷歌官方git地址https://github.com/googlecartographer/cartographer_ros.git


测试cartographer版本  cartographer 2018-07-22版本


测试ros环境: ros-ardent,ros-bouncy

依赖包:

boost
ceres
protobuf 3.0.0


##主要修改部分

1.消息接口：比如一个标准的激光雷达扫描，在ros1中为sensor_msgs::Laserscan而在ros2中为sensor_msgs::msg::Laserscan。

2.自定义消息：以cartographer_ros_msgs::msg::Metric为例，包含的头文件为cartographer_ros_msgs/msg/metric.hpp,注意：这里ros2生成消息头文件时会自动将驼峰法命名方式修改为匈牙利法即全小写下划线区分。以及包含的不再是.h文件，是hpp文件。打开hpp文件，发现只有如下两行代码，应该是ros2为了实现dds而对消息进一步封装。
<pre><code>
#include "cartographer_ros_msgs/msg/metric__struct.hpp"

#include "cartographer_ros_msgs/msg/metric__traits.hpp"
</code></pre>
 

3.类调用：ros2大量使用智能指针，因此大部分类都采用智能指针进行初始化，而调用类内函数也需要用指针的方式，这里需要修改很多地方，比如发布话题，ros1中为topic.publish()而ros2中修改为topic->publish

4.时间接口：ros1中时间戳ros::Time以及时间长度Duration都没有对应的ros2中函数，而且麻烦的是ros1中时间单位与ros2中不同。在ros2中引入了builtin_interfaces::msg::Time 消息类型，用来取代ros::Time移植过程如下

这里我查了一下ros2标准msg的时间戳类型，比如geometry::msg::Odometry消息，他的时间戳类型为builtin_interfaces/time，因此将所有时间戳变量用该值替代。而获取当前时间本来采用的是ros::time::now()函数，这里修改成clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME); 然后调用clock_->now()来实现，这部分函数在rclcpp/clock.hpp里面有实现，还是一个智能指针。

2018.07.26 LD