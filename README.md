# ros2 环境下cartographer与amcl实现

##主要功能
主要实现双激光雷达在ros2环境下建图与定位。

建图算法采用cartographer，移植到ros2环境

定位算法用amcl，针对双激光雷达修改观测模型


##cartographer部分

移植cartographer_ros 2018-07-20版本到ros2环境，cartographer_ros谷歌官方git地址https://github.com/googlecartographer/cartographer_ros.git


测试cartographer版本  cartographer 2018-07-22版本


测试ros环境: ros-ardent,ros-bouncy

依赖包:


boost | ceres |protobuf 3.0.0 | cartographer 


###主要修改部分

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

##amcl部分

这部分主要是要修改观测模型，首先阅读amcl源码：
首先阅读amcl_node.cpp

这个文件结构比较清晰，主函数中调用AmclNode构造函数，这个函数主要干了这么几件事：读取参数，初始化位置，发布/订阅话题（发布话题：amcl_pose，particlecloud）（订阅话题：initialpose，map）（服务：global_localization，request_nomotion_update，set_map）

最关键的一个函数是激光雷达话题的回调函数，所有算法在这里运行。来看laserReceived函数都在干啥：

1)首先获取了laser相对于baselink的tf

2)然后获取了baselink相对于odom的tf，这样就知道laser在世界坐标系下的位置可以继续进行匹配了。

3)接下来分为两种情况，pf是否第一次运行，如果是第一次就将里程计值赋给pf_odom_pose_。如果是正常迭代过程，则计算里程计变化值，并判断里程计变化是不是达到了我们要进行更新的阈值。

4)里程计运动达到更新阈值即调用UpdateAction()函数，打开该函数，这里可以看出来是根据设置的运动预测模型进行采样，这就是粒子滤波的预测模型了。

5)然后回到laserReceived函数中，读取了激光数据后，将最大范围与最小范围传给amcl_laser。然后由于amcl算法并未对最小距离做处理，因此如果小于range_min就赋值为range_max做与max同样处理。之后处理好的数据调用UpdateSensor，将激光数据上传，我们打开这个函数

6)UpdateSensor对于激光数据处理，根据模型不同调用了pf_update_sensor，一般情况下，激光观测模型为LikelihoodFieldModel这个经典模型。这里根据似然域来做更新。

7)看一下LikelihoodFieldModel这个函数，这里我理解的是由于里程计运动大于一定值才进行预测及更新，这里会取与里程计时间相近的一次激光，模型里计算流程大概是：对于每一个粒子，首先计算激光坐标系，然后滤去大于max_range的激光点（这里由于上一步的处理，连min_range一起滤掉了），滤去not a number,计算每一束激光落点栅格，计算落点栅格与地图上障碍点最近曼哈顿距离，根据激光测距的高斯模型概率，每个粒子权重为每一束激光单独扫描的乘积，再计算总权重。这里是激光观测模型的计算流程。

8)回到laserReceived函数中，这里完成更新部分之后进行重采样，重采样函数为pf_update_resample()函数。重采样的方法比较经典，根据w值增加了一部分随机生成的粒子，当drand48<w_diff，随机均匀在地图范围内产生一个粒子。这里是调用uniformPoseGenerator产生的。而drand48>w_diff时，将PDF积分得到CDF，然后drand48来对应离散CDF中采样，这样drand48落在哪个点上就把哪个粒子复制一份，将所有粒子权重置为1，完成重采样。然后对每个粒子权重进行归一化，重置fast和slow，重新计算统计数据（均值，方差）

9)回到laserReceived函数中，最后一步是发布计算好的点云数据。

阅读源码注释在源码中

2018.8.1  LD


##在ubuntu18.04或者ubuntu16.04上安装ros2-bouncy版本教程

见InstallROS2.md

2018.8.2  LD

##删除cartographer文件中的.git，一起上传了
2018.8.4  LD