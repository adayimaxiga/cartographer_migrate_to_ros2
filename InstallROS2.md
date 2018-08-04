#在ubuntu18.04或者ubuntu16.04上安装ros2-bouncy版本教程

当前，ros2最新版本为2018年7月20日发布的ros2-bouncy版本。在ubuntu18.04环境下支持二进制安装，在ubuntu16.04环境下可以进行源码安装

##ubuntu18.04环境下二进制安装方法

####1.获取软件源

首先需要授权gpg key，安装curl:

<pre><code>sudo apt update && sudo apt install curl
</code></pre>

然后授权gpg key:

<pre><code>curl http://repo.ros2.org/repos.key | sudo apt-key add -
</code></pre>

然后将库添加到源列表：

<pre><code>sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
</code></pre>

####2.安装ros2

首先设置ros2版本环境变量，后面安装需要使用，这里设置为bouncy版本（最新）：

<pre><code>export ROS_DISTRO=bouncy  # or ardent
</code></pre>

更新软件源：

<pre><code>sudo apt update
</code></pre>

安装ros2区分桌面版和基本版，这个与ros1相同。

桌面版包括ros，rviz和一些demo安装，指令如下：

<pre><code>sudo apt install ros-$ROS_DISTRO-desktop
</code></pre>

基本版只有library，messege和一些命令行工具，没有GUI:

<pre><code>sudo apt install ros-$ROS_DISTRO-ros-base
</code></pre>


####3.配置环境

安装argcomplete （虽然是可选项目，但是推荐安装）

对于18.04版本：

<pre><code>sudo apt install python3-argcomplete
</code></pre>

source环境：

<pre><code>source /opt/ros/$ROS_DISTRO/setup.bash
</code></pre>

不想每次都输入的话可以输入下面指令写入.bashrc中
<pre><code>echo "source /opt/ros/bouncy/setup.bash" >> ~/.bashrc
</code></pre>

####4.安装RMW（这个应该就是ros2的实时内核）：

<pre><code>sudo apt update
sudo apt install ros-$ROS_DISTRO-rmw-opensplice-cpp # for OpenSplice
sudo apt install ros-$ROS_DISTRO-rmw-connext-cpp # for RTI Connext (requires license agreement)</code></pre>

通过设置环境变量RMW_IMPLEMENTATION = rmw_opensplice_cpp，可以切换为使用OpenSplice。 对于ROS2版本Bouncy版本，也可以选择RMW_IMPLEMENTATION = rmw_connext_cpp来使用RTI Connext。

####5.安装ros1-bridge：

由于ros2的package还不够完善，因此最好转ros1-bridge，一些ros2没有的package可以使用ros1版本。
<pre><code>sudo apt update
sudo apt install ros-$ROS_DISTRO-ros1-bridge ros-$ROS_DISTRO-turtlebot2-*</code></pre>

到这里安装就完成了。

##ubuntu16.04环境下源码安装ros2-bouncy方法

首先修改时区设置要改成UTF-8，最好顺便语言改成english，设置如下：

<pre><code>sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8</code></pre>

####1.获取软件源

首先需要授权gpg key，安装curl:

<pre><code>sudo apt update && sudo apt install curl
</code></pre>

然后授权gpg key:

<pre><code>curl http://repo.ros2.org/repos.key | sudo apt-key add -
</code></pre>

然后将库添加到源列表：

<pre><code>sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
</code></pre>

####2.配置环境：

安装一些工具：
<pre><code>sudo apt update && sudo apt install -y \
build-essential \
git \
python3-colcon-common-extensions \
python3-pip \
python-rosdep \
python3-vcstool \
wget
# install some pip packages needed for testing
sudo -H python3 -m pip install -U \
argcomplete \
flake8 \
flake8-blind-except \
flake8-builtins \
flake8-class-newline \
flake8-comprehensions \
flake8-deprecated \
flake8-docstrings \
flake8-import-order \
flake8-quotes \
pytest-repeat \
pytest-rerunfailures
# [Ubuntu 16.04] install extra packages not available or recent enough on Xenial
python3 -m pip install -U \
pytest \
pytest-cov \
pytest-runner \
setuptools
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
libasio-dev \
libtinyxml2-dev</code></pre>

获取ros2源码：
<pre><code>mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs import src < ros2.repos
</code></pre>

通过rosdep安装依赖：

<pre><code>sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
</code></pre>

####3.安装dds：

<pre><code>sudo apt install libopensplice67  # from repo.ros2.org
</code></pre>

添加到.bashrc中

<pre><code>export OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml
</code></pre>

获取RTI证书：

<pre><code>export RTI_LICENSE_FILE=path/to/rti_license.dat</code></pre>

安装RTI：

<pre><code>sudo apt install -q -y \
    rti-connext-dds-5.3.1  # from repo.ros2.org</code></pre>

获取安装文件以设置NDDSHOME环境变量：

<pre><code>cd /opt/rti.com/rti_connext_dds-5.3.1/resource/scripts && source ./rtisetenv_x64Linux3gcc5.4.0.bash; cd -</code></pre>

####4.编译：

<pre><code>cd ~/ros2_ws/
colcon build --symlink-install</code></pre>


如果某个包编译失败，就创建一个AMENT_IGNORE的空文件。不编译他了

source时用如下命令
<pre><code>source ~ros2_ws/install/setup.bash</code></pre>




