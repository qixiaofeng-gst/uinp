# >>>>>>> Information collected from http://wiki.ros.org/melodic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full

# Belows are environment setup for ROS
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# Dependencies for building ROS packages
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
# Initialize rosdep
sudo rosdep init
rosdep update

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/CreatingPackage
cd ~/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
cd ~/catkin_ws
catkin_make

# Add created workspace to ROS environment
. ~/catkin_ws/devel/setup.bash
rospack depends1 beginner_tutorials
roscd beginner_tutorials
rospack depends1 rospy
rospack depends beginner_tutorials
atom package.xml

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/BuildingPackages
# In a catkin workspace
catkin_make
catkin_make install
# OR
catkin_make --source my_src
catkin_make install --source my_src

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes
sudo apt-get install ros-melodic-ros-tutorials
# Nodes: A node is an executable that uses ROS to communicate with other nodes.
# Messages: ROS data type used when subscribing or publishing to a topic.
# Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
# Master: Name service for ROS (i.e. helps nodes find each other)
# rosout: ROS equivalent of stdout/stderr
# roscore: Master + rosout + parameter server (parameter server will be introduced later)

# Run roscore
roscore
# Open a new terminal
rosnode list
rosnode info /rosout
rosrun turtlesim turtlesim_node
# OR
rosrun turtlesim turtlesim_node __name:=my_turtle
# Open a new terminal
rosnode list
rosnode ping my_turtle

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
# Ensure that turtlesim_node is running
rosrun turtlesim turtle_teleop_key
sudo apt-get install ros-melodic-rqt
sudo apt-get install ros-melodic-rqt-common-plugins
rosrun rqt_graph rqt_graph
rostopic -h
rostopic echo /turtle1/cmd_vel
# Refresh on rqt_graph GUI could see new subscription
rostopic list -h
rostopic list -v
rostopic type /turtle1/cmd_vel
rosmsg show geometry_msgs/Twist
# -1 means only pulbish one mesage, then follows topic, then message type, -- means following is not an option, then follows value in YAML syntax
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
# -r publish a steady stream of commands
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
rostopic hz /turtle1/pose
rostopic type /turtle1/cmd_vel | rosmsg show
rosrun rqt_plot rqt_plot

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams
rosservice list
rosservice type /clear
rosservice call /clear
rosservice type /spawn | rossrv show
rosservice call /spawn 2 2 0.2 ""

rosparam list
rosparam set /turtlesim/background_r 150
rosservice call /clear
rosparam get /turtlesim/background_b
rosparam get /
rosparam dump params.yaml
rosparam load params.yaml copy
rosparam get /copy/turtlesim/background_b

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
rosrun turtlesim turtlesim_node
# Change the logger level to warn
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
# Close old turtlesim nodes.
cd ~/catkin_ws
. devel/setup.bash
roscd beginner_tutorials
mkdir launch && cd launch
touch turtlemimic.launch
# Fill the launch file with proper content
roslaunch beginner_tutorials turtlemimic.launch
rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
rqt
# Plugins > Introspection > Node Graph
# OR rqt_graph instead of rqt

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/UsingRosEd
rosed roscpp Logger.msg

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
# msg files are stored in msg directory, srv files stored in srv directory
# field types:
#   int8/16/32/64, plus uint*
#   float32/64
#   string
#   time, duration
#   other msg files
#   variable-length array[] and fixed-length array[C]
# Header is a special ROS type, it contains a timestamp and coordinate frame information that are commonly used in ROS
# An msg example:
#   Header header
#   string child_frame_id
#   geometry_msgs/PoseWithCovariance pose
#   geometry_msgs/TwistWithCovariance twist
# srv files are just like msg files, except they contain two parts: a request and a response.
# The two parts are separated by a --- line. An srv example:
#   int64 A
#   int64 B
#   ---
#   int64 sum
# Using msg:
roscd beginner_tutorials
mkdir msg
echo "int64 num" > msg/Num.msg
# Open package.xml, make sure below two lines exist:
#   <build_depend>message_generation</build_depend>
#   <exec_depend>message_runtime</exec_depend>
# Add belows to CMakeLists.txt:
#   find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     rospy
#     std_msgs
#     message_generation
#   )
# Make sure export messsage runtime:
#   catkin_package(
#     ...
#     CATKIN_DEPENDS message_runtime ...
#     ...)
# And add:
#   add_message_files(
#     FILES
#     Num.msg)
# Then and:
#   generate_messages(
#     DEPENDENCIES
#     std_msgs)
rosmsg show beginner_tutorial/Num
rosmsg show Num
# Using srv
roscd beginner_tutorials
mkdir srv
roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
# Make sure belows exist in CMakeLists.txt
#   add_service_files(
#     FILES
#     AddTwoInts.srv)
rossrv show beginner_tutorials/AddTwoInts
rossrv show AddTwoInts
# Build them:
catkin_make install
cd -

# >>>>>>> Informations collected from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
roscd beginner_tutorials
mkdir -p src
# Copy https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp into src/talker.cpp
# Copy https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp into src/listener.cpp
# Use add_executable target_link_libraries and add_dependencies to let talker and listener be setup correctly.
cd ~/catkin_ws && catkin_make

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber
roscore
cd ~/catkin_ws && source ./devel/setup.sh
rosrun beginner_tutorials talker
rosrun beginner_tutorials listener

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
roscd beginner_tutorials
# Create src/add_two_ints_server.cpp with:
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
# Create src/add_two_ints_client.cpp with:
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
# Add CMakeList.txt contents with add_executable, target_link_libraries and add_dependencies then:
cd ~/catkin_ws && catkin_make
rosrun beginner_tutorials add_two_ints_client 1 3

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
rostopic list -v
mkdir ~/bagfiles && cd ~/bagfiles
rosbag record -a
# -a above means accumulate topics
rosbag info <bagfile>
rosbag play <bagfile>
rosbag play -r 2 <bagfile>
# above -r means playback rate
rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
# above -O means output to subset.bag file. Following topics limit the record scope.
# Play turtle with keyboard arrows, then ctrl-c the record console.
rosbag info subset.bag

# >>>>>>> Information collected from http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf
roscd rosmaster
roswtf
roscd
roswtf
roscd
ROS_PACKAGE_PATH=bad:$ROS_PACKAGE_PATH roswtf

# >>>>>>> End of beginner tutorials. Belows are some immediate links:
# http://wiki.ros.org/ROS/NetworkSetup
# http://wiki.ros.org/ROS/Troubleshooting
# http://wiki.ros.org/ROS/Tutorials/MultipleMachines
# http://wiki.ros.org/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python
# http://wiki.ros.org/tf/Tutorials
# http://wiki.ros.org/urdf/Tutorials

