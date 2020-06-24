#ifndef adroit_interface_h
#define adroit_interface_h

#include <can_msgs/Frame.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <map>
#include <mutex>
#include <queue>

#define MAX_DRIVES  127

class AdroitInterface : public hardware_interface::RobotHW
{
public:
  AdroitInterface(std::string robot_name, bool fake_execution=false);
  ~AdroitInterface();
  
  bool init(ros::NodeHandle& nh);
  bool read();
  bool write();

private:
  std::string robot_name_;
  
  void msgCallback(const can_msgs::Frame::ConstPtr& msg);
  
  bool processMsg(const can_msgs::Frame *msg);
  
  std::map<std::string, ros::Subscriber> msg_sub_;
  
  std::map<std::string, ros::Publisher> msg_pub_;
  
  hardware_interface::JointStateInterface state_interface_;
  hardware_interface::PositionJointInterface pos_interface_;
  
  std::map<int, bool> addr_map_;
  std::map<int, std::string> namspace_map_;
  std::vector<std::string> namespace_list_;
  
  double cmd_[MAX_DRIVES];
  double pos_[MAX_DRIVES];
  double vel_[MAX_DRIVES];
  double eff_[MAX_DRIVES];
  
  bool fake_execution_;
  bool control_enabled_;
  
  std::mutex mutex_;
  std::queue<can_msgs::Frame> msg_queue_;
};

#endif // adroit_interface_h

