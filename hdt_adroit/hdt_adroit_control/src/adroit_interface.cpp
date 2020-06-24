#define _USE_MATH_DEFINES

#include <hdt_adroit_control/adroit_interface.h>

#include <math.h>
#include <sstream>

// message types
static const int ERROR_TELEM = 1;
static const int HIGH_SPEED_TELEM = 3;
static const int CONTROL_CMD = 4;

// fixed point conversions
static const double POSITION_CONV = 2.0*M_PI;
static const double VELOCITY_CONV = 8.0;
static const double EFFORT_CONV = 256.0;
static const double CURRENT_CONV = 64.0;

/*----------------------------------------------------------------------------
 * constructor
 *----------------------------------------------------------------------------*/
AdroitInterface::AdroitInterface(std::string robot_name, bool fake_execution): robot_name_(robot_name), fake_execution_(fake_execution), control_enabled_(false)
{
  // initialize data
  for (int addr = 0; addr < MAX_DRIVES; addr++)
  {
    cmd_[addr] = 0.0;
    pos_[addr] = 0.0;
    vel_[addr] = 0.0;
    eff_[addr] = 0.0;
  }
}

/*----------------------------------------------------------------------------
 * destructor
 *----------------------------------------------------------------------------*/
AdroitInterface::~AdroitInterface()
{
}

/*----------------------------------------------------------------------------
 * msg callback
 *----------------------------------------------------------------------------*/
void AdroitInterface::msgCallback(const can_msgs::Frame::ConstPtr& msg)
{
  //ROS_INFO("AdroitInterface:msgCallback");
  
  // lock mutex
  std::lock_guard<std::mutex> lock(mutex_);
  
  // push to msg queue
  msg_queue_.push(*msg);
}

/*----------------------------------------------------------------------------
 * init
 *----------------------------------------------------------------------------*/
bool AdroitInterface::init(ros::NodeHandle& nh)
{ 
  //ROS_INFO("AdroitInterface:init robot name = %s", robot_name_.c_str());
  
  ros::NodeHandle robot_nh(nh, robot_name_);
  
  // get joints for given robot
  std::vector<std::string> joints;
  if (!robot_nh.getParam("hardware_interface/joints", joints))
  {
    ROS_ERROR("AdroitInterface:init could not get joints param");
    return false;
  }
  
  //ROS_INFO("AdroitInterface:init found %d joints", (int)joints.size());

  // iterate through joints
  for (std::vector<std::string>::const_iterator joint = joints.begin(); joint != joints.end(); ++joint)
  {
    // look for id param associated with joint
    std::stringstream addr_param;
    addr_param << "hardware_interface/" << *joint << "/addr";
    
    // check for valid addr
    int addr;
    if (robot_nh.getParam(addr_param.str(), addr))
    {
     if ((addr > 0) && (addr < MAX_DRIVES))
     {
        //ROS_INFO("AdroitInterface:init registering %s with id %d", joint->c_str(), id);
        // set enabled to false initially
        addr_map_[addr] = false;
        
        // connect and register state interface
        hardware_interface::JointStateHandle state_handle(*joint, &pos_[addr], &vel_[addr], &eff_[addr]);
        state_interface_.registerHandle(state_handle);

        // connect and register position interface
        hardware_interface::JointHandle pos_handle(state_interface_.getHandle(*joint), &cmd_[addr]);
        pos_interface_.registerHandle(pos_handle);
      }
      // report invalid id
      else
      {
        ROS_ERROR("AdroitInterface:init invalid addr = %d", addr);
        return false;
      }
    }
    else
    {
      ROS_ERROR("AdroitInterface:init could not find param %s", addr_param.str().c_str());
      return false;
    }

    // look for namespace param associated with joint
    std::stringstream namespace_param;
    namespace_param << "hardware_interface/" << *joint << "/namespace";

    // check for namespace
    std::string temp_namespace;
    if (robot_nh.getParam(namespace_param.str(), temp_namespace))
    {
      //ROS_INFO("AdroitInterface:init found param %s is %s", namespace_param.str().c_str(),temp_namespace.c_str());
      namspace_map_[addr] = temp_namespace;
      if(std::find(namespace_list_.begin(), namespace_list_.end(), temp_namespace) != namespace_list_.end()){
        // namespace is already in the list?
      }else{
        namespace_list_.push_back(temp_namespace);
      }
    }
    else
    {
      // namespace is optional for now, if not specfied in hardware_interface.yaml file assume there is no namespace
      // use empty string as place holder for now
      namspace_map_[addr] = "";
      if(std::find(namespace_list_.begin(), namespace_list_.end(), "") != namespace_list_.end()){
        // namespace is already in the list?
      }else{
        namespace_list_.push_back("");
      }
      //ROS_ERROR("AdroitInterface:init could not find param %s", namespace_param.str().c_str());
      //return false;
    }
  }
  
  // register interfaces
  registerInterface(&state_interface_);
  registerInterface(&pos_interface_);
  
  // link to hardware topics, or warn about fake execution
  if (fake_execution_)
  {
    ROS_WARN("AdroitInterface:init fake execution enabled");
  }    
  else
  {
    ROS_INFO("AdroitInterface:init creating topics for %d namespaces",namespace_list_.size());
    for (std::vector<std::string>::iterator it = namespace_list_.begin() ; it != namespace_list_.end(); ++it){
      if(it->empty()){
        // handle non-namespaced drives
        // subscribers
        std::string temp_string;
        temp_string="/received_messages";
        ROS_INFO("\tsubscriber %s in namespace %s",temp_string.c_str(),it->c_str());
        msg_sub_[*it] = nh.subscribe(temp_string.c_str(), 10, &AdroitInterface::msgCallback, this);
  
        // publishers
        temp_string="/sent_messages";
        ROS_INFO("\tpublisher %s in namespace %s",temp_string.c_str(),it->c_str());
        msg_pub_[*it] = nh.advertise<can_msgs::Frame>(temp_string.c_str(), 10);
      }else{
        // handle namespaced drives
        // subscribers
        std::string temp_string;
        temp_string="/";
        temp_string+=*it;
        temp_string+="/received_messages";
        ROS_INFO("\tsubscriber %s in namespace %s",temp_string.c_str(),it->c_str());
        msg_sub_[*it] = nh.subscribe(temp_string.c_str(), 10, &AdroitInterface::msgCallback, this);
  
        // publishers
        temp_string="/";
        temp_string+=*it;
        temp_string+="/sent_messages";
        ROS_INFO("\tpublisher %s in namespace %s",temp_string.c_str(),it->c_str());
        msg_pub_[*it] = nh.advertise<can_msgs::Frame>(temp_string.c_str(), 10);
      }
    }
  }
  
  return true;
}

/*----------------------------------------------------------------------------
 * process msg
 *----------------------------------------------------------------------------*/
bool AdroitInterface::processMsg(const can_msgs::Frame *msg)
{
    // get message id and addr
    uint8_t id = (uint8_t)((msg->id >> 7) & 0x0F);
    uint8_t addr = (uint8_t)(msg->id & 0x7F);
    
    switch (id)
    {
      case HIGH_SPEED_TELEM:
      {
        // get values
	      int16_t pos = msg->data[0] | (msg->data[1] << 8);
	      int16_t vel = msg->data[2] | (msg->data[3] << 8);
	      int16_t eff = msg->data[4] | (msg->data[5] << 8);
	      
	      // set values
	      pos_[addr] = pos/(double)INT16_MAX*POSITION_CONV;
	      vel_[addr] = vel/(double)INT16_MAX*VELOCITY_CONV;
	      eff_[addr] = eff/(double)INT16_MAX*EFFORT_CONV;
	      
	      //ROS_INFO("AdroitInterface:msgCallback drive %d, pos = %3.3f, vel  %3.3f, eff = %3.3f", addr, pos_[addr], vel_[addr], eff_[addr]);
	      
	      // update initial command
	      if (!addr_map_[addr])
	      {
	        cmd_[addr] = pos_[addr];
	        addr_map_[addr] = true;
	        ROS_INFO("AdroitInterface:msgCallback received telemetry from drive %d", addr);
	      }
	      return true;
	    }
	    case ERROR_TELEM:
	    {
	      uint8_t code = msg->data[0];
		    uint8_t severity = msg->data[1] & 0x0F;
		    uint8_t type = (msg->data[1] & 0xF0) >> 4;
		    uint32_t value = msg->data[2] + (msg->data[3] << 8) + (msg->data[4] << 16) + (msg->data[5] << 24);
		    
	      std::stringstream error_msg;
	      error_msg << "AdroitInterface:msgCallback drive " << addr << " reported error code " << code;
	      
	      // get type dependent value
	      switch (type)
	      {
	        case 0:
	          error_msg << " with value " << value;
	          break;
	        case 1:
	        {
	          float value_float;
	          memcpy(&value_float, &value, sizeof(float));
	          error_msg << " with value " << value_float;
	          break;
	        }
	        default:
	          break;
	      }
	      
        // report error
        switch (severity)
        {
          case 0:
            ROS_INFO_STREAM(error_msg.str());
            break;
          case 1:
            ROS_WARN_STREAM(error_msg.str());
            break;
          default:
            ROS_ERROR_STREAM(error_msg.str());
            break;
        }
        return true;
	    }
	    default:
	      return false;
	  }
}

/*----------------------------------------------------------------------------
 * read
 *----------------------------------------------------------------------------*/
bool AdroitInterface::read()
{
  // lock mutex
  std::lock_guard<std::mutex> lock(mutex_);
  
  // process all messages in queue
  while (!msg_queue_.empty())
  {
    // get message at top of queue
    can_msgs::Frame msg = msg_queue_.front();
    
    // process message
    processMsg(&msg);

    // pop msg
    msg_queue_.pop();
  }
  
  // update "telem" for fake execution
  if (fake_execution_)
  {
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      pos_[addr] = cmd_[addr];
      vel_[addr] = 0.0;
      eff_[addr] = 0.0;
    }
    
    return true;
  }
  // otherwise check if we can enable control
  else if (!control_enabled_)
  {
    bool temp = true;
    
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      bool enabled = it->second;
      if (!enabled)
        temp = false;
    }
    
    // update control enabled
    control_enabled_ = temp;
    if (control_enabled_)
      ROS_INFO("AdroitInterface:read control enabled");  
  }
  
  // return control enabled
  return control_enabled_;
}
 
 /*----------------------------------------------------------------------------
 * write
 *----------------------------------------------------------------------------*/
bool AdroitInterface::write()
{ 
  if ((!fake_execution_) && (control_enabled_))
  {
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      can_msgs::Frame msg;
      
      // build msg
      msg.header.stamp = ros::Time::now();
      msg.id = (CONTROL_CMD << 7) | (addr & 0x7F);
      msg.dlc = 8;
      
      // set data
      int16_t pos = (int16_t)(cmd_[addr]*(double)INT16_MAX/POSITION_CONV);
      int16_t vel = (int16_t)(VELOCITY_CONV*(double)INT16_MAX/VELOCITY_CONV);
      int16_t eff = (int16_t)(0.0*(double)INT16_MAX/EFFORT_CONV);
      int16_t cur = (int16_t)(CURRENT_CONV*(double)INT16_MAX/CURRENT_CONV);

      msg.data[0] = pos & 0xFF;
      msg.data[1] = (pos >> 8) & 0xFF;
      msg.data[2] = vel & 0xFF;
      msg.data[3] = (vel >> 8) & 0xFF;
      msg.data[4] = eff & 0xFF;
      msg.data[5] = (eff >> 8) & 0xFF;
      msg.data[6] = cur & 0xFF;
      msg.data[7] = (cur >> 8) & 0xFF;

      // check if this drive ID has a known namespace
      if(namspace_map_.find(addr) != namspace_map_.end()){
        //ROS_INFO("AdroitInterface:write drive %d is in namespace [%s]",addr,namspace_map_[addr].c_str());  
        // send message
        msg_pub_[namspace_map_[addr]].publish(msg);
      }
    }
    
    return true;
  }
  
  return false;
}

