#include <hdt_adroit_control/adroit_interface.h>

#include <controller_manager/controller_manager.h>

#include <sstream>

static const int SAMPLE_RATE = 10.0;

/*----------------------------------------------------------------------------
  main function
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "adroit_control");
  
  // async spinner for controller manager(??)
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  // get private params
  ros::NodeHandle private_nh("~");
  std::string robot_name;
  bool fake_execution;
  
  if (!private_nh.getParam("robot_name", robot_name))
  {
    ROS_ERROR("Robot name not provided, exiting...");
    return 0;
  }
  private_nh.param<bool>("fake_execution", fake_execution, false);
  
  // hardware interface
  ros::NodeHandle nh;
  AdroitInterface robot(robot_name, fake_execution);
  if (robot.init(nh) == false)
    return 0;
  
  // controller manager
  controller_manager::ControllerManager manager(&robot);
 
  // look for sample rate
  std::stringstream loop_hz_param;
  loop_hz_param << robot_name << "/hardware_interface/loop_hz";
  double loop_hz;
  if (!nh.getParam(loop_hz_param.str(), loop_hz))
  {
    // otherwise set to default
    loop_hz= SAMPLE_RATE;
  }
  
  // update loop
  ros::Rate rate(loop_hz);
  ros::Time last = ros::Time::now();
  while (ros::ok())
  {
    // get time
    ros::Time now = ros::Time::now();
    ros::Duration period = now - last;
    //ROS_INFO_STREAM("AdroitControl:main period " << period);
    
    // read from hw interface, check if hw is ready
    if (robot.read())
    {
      // update controller manager
      manager.update(now, period);
    
      // write to hw interface
      robot.write();
    }
      
    // update time and sleep
    last = now;
    rate.sleep();
  }
  
  // wait for shutdown
  ros::waitForShutdown();
    
  return 0;
}
