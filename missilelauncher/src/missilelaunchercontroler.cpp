#include "ros/ros.h"
#include "std_msgs/String.h"

#include "missilelauncher.h"

missileLauncher ml;

void sendCommandToLauncherCallback(const std_msgs::String::ConstPtr& CmdRcv)
{
  string cmd = CmdRcv->data.c_str();

    ROS_INFO("sendCommandToLauncherCallback::I heard: [%s]", cmd.c_str());

  if (!strcmp(cmd.c_str(), "up")) {
    ml.sendCommand(cmd);
  } else if (!strcmp(cmd.c_str(), "down")) {
    ml.sendCommand(cmd);
  } else if (!strcmp(cmd.c_str(), "left")) {
    ml.sendCommand(cmd);
  } else if (!strcmp(cmd.c_str(), "right")) {
    ml.sendCommand(cmd);
  } else if (!strcmp(cmd.c_str(), "fire")) {
    ml.sendCommand(cmd);
  }
  ros::Duration(0.2).sleep();
  cmd = "stop";
  ml.sendCommand(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "missilelauncher");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("missilelaunchercmd",
                                    1,
                                    sendCommandToLauncherCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).
   * ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();


  return 0;
}
