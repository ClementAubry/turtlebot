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
  } else if (strcmp(cmd.c_str(), "stop")) {
    ROS_INFO("sendCommandToLauncherCallback::Unknown command: [%s]", cmd.c_str());
  }
  ros::Duration(0.1).sleep();
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

/**
   int main(int argc, char *argv[])
   {
   struct usb_bus *busses, *bus;
   struct usb_device *dev = NULL;

   if (argc != 2) {
    printf("Usage: ctlmissile [ up | down | left | right | fire ]\n");
    exit(EXIT_FAILURE);
   }

   usb_init();
   usb_find_busses();
   usb_find_devices();

   busses = usb_get_busses();

   for (bus = busses; bus && !dev; bus = bus->next) {
    for (dev = bus->devices; dev; dev = dev->next) {
      if (debug) {
        printf("Checking 0x%04x:0x%04x\n",
          dev->descriptor.idVendor,
          dev->descriptor.idProduct);
      }
      if (dev->descriptor.idVendor == 0x1130 &&
        dev->descriptor.idProduct == 0x0202) {
        send_command_ms(dev, argv[1]);
        break;
      }
      if (dev->descriptor.idVendor == 0x1941 &&
        dev->descriptor.idProduct == 0x8021) {
        send_command_cheeky(dev, argv[1]);
        break;
      }
    }
   }

   if (!dev) {
    fprintf(stderr, "Unable to find device.\n");
    exit(EXIT_FAILURE);
   }

   return 0;
   }
 */
