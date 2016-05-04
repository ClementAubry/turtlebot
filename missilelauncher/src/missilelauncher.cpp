#include "missilelauncher.h"
#include "ctlmissilefunctions.cpp"
#include "ros/ros.h"

missileLauncher::missileLauncher()
{
  busses = NULL;
  bus    = NULL;
  dev    = NULL;
  usb_init();
  usb_find_busses();
  usb_find_devices();
  busses = usb_get_busses();

  for (bus = busses; bus && !dev; bus = bus->next) {
    for (dev = bus->devices; dev; dev = dev->next) {
      if (debug) {
        ROS_INFO("Checking 0x%04x:0x%04x\n",
                 dev->descriptor.idVendor,
                 dev->descriptor.idProduct);
      }

      if ((dev->descriptor.idVendor == 0x1130) &&
          (dev->descriptor.idProduct == 0x0202)) {
        send_command_ms(dev, "stop");
        break;
      }
    }
  }
}

missileLauncher::~missileLauncher()
{}

int missileLauncher::sendCommand(string& command)
{
  ROS_INFO("Sending command 0x%04x:0x%04x\n",
           dev->descriptor.idVendor,
           dev->descriptor.idProduct);

  if ((dev->descriptor.idVendor == 0x1130) &&
      (dev->descriptor.idProduct == 0x0202)) {
    send_command_ms(dev, command.c_str());
  }
  return 1;
}
