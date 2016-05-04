#ifndef _missilelauncher_h
#define _missilelauncher_h

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <usb.h>
using std::string;

class missileLauncher
{
public:
  missileLauncher();
  ~missileLauncher();

  int sendCommand(string& command);

private:
  struct usb_bus *busses, *bus;
  struct usb_device *dev;


};
#endif
