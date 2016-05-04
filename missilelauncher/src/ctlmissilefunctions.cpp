/*
 * ctlmissile.c - simple code to control USB missile launchers.
 *
 * Copyright 2006 James Puderer <jpuderer@littlebox.ca>
 * Copyright 2006 Jonathan McDowell <noodles@earth.li>
 *
 * This is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; version 2.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>
#include "ros/ros.h"

int debug = 1;

/*
 * Command to control original M&S USB missile launcher.
 */
void send_command_ms(struct usb_device *dev, const char *cmd)
{
    ROS_INFO("DEBUT \n");
  usb_dev_handle *launcher;
  char data[64];
  int  ret;

  launcher = usb_open(dev);

  if (launcher == NULL) {
    ROS_INFO("Unable to open device");
    exit(EXIT_FAILURE);
  }

  /* Detach kernel driver (usbhid) from device interface and claim */
  usb_detach_kernel_driver_np(launcher, 0);
  usb_detach_kernel_driver_np(launcher, 1);

  ret = usb_set_configuration(launcher, 1);

  if (ret < 0) {
    ROS_INFO("Unable to set device configuration");
    exit(EXIT_FAILURE);
  }

  ret = usb_claim_interface(launcher, 1);

  if (ret < 0) {
    ROS_INFO("Unable to claim interface");
    exit(EXIT_FAILURE);
  }

  ret = usb_set_altinterface(launcher, 0);

  if (ret < 0) {
    ROS_INFO("Unable to set alternate interface");
    exit(EXIT_FAILURE);
  }

  data[0] = 'U';
  data[1] = 'S';
  data[2] = 'B';
  data[3] = 'C';
  data[4] = 0;
  data[5] = 0;
  data[6] = 4;
  data[7] = 0;
  ret     = usb_control_msg(launcher,
                            USB_DT_HID,                // request type
                            USB_REQ_SET_CONFIGURATION, // request
                            USB_RECIP_ENDPOINT,        // value
                            1,                         // index
                            data,                      // data
                            8,                         // Length of data.
                            500);                       // Timeout

  if (ret != 8) {
    ROS_INFO("Error: %s\n", usb_strerror());
    exit(EXIT_FAILURE);
  }

  data[0] = 'U';
  data[1] = 'S';
  data[2] = 'B';
  data[3] = 'C';
  data[4] = 0;
  data[5] = 0x40;
  data[6] = 2;
  data[7] = 0;
  ret     = usb_control_msg(launcher,
                            USB_DT_HID,
                            USB_REQ_SET_CONFIGURATION,
                            USB_RECIP_ENDPOINT,
                            1,
                            data,
                            8,   // Length of data.
                            500); // Timeout

  if (ret != 8) {
    ROS_INFO("Error: %s\n", usb_strerror());
    exit(EXIT_FAILURE);
  }

  usb_set_altinterface(launcher, 0);

    memset(data, 0, 64);

  if (!strcmp(cmd, "up")) {
    data[3] = 1;
  } else if (!strcmp(cmd, "down")) {
    data[4] = 1;
  } else if (!strcmp(cmd, "left")) {
    data[1] = 1;
  } else if (!strcmp(cmd, "right")) {
    data[2] = 1;
  } else if (!strcmp(cmd, "fire")) {
    data[5] = 1;
  } else if (!strcmp(cmd, "stop")) {
    memset(data, 0, 64);

    // exit(EXIT_FAILURE);
  }

  data[6] = 8;
  data[7] = 8;

  ret = usb_control_msg(launcher,
                        USB_DT_HID,
                        USB_REQ_SET_CONFIGURATION,
                        USB_RECIP_ENDPOINT,
                        1,
                        data,
                        64,  // Length of data.
                        500); // Timeout

  if (ret != 64) {
    ROS_INFO("Error: %s\n", usb_strerror());
    exit(EXIT_FAILURE);
  }

  ROS_INFO("FIN \n");
    usb_close(launcher);
  usb_release_interface(launcher, 1);

}
