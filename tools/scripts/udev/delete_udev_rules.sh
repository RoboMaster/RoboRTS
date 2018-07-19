#!/bin/bash
echo "Delete remap the device port(ttyUSBX) to alias"
echo "USB connection for"
echo " 1. rplidar"
echo " 2. usb serial"
echo " 3. industrial camera"
echo " 4. usb camera"
echo "Delete udev in the /etc/udev/rules.d/"
sudo rm   /etc/udev/rules.d/roborts.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Finish"
