#!/bin/bash
echo "Remap the device port(ttyUSBX) to alias"
echo "USB connection for"
echo " 1. rplidar"
echo " 2. usb serial"
echo " 3. industrial camera"
echo " 4. usb camera"
echo "Check them using the command : ls -l /dev|grep ttyUSB"
echo "Start to copy udev rules to  /etc/udev/rules.d/"

sudo cp `rospack find roborts`/tools/scripts/udev/roborts.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Finish "
