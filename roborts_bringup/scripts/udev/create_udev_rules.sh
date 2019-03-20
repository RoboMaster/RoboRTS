#!/bin/bash
echo "Remap the device port(ttyUSBX) to alias"
echo "USB connection for serial"

echo "Check them using the command : ls -l /dev|grep ttyUSB"
echo "Start to copy udev rules to  /etc/udev/rules.d/"

sudo cp `rospack find roborts_bringup`/scripts/udev/roborts.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Finish "
