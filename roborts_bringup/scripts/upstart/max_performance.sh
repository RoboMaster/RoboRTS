#/usr/bash
sleep 10
echo dji | sudo -S sudo  "/home/dji/jetson_clocks.sh"
echo "Max performance"
# disable wifi power saving

echo dji | sudo -S sudo iw dev wlan0 set power_save off
echo "Set power_save off "
