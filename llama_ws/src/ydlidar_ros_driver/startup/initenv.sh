#!/bin/bash

echo  'KERNEL=="ttyS4", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules

service udev reload
sleep 2
service udev restart

