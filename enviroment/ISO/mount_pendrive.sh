#!/bin/bash

DEVICE_NAME=$1
LOG_FILE="/var/log/mount_pendrive.log"

mkdir -p /media/pendrive # Create mount point

echo "$(date) - Mounting /dev/${DEVICE_NAME} on /media/pendrive" >> $LOG_FILE
/usr/bin/mount -v /dev/${DEVICE_NAME} /media/pendrive >> $LOG_FILE 2>&1
if [ $? -eq 0 ]; then
    echo "$(date) - Mount successful" >> $LOG_FILE
    mkdir -p /root/pendrive_config
    cp /media/pendrive/* /root/pendrive_config

    umount /media/pendrive
    echo "Notifying ro2 service..."

    # Notify /pendrive_status service
    export HOME=/home/root/
    source /opt/ros/foxy/setup.bash
    ros2 service call /pendrive_status std_srvs/srv/SetBool "{data: true}"
else
    echo "$(date) - Mount failed with error code $?" >> $LOG_FILE
fi

