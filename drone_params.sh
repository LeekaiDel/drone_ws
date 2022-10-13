#!/bin/bash
export DRONE_ID=0

# Конфигурационные параметры для mavros
export MAVROS_GCS_IP=192.168.128.253
export MAVROS_FCU_URL='/dev/ttyS0:115200'
export MAVROS_FCU_PORT=14101

export ROS_DOMAIN_ID=137