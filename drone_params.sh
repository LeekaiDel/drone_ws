#!/bin/bash
source ~/drone_ws/devel/setup.bash

# export DRONE_MARKER_ID=1            #
# export DRONE_IP=192.168.1.253       #
# export DRONE_SAFE_MAX_ANGLE=85      #
# export DRONE_SAFE_DISARM_DELAY=0.2  #
export DRONE_ID=1

# Конфигурационные параметры для mavros
export MAVROS_GCS_IP=192.168.128.192
export MAVROS_FCU_URL='/dev/ttyACM0:921600'
export MAVROS_FCU_PORT=14101

# Флаги настройки системы:
export DRONE_REG_USE_POTENTIAL_PLANNER=False    # Использовать потенциальные поля?
export DRONE_REG_USE_UNSTABLE_PLANNER=False     # Использовать неустойчивый планировщик?
export DRONE_REG_USE_GEO_MODE=False             # Использовать расчет позиции в нодах из координат GPS?          
export USE_D435=False                           # Использовать камеру d435?

# Настройки для rosmaster 
#export ROS_MASTER_URI=http://localhost:11311/
#export ROS_IP=localhost
#export ROS_HOSTNAME=localhost

