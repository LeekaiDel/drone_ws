#!/bin/bash
export DRONE_ID=0

# Конфигурационные параметры для mavros
export MAVROS_GCS_IP=192.168.128.253
export MAVROS_FCU_URL='/dev/ttyACM0:115200'
export MAVROS_FCU_PORT=14101

# Флаги настройки системы:
export DRONE_REG_USE_POTENTIAL_PLANNER=False    # Использовать потенциальные поля?
export DRONE_REG_USE_UNSTABLE_PLANNER=False     # Использовать неустойчивый планировщик?
export DRONE_REG_USE_GEO_MODE=False             # Использовать расчет позиции в нодах из координат GPS?          
export USE_D435=False                           # Использовать камеру d435?

export ROS_DOMAIN_ID=137


