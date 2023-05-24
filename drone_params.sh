#!/bin/bash

# $ Конфиг-шаблоны $ #
# MAVROS_FCU_URL='/dev/ttyACM0:115200'  для общения с полетником через сериал порт
# MAVROS_FCU_URL='udp://:14540@localhost:14557'   для общения с полетным контроллером через udp port на виртуальной машине

# Id БЛА в группе, для каждого отдельного БЛА топики помечаются преффиксом /uav_<DRONE_ID>
export DRONE_ID=1

# Конфигурационные параметры для mavros
# Адрес полетного контроллера
export MAVROS_FCU_URL='udp://:14540@localhost:14557' # 
# Порт полетного контроллера
export MAVROS_FCU_PORT=14101
# Ip-адрес компьтера контролирующего полетный контроллер
export MAVROS_GCS_IP='localhost'

# Параметры ROS2
export ROS_DOMAIN_ID=137
source ~/Workspaces/drone_ws/install/setup.bash