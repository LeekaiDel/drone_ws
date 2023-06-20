# drone_ws
>it's essential ws for using mavros and PX4 with my custom pckgs

### base installs:
```
sudo apt install ros-noetic-tf2-sensor-msgs
```
  
### .bashrc confs:
```
echo "source ~/drone_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/drone_ws/drone_params.sh" >> ~/.bashrc
source ~/.bashrc
```

### Install mavros
```
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```

### Install geodetic lib

```
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```
  
### extent to .bashrc:
```
export ROS_MASTER_URI=http://192.168.1.254:11311/
export ROS_IP=192.168.1.254
export ROS_HOSTNAME=192.168.1.254
```

### Описание пакетов:
* [drone_essentials](src/drone_essentials): основной пакет для запуска всех необходимых для работы узлов

  launchers:
      
    - **bringup_gps.launch**: подключает mavros и позиционный регулятор, устанавливает tf дерево через robot_state_publisher
    - **bringup_gps_sim.launch**: подключает позиционный регулятор, устанавливает tf дерево через robot_state_publisher (mavros не запускается, это делает симулятор)
    - **bringup_realsense.launch**: запускает mavros с чтением одометрии из tf дерева от realsense, устанавливает tf дерево через robot_state_publisher
    - **cartographer_lidar_odom_gek.launch**: запускает google cartographer с подключенной одометрией из mavros
    - **realsense_d435i_t265.launch**: запускает узлы для камер t265 и d435
    - **reg.launch**: запускает позиционный регулятор, а так же планировщики на потенциальных полях
* [mavros_link](src/mavros_link): пакет хранящий лончи и конфиги запуска mavros под разные задачи
    
    launchers:
      
    - **px4_gps.launch**: запуск mavros для конфигурации с gps
    - **px4_realsense.launch**: запуск mavros для конфигурации принятия навигации от камеры t265
    - **px4_slam.launch**: запуск mavros для конфигурации принятия навигации от slam алгоритма
* [drone_reg](src/drone_reg): пакет позиционного регулятора на основе PD регуляторов

