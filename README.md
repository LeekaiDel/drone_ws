# drone_ws
it's essential ws for using mavros and PX4 with my custom pckgs

# base installs:
  <br>sudo apt install ros-noetic-tf2-sensor-msgs
  
  
# .bashrc confs:
  <br>echo "source ~/drone_ws/devel/setup.bash" >> ~/.bashrc
  <br>echo "source ~/drone_ws/drone_params.sh" >> ~/.bashrc
  <br>source ~/.bashrc
  <br>export ROS_MASTER_URI=http://192.168.1.254:11311/
  <br>export ROS_IP=192.168.1.254
  <br>export ROS_HOSTNAME=192.168.1.254

# Описание пакетов:
* [drone_essentials](src/drone_essentials): основной пакет для запуска всех необходимых для работы узлов
launchers:
bringup_gps_sim.launch: подключает позиционный регулятор и устанавливает tf дерево через robot_state_publisher
