# Instalación
```bash
mkdir RapiDron && cd RapiDron
mkdir -p catkin_ws/src && cd catkin_ws 
catkin_make && cd src/
git clone https://github.com/pal-robotics/aruco_ros.git && cd aruco_ros
git checkout noetic-devel && cd ../
git clone https://github.com/dariasc/drone_lander.git
mkdir -p drone_lander/lander/include && cd ../ && catkin_make
```
