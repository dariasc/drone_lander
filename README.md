# Instalación
```bash
mkdir RapiDron && cd RapiDron
mkdir -p catkin_ws/src/ && cd catkin_ws 
catkin_make && cd src/
git clone https://github.com/pal-robotics/aruco_ros.git
git clone https://github.com/dariasc/drone_lander.git
mkdir src/drone_lander/lander/include && catkin_make
```
