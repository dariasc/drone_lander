# Instalación
```bash
mkdir RapiDron && cd RapiDron
mkdir catkin_ws && cd catkin_ws 
catkin_make ./ && cd src/
git clone https://github.com/pal-robotics/aruco_ros.git
git clone https://github.com/dariasc/drone_lander.git
mkdir -p src/drone_lander/lander/include && cd ../ && catkin_make
```
