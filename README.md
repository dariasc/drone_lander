# Instalación
Primero creamos la carpeta **RapiDron**, en donde creamos y construimos el espacio de trabajo para catkin

```bash
mkdir RapiDron && cd RapiDron
mkdir -p catkin_ws/src && cd catkin_ws 
catkin_make && cd src/
```
Luego, como nuestro proyecto depende de **Aruco**, clonamos esta dependencia ([aruco_ros](https://github.com/pal-robotics/aruco_ros)) y nuestro repositorio.

```bash
git clone https://github.com/pal-robotics/aruco_ros.git && cd aruco_ros
git checkout noetic-devel && cd ../
git clone https://github.com/dariasc/drone_lander.git
```
Por último, creamos la carpeta "include" necesaria para poder compilar el proyecto, y volvemos construir el espacio de trabajo.

```bash
mkdir -p drone_lander/lander/include && cd ../ && catkin_make
```

# Configuración
Cada vez que se quiera utilizar el paquete se tiene que agregar el espacio de trabajo al entorno

```bash
source ~/RapiDron/catkin_ws/devel/setup.bash
```

También puede ser conveniente agregar esta línea al archivo **.bashrc**, para así no tener que ejecutarla cada vez que se quiera ocupar el paquete. 

```bash
echo "source ~/RapiDron/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
