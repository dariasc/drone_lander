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

# Consideraciones importantes 

El PID del dron fue modificado y los cambios fueros realizados en *~/catkin_ws/src/pidrone_pkg/scripts/pid_class.py*

```python
class PID:

    def __init__(self,

         roll=PIDaxis(5.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
         roll_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

         pitch=PIDaxis(4.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
         pitch_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

         yaw=PIDaxis(0.0, 0.0, 0.0),

         # Kv 2300 motors have midpoint 1300, Kv 2550 motors have midpoint 1250
         throttle=PIDaxis(2,
                          0.8, #0.5/height_factor * battery_factor,
                          1.3,
                          i_range=(-400, 400), control_range=(1200, 1375),
                          d_range=(-40, 40), midpoint=1350)
         ):
        
        .
        .
        .

    .
    .
    .

```
