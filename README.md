# Memoria de Título

## Setup Robot Baxter
Las siguientes instrucciones deben realizarse en un sistema **Ubuntu 14.04** (Trusty Tahr). No garantizo de que funcionen en otra plataforma.

Setear sources.list:

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
```

Setear keys:
```bash
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

Verificar paquetes:
```bash
$ sudo apt-get update
```

Instalar ROS Indigo Desktop Full:
```bash
$ sudo apt-get install ros-indigo-desktop-full
```

Inicializar rosdep:
```bash
$ sudo rosdep init
$ rosdep update```

Instalar rosinstall:
```bash
$ sudo apt-get install python-rosinstall
```

Crear Workspace de ROS:
```bash
$ mkdir -p ~/ros_ws/src
```

Sourcear ROS y construir:
```bash
$ source /opt/ros/indigo/setup.bash
```

Construir e instalar:
```bash
$ cd ~/ros_ws
$ catkin_make
$ catkin_make install```


Install dependencias del Baxter SDK:
```bash
$ sudo apt-get update
$ sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers
```

Instalar SDK de Baxter:
```bash
$ cd ~/ros_ws/src
$ wstool init .
$ wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall`
$ wstool update
```

Sourcear ROS Setup:
```bash
$ source /opt/ros/indigo/setup.bash
```

Construir e instalar:
```bash
$ cd ~/ros_ws
$ catkin_make
$ catkin_make install
```
