# Guía de Instalación — Rover Lunar TMR 2026
## WSL2 + Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11

---

## PASO 1: Instalar WSL2 en Windows

Abre **PowerShell como Administrador** y ejecuta:

```powershell
wsl --install -d Ubuntu-22.04
```

Reinicia tu PC cuando te lo pida. Al reiniciar se abrirá Ubuntu automáticamente.
Crea tu usuario y contraseña de Ubuntu cuando te lo solicite.

Verifica que WSL2 esté activo:
```powershell
wsl --list --verbose
# Debe mostrar VERSION 2 junto a Ubuntu-22.04
```

---

## PASO 2: Configurar Ubuntu 22.04

Dentro de la terminal de Ubuntu:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release wget git build-essential
```

### Habilitar soporte de GUI (WSLg - ya viene en Windows 11)
```bash
# Verificar que funciona
echo $DISPLAY
# Debe mostrar algo como :0
```

---

## PASO 3: Instalar ROS 2 Humble

```bash
# Agregar repositorio ROS 2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Instalar ROS 2 Humble completo
sudo apt install -y ros-humble-desktop

# Instalar herramientas de desarrollo
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Inicializar rosdep
sudo rosdep init
rosdep update
```

Agregar ROS 2 al PATH (ejecutar UNA sola vez):
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verificar instalación:
```bash
ros2 --version
# Debe mostrar: ros2 cli version X.X.X
```

---

## PASO 4: Instalar Gazebo Classic 11

```bash
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher

# Verificar Gazebo
gazebo --version
# Debe mostrar: Gazebo multi-robot simulator, version 11.X.X
```

---

## PASO 5: Instalar paquetes para navegación y SLAM

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-teleop-twist-keyboard \
  ros-humble-twist-mux

pip3 install transforms3d numpy scipy
```

---

## PASO 6: Crear workspace y compilar el paquete del rover

```bash
# Crear workspace
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src

# Copiar el paquete lunar_rover_sim aquí
# (copia la carpeta lunar_rover_sim/ a ~/rover_ws/src/)

# Instalar dependencias
cd ~/rover_ws
rosdep install --from-paths src --ignore-src -r -y

# Compilar
colcon build --symlink-install

# Agregar al PATH
echo "source ~/rover_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## PASO 7: Lanzar la simulación

### Terminal 1 — Gazebo + Rover:
```bash
ros2 launch lunar_rover_sim full_simulation.launch.py
```

### Terminal 2 — Navegación (Nav2 + SLAM):
```bash
ros2 launch lunar_rover_sim navigation.launch.py
```

### Terminal 3 — Misión autónoma:
```bash
ros2 run lunar_rover_sim mission_manager.py
```

### Ver el mapa en RViz2:
```bash
ros2 launch lunar_rover_sim rviz.launch.py
```

---

## Solución de problemas comunes

### Gazebo no abre (problema de GUI en WSL2)
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_IP=127.0.0.1
gazebo  # probar primero solo
```

### Error "No such file or directory" al compilar
```bash
cd ~/rover_ws
colcon build --symlink-install --packages-select lunar_rover_sim
```

### RViz2 tarda en abrir
Normal en WSL2. Espera 30-60 segundos la primera vez.
