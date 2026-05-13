#!/bin/bash
# ============================================================
# start_mission.sh — Lanzador rápido del Rover Lunar TMR 2026
# Uso: bash start_mission.sh
# ============================================================

set -e

PKG="lunar_rover_sim"
WS="$HOME/rover_ws"

echo ""
echo "🌙 ============================================="
echo "   ROVER LUNAR TMR 2026 — Simulación Gazebo"
echo "==============================================="
echo ""

# Verificar que el workspace esté compilado
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "❌ Workspace no compilado. Ejecuta primero:"
  echo "   cd ~/rover_ws && colcon build --symlink-install"
  exit 1
fi

source "$WS/install/setup.bash"
source /opt/ros/humble/setup.bash

echo "✅ Workspace cargado"
echo ""
echo "Abriendo terminales..."
echo ""

# Función para abrir terminal nueva con comando
open_terminal() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="$title" -- bash -c "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $cmd; exec bash" 2>/dev/null ||
  xterm -title "$title" -e "bash -c 'source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $cmd; exec bash'" 2>/dev/null ||
  echo "Ejecuta manualmente: $cmd"
}

echo "1️⃣  Lanzando Gazebo + Rover..."
open_terminal "Gazebo + Rover" \
  "ros2 launch $PKG full_simulation.launch.py"

sleep 5

echo "2️⃣  Lanzando Navegación (SLAM + Nav2)..."
open_terminal "Navegación" \
  "ros2 launch $PKG navigation.launch.py"

sleep 5

echo "3️⃣  Lanzando Rock Detector..."
open_terminal "Rock Detector" \
  "ros2 run $PKG rock_detector.py"

sleep 2

echo "4️⃣  Lanzando Arm Controller..."
open_terminal "Arm Controller" \
  "ros2 run $PKG arm_controller.py"

sleep 2

echo ""
echo "🚀 Todo listo! Para INICIAR LA MISIÓN ejecuta:"
echo ""
echo "   ros2 topic pub --once /mission/cmd std_msgs/String '{data: \"START\"}'"
echo ""
echo "📊 Para ver el estado de la misión:"
echo "   ros2 topic echo /mission/status"
echo ""
echo "🗺️  Para ver el mapa en RViz2:"
echo "   ros2 launch $PKG full_simulation.launch.py"
echo ""
echo "🎮 Para control manual (teleop):"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "   (Remap: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "    --ros-args --remap cmd_vel:=/rover/cmd_vel)"
echo ""
echo "⛔ Para DETENER la misión:"
echo "   ros2 topic pub --once /mission/cmd std_msgs/String '{data: \"STOP\"}'"
echo ""
