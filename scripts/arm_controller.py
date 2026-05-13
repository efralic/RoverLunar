#!/usr/bin/env python3
"""
arm_controller.py
Controlador del brazo robótico de 6 DOF para el Rover Lunar TMR 2026

Articulaciones:
  joint_1: base yaw         — MG996R  [−π,   π]
  joint_2: shoulder pitch   — MG996R  [−π/2, π/2]
  joint_3: elbow pitch      — MG996R  [−2π/3, 2π/3]
  joint_4: wrist pitch      — MG90S   [−π/2, π/2]
  joint_5: wrist roll       — MG90S   [−π,   π]
  joint_6: gripper open     — MG90S   [0,    π/3]

Longitudes de eslabón (metros):
  L1 = 0.095  (base)
  L2 = 0.115  (upper arm)
  L3 = 0.100  (forearm)
  L4 = 0.030  (wrist pitch)
  L5 = 0.040  (wrist roll)
  L6 = 0.030  (gripper)
"""

import rclpy
from rclpy.node import Node
import json
import math
import time
import numpy as np

from std_msgs.msg import String, Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── Longitudes de eslabón (metros) ────────────────────────────────
L1 = 0.095
L2 = 0.115
L3 = 0.100
L4 = 0.030
L5 = 0.040
L6 = 0.030

# ── Nombres de joints (deben coincidir con el URDF) ───────────────
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

# ── Posición HOME del brazo (posición plegada segura) ────────────
HOME_ANGLES = [0.0, -1.2, 1.0, 0.5, 0.0, 0.0]

# ── Posición de pick (preparada para bajar a recoger roca) ───────
PRE_PICK_ANGLES = [0.0, -0.5, 0.8, 0.3, 0.0, 0.33]  # gripper abierto

# ── Posición de deposit (sobre el contenedor) ────────────────────
PRE_DEPOSIT_ANGLES = [3.14, -0.8, 1.2, 0.3, 0.0, 0.0]


class ArmController(Node):

    def __init__(self):
        super().__init__('arm_controller')

        # Estado del brazo
        self.current_angles = list(HOME_ANGLES)
        self.action_in_progress = False
        self.rocks_held = []

        # ── Publishers ─────────────────────────────────────────
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory', 10)

        self.done_pub = self.create_publisher(Bool, '/arm/action_done', 10)

        # ── Subscribers ────────────────────────────────────────
        self.create_subscription(String, '/arm/command', self._on_command, 10)

        self.get_logger().info('🦾 Arm Controller iniciado')
        # Mover a HOME al arrancar
        self._move_to_angles(HOME_ANGLES, duration_sec=3.0)

    # ================================================================
    #  CALLBACK DE COMANDOS
    # ================================================================
    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Comando inválido: {msg.data}')
            return

        action = cmd.get('action', '')

        if action == 'pick':
            self._execute_pick(cmd)
        elif action == 'deposit_all':
            self._execute_deposit()
        elif action == 'home':
            self._move_to_angles(HOME_ANGLES, duration_sec=2.0)
        elif action == 'joints':
            angles = cmd.get('angles', HOME_ANGLES)
            self._move_to_angles(angles, duration_sec=cmd.get('duration', 2.0))
        else:
            self.get_logger().warn(f'Acción desconocida: {action}')

    # ================================================================
    #  SECUENCIA DE RECOLECCIÓN DE ROCA
    # ================================================================
    def _execute_pick(self, cmd: dict):
        rock_id = cmd.get('rock_id', '?')
        self.get_logger().info(f'🦾 Iniciando pick de {rock_id}...')
        self.action_in_progress = True

        # 1. Abrir gripper + ir a pre-pick
        self.get_logger().info('  → Abriendo gripper')
        pre = list(PRE_PICK_ANGLES)
        pre[5] = 0.33  # gripper abierto (~60°)
        self._move_to_angles(pre, duration_sec=1.5)
        time.sleep(1.6)

        # 2. Bajar a la roca (usando IK simple para posición en el suelo)
        self.get_logger().info('  → Bajando a la roca')
        pick_angles = self._compute_pick_ik()
        self._move_to_angles(pick_angles, duration_sec=1.5)
        time.sleep(1.6)

        # 3. Cerrar gripper
        self.get_logger().info('  → Cerrando gripper')
        closed = list(pick_angles)
        closed[5] = 0.0  # cerrado
        self._move_to_angles(closed, duration_sec=0.8)
        time.sleep(0.9)

        # 4. Levantar la roca
        self.get_logger().info('  → Levantando roca')
        lift = list(HOME_ANGLES)
        lift[5] = 0.0  # gripper cerrado sosteniendo la roca
        self._move_to_angles(lift, duration_sec=1.5)
        time.sleep(1.6)

        self.rocks_held.append(rock_id)
        self.action_in_progress = False
        self.done_pub.publish(Bool(data=True))
        self.get_logger().info(f'✅ Roca {rock_id} recogida ({len(self.rocks_held)} en brazo)')

    # ================================================================
    #  SECUENCIA DE DEPÓSITO
    # ================================================================
    def _execute_deposit(self):
        self.get_logger().info(f'📦 Depositando {len(self.rocks_held)} rocas...')
        self.action_in_progress = True

        # 1. Girar hacia el contenedor (atrás del rover, yaw ~180°)
        self._move_to_angles(PRE_DEPOSIT_ANGLES, duration_sec=2.0)
        time.sleep(2.1)

        # 2. Extender brazo sobre el contenedor
        deposit = list(PRE_DEPOSIT_ANGLES)
        deposit[1] = -0.3  # bajar un poco el hombro
        self._move_to_angles(deposit, duration_sec=1.5)
        time.sleep(1.6)

        # 3. Abrir gripper → las rocas caen al contenedor
        deposit[5] = 0.33  # abrir
        self._move_to_angles(deposit, duration_sec=0.8)
        time.sleep(0.9)
        self.get_logger().info(f'📦 {len(self.rocks_held)} rocas depositadas!')
        self.rocks_held.clear()

        # 4. Volver a HOME
        self._move_to_angles(HOME_ANGLES, duration_sec=2.0)
        time.sleep(2.1)

        self.action_in_progress = False
        self.done_pub.publish(Bool(data=True))

    # ================================================================
    #  CINEMÁTICA INVERSA SIMPLIFICADA PARA PICK
    #  Objetivo: TCP en el suelo, frente al rover
    #  Asume roca a ~15cm del frente del rover y a z=0
    # ================================================================
    def _compute_pick_ik(self) -> list:
        """
        IK analítica de 3R planar (joints 2, 3, 4) para alcanzar
        una posición en el suelo frente al rover.

        Target (en frame del brazo):
          x_t = 0.15  (15cm hacia adelante)
          z_t = -(L1) (nivel del suelo)
        """
        # Offset del TCP respecto al base del brazo
        x_t = 0.18   # distancia horizontal al objetivo
        z_t = -0.05  # altura del objetivo (sobre el suelo, no el suelo exacto)

        # Distancia al objetivo (radio planar desde base del brazo)
        r = math.sqrt(x_t**2 + z_t**2)

        # Ley de cosenos para joint_3 (codo)
        reach2 = L2 + L3
        if r > reach2:
            r = reach2 * 0.95  # no exceder alcance

        cos_q3 = (r**2 - L2**2 - L3**2) / (2 * L2 * L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3 = -math.acos(cos_q3)  # codo abajo

        # Joint 2 (hombro)
        beta  = math.atan2(z_t, x_t)
        alpha = math.atan2(L3 * math.sin(-q3),
                           L2 + L3 * math.cos(-q3))
        q2 = beta - alpha

        # Joint 4 (muñeca horizontal al suelo)
        q4 = -(q2 + q3)

        return [
            0.0,   # joint_1: sin rotación base
            q2,    # joint_2: hombro
            q3,    # joint_3: codo
            q4,    # joint_4: muñeca
            0.0,   # joint_5: roll neutro
            0.33,  # joint_6: gripper abierto
        ]

    # ================================================================
    #  PUBLICACIÓN DE TRAYECTORIA
    # ================================================================
    def _move_to_angles(self, angles: list, duration_sec: float = 2.0):
        """Publica una trayectoria de un punto al joint trajectory controller."""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in angles]
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )

        traj.points = [point]
        self.traj_pub.publish(traj)
        self.current_angles = list(angles)

        self.get_logger().debug(
            f'Brazo → {[f"{math.degrees(a):.1f}°" for a in angles]}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
