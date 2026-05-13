#!/usr/bin/env python3
"""
mission_manager.py
Máquina de estados de la misión completa del Rover Lunar TMR 2026

Estados:
  1. EXPLORE_AND_MAP  → Recorrer el área, detectar rocas, construir mapa
  2. NAVIGATE_TO_FIN  → Navegar autónomamente al letrero FIN
  3. CONFIRM_FIN      → Confirmar localización del letrero FIN
  4. RETURN_TO_START  → Regresar al punto INICIO
  5. COLLECT_ROCKS    → Recoger rocas detectadas (con brazo)
  6. DEPOSIT_ROCKS    → Depositar en contenedor
  7. MISSION_COMPLETE → Fin de misión

Puntaje objetivo: 535 pts máximo (sin tablero)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum, auto
import json
import math

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration


class MissionState(Enum):
    IDLE            = auto()
    EXPLORE_AND_MAP = auto()
    NAVIGATE_TO_FIN = auto()
    CONFIRM_FIN     = auto()
    RETURN_TO_START = auto()
    COLLECT_ROCKS   = auto()
    DEPOSIT_ROCKS   = auto()
    MISSION_COMPLETE = auto()
    ERROR           = auto()


class MissionManager(Node):
    """Nodo principal de gestión de misión autónoma."""

    # ── Coordenadas clave en el mapa (metros) ───────────────────
    START_POS   = (1.0, 5.0)   # Zona INICIO / contenedor
    FIN_POS     = (9.0, 5.0)   # Letrero FIN

    # Waypoints de exploración — patrón serpentina 10x10m
    EXPLORE_WAYPOINTS = [
        (2.0, 2.0), (4.0, 2.0), (6.0, 2.0), (8.5, 2.0),
        (8.5, 4.0), (6.0, 4.0), (4.0, 4.0), (2.0, 4.0),
        (2.0, 6.0), (4.0, 6.0), (6.0, 6.0), (8.5, 6.0),
        (8.5, 8.0), (6.0, 8.0), (4.0, 8.0), (2.0, 8.0),
        (9.0, 5.0),  # ← letrero FIN al final de la exploración
    ]

    def __init__(self):
        super().__init__('mission_manager')

        # ── Estado de la misión ──────────────────────────────────
        self.state = MissionState.IDLE
        self.prev_state = None

        # ── Inventario de rocas detectadas ───────────────────────
        # Cada roca: {'id': str, 'color': str, 'x': float, 'y': float,
        #             'size_cm3': float, 'collected': bool}
        self.rock_inventory = []
        self.rocks_collected = 0
        self.fin_confirmed = False

        # ── Índice de waypoints de exploración ───────────────────
        self.waypoint_index = 0

        # ── Score tracker ────────────────────────────────────────
        self.score = {
            'navegacion': 0,
            'exploracion': 0,
            'recoleccion': 0,
            'total': 0
        }

        # ── Action client Nav2 ───────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None
        self._nav_in_progress = False

        # ── Publishers ───────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.status_pub  = self.create_publisher(String, '/mission/status', 10)
        self.score_pub   = self.create_publisher(String, '/mission/score', 10)
        self.marker_pub  = self.create_publisher(MarkerArray, '/mission/rock_markers', 10)
        self.arm_cmd_pub = self.create_publisher(String, '/arm/command', 10)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(String, '/rock_detector/detections',
                                 self._on_rock_detected, 10)
        self.create_subscription(Bool, '/rock_detector/fin_detected',
                                 self._on_fin_detected, 10)
        self.create_subscription(Bool, '/arm/action_done',
                                 self._on_arm_done, 10)

        # ── Timer principal (máquina de estados) ─────────────────
        self.create_timer(0.5, self._state_machine_tick)

        # ── Timer publicación de estado ───────────────────────────
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('🌙 Mission Manager iniciado — esperando comando START')
        self.get_logger().info('   Publica "START" en /mission/cmd para comenzar')

        # Suscripción a comando de inicio
        self.create_subscription(String, '/mission/cmd', self._on_cmd, 10)

    # ================================================================
    #  MÁQUINA DE ESTADOS PRINCIPAL
    # ================================================================
    def _state_machine_tick(self):
        if self.state == MissionState.IDLE:
            pass  # Esperar comando START

        elif self.state == MissionState.EXPLORE_AND_MAP:
            self._handle_explore()

        elif self.state == MissionState.NAVIGATE_TO_FIN:
            self._handle_navigate_to_fin()

        elif self.state == MissionState.CONFIRM_FIN:
            self._handle_confirm_fin()

        elif self.state == MissionState.RETURN_TO_START:
            self._handle_return_to_start()

        elif self.state == MissionState.COLLECT_ROCKS:
            self._handle_collect_rocks()

        elif self.state == MissionState.DEPOSIT_ROCKS:
            self._handle_deposit()

        elif self.state == MissionState.MISSION_COMPLETE:
            self._handle_mission_complete()

    # ================================================================
    #  ESTADO 1: EXPLORACIÓN Y MAPEO
    # ================================================================
    def _handle_explore(self):
        if self._nav_in_progress:
            return  # Esperar a que termine el movimiento actual

        if self.waypoint_index < len(self.EXPLORE_WAYPOINTS) - 1:
            # Navegar al siguiente waypoint de exploración
            wp = self.EXPLORE_WAYPOINTS[self.waypoint_index]
            self.get_logger().info(
                f'🗺️  Exploración waypoint {self.waypoint_index + 1}/'
                f'{len(self.EXPLORE_WAYPOINTS) - 1}: ({wp[0]:.1f}, {wp[1]:.1f})')
            self._navigate_to(wp[0], wp[1])
            self.waypoint_index += 1
            # Sumar puntos de terreno al pasar
            self._check_terrain_waypoint(wp)

        else:
            # Exploración completa → ir al letrero FIN
            self.get_logger().info('✅ Exploración completa. Rocas detectadas: '
                                   f'{len(self.rock_inventory)}')
            self._publish_rock_markers()
            self._transition_to(MissionState.NAVIGATE_TO_FIN)

    def _check_terrain_waypoint(self, wp):
        """Suma puntos de navegación al pasar por zonas de terreno."""
        x, y = wp
        # Valles (aprox.)
        valley_zones = [(4,3), (5,6.5), (7,3.5)]
        groove_zones  = [(3,7), (5.5,5), (7.5,7)]
        slope_zones   = [(3,2), (7,8)]

        for vx, vy in valley_zones:
            if math.dist((x,y), (vx,vy)) < 1.0 and self.score['navegacion'] < 60:
                self.score['navegacion'] += 10
                self.get_logger().info(f'🏔️  Valle cruzado +10 pts')

        for gx, gy in groove_zones:
            if math.dist((x,y), (gx,gy)) < 1.0 and self.score['navegacion'] < 90:
                self.score['navegacion'] += 10
                self.get_logger().info(f'〰️  Surco cruzado +10 pts')

        for sx, sy in slope_zones:
            if math.dist((x,y), (sx,sy)) < 1.0 and self.score['navegacion'] < 115:
                self.score['navegacion'] += 10
                self.get_logger().info(f'📐 Pendiente cruzada +10 pts')

    # ================================================================
    #  ESTADO 2: NAVEGAR AL LETRERO FIN
    # ================================================================
    def _handle_navigate_to_fin(self):
        if self._nav_in_progress:
            return
        self.get_logger().info('🎯 Navegando hacia letrero FIN...')
        self._navigate_to(self.FIN_POS[0], self.FIN_POS[1])
        self._transition_to(MissionState.CONFIRM_FIN)

    # ================================================================
    #  ESTADO 3: CONFIRMAR LETRERO FIN
    # ================================================================
    def _handle_confirm_fin(self):
        if self._nav_in_progress:
            return
        if self.fin_confirmed:
            self.get_logger().info('🏁 Letrero FIN confirmado! +10 pts')
            self.score['navegacion'] += 10
            # Girar para señalar que lo encontró
            self._spin_in_place(duration=2.0)
            self._transition_to(MissionState.RETURN_TO_START)
        else:
            # Buscar girando
            self.get_logger().info('🔍 Buscando letrero FIN...')
            self._spin_slow()

    # ================================================================
    #  ESTADO 4: RETORNO AL INICIO
    # ================================================================
    def _handle_return_to_start(self):
        if self._nav_in_progress:
            return
        self.get_logger().info('🏠 Regresando al INICIO...')
        self._navigate_to(self.START_POS[0], self.START_POS[1])
        self.score['navegacion'] += 25  # Regreso INICIO: 25 pts
        self.get_logger().info('🏠 Regreso al INICIO: +25 pts')
        self._transition_to(MissionState.COLLECT_ROCKS)

    # ================================================================
    #  ESTADO 5: RECOLECCIÓN DE ROCAS
    # ================================================================
    def _handle_collect_rocks(self):
        if self._nav_in_progress:
            return

        uncollected = [r for r in self.rock_inventory if not r['collected']]

        if not uncollected:
            self.get_logger().info(
                f'✅ Recolección completa. {self.rocks_collected} rocas recogidas.')
            self._transition_to(MissionState.DEPOSIT_ROCKS)
            return

        # Ir a la roca más cercana al INICIO (optimización de ruta)
        rock = min(uncollected,
                   key=lambda r: math.dist(
                       (r['x'], r['y']), self.START_POS))

        self.get_logger().info(
            f'🪨 Yendo a roca {rock["id"]} ({rock["color"]}) '
            f'en ({rock["x"]:.2f}, {rock["y"]:.2f})')
        self._navigate_to(rock['x'], rock['y'])

        # Enviar comando al brazo para recoger
        arm_cmd = json.dumps({
            'action': 'pick',
            'rock_id': rock['id'],
            'color': rock['color'],
            'size': rock['size_cm3']
        })
        self.arm_cmd_pub.publish(String(data=arm_cmd))

        rock['collected'] = True
        self.rocks_collected += 1

        # Puntos por roca transportada
        self.score['recoleccion'] += 10
        self.get_logger().info(
            f'✅ Roca {rock["id"]} recolectada +10 pts '
            f'({self.rocks_collected}/10)')

    # ================================================================
    #  ESTADO 6: DEPOSITAR ROCAS
    # ================================================================
    def _handle_deposit(self):
        if self._nav_in_progress:
            return

        # Navegar al contenedor (junto al punto INICIO)
        container_pos = (self.START_POS[0], self.START_POS[1] - 0.8)
        self.get_logger().info('📦 Depositando rocas en el contenedor...')
        self._navigate_to(container_pos[0], container_pos[1])

        # Comando al brazo para vaciar las rocas
        arm_cmd = json.dumps({'action': 'deposit_all'})
        self.arm_cmd_pub.publish(String(data=arm_cmd))

        # Puntos por rocas depositadas
        deposit_pts = self.rocks_collected * 10
        self.score['recoleccion'] += deposit_pts
        self.get_logger().info(
            f'📦 {self.rocks_collected} rocas depositadas +{deposit_pts} pts')

        self._transition_to(MissionState.MISSION_COMPLETE)

    # ================================================================
    #  ESTADO 7: MISIÓN COMPLETA
    # ================================================================
    def _handle_mission_complete(self):
        self.score['total'] = sum([
            self.score['navegacion'],
            self.score['exploracion'],
            self.score['recoleccion']
        ])

        self.get_logger().info('=' * 50)
        self.get_logger().info('🏆 MISIÓN COMPLETA — ROVER LUNAR TMR 2026')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Navegación:   {self.score["navegacion"]} pts')
        self.get_logger().info(f'  Exploración:  {self.score["exploracion"]} pts')
        self.get_logger().info(f'  Recolección:  {self.score["recoleccion"]} pts')
        self.get_logger().info(f'  TOTAL:        {self.score["total"]} / 535 pts')
        self.get_logger().info('=' * 50)

        # Publicar score final
        self._publish_status()
        # Detener motores
        self.cmd_vel_pub.publish(Twist())

        # Cambiar a idle para no seguir procesando
        self.state = MissionState.IDLE

    # ================================================================
    #  CALLBACKS
    # ================================================================
    def _on_cmd(self, msg: String):
        if msg.data.upper() == 'START' and self.state == MissionState.IDLE:
            self.get_logger().info('🚀 Iniciando misión autónoma...')
            self._transition_to(MissionState.EXPLORE_AND_MAP)

        elif msg.data.upper() == 'STOP':
            self.get_logger().warn('⛔ Misión detenida manualmente')
            self.cmd_vel_pub.publish(Twist())
            self.state = MissionState.IDLE

    def _on_rock_detected(self, msg: String):
        """Recibe detecciones del nodo rock_detector."""
        try:
            data = json.loads(msg.data)
            rock_id = f"rock_{len(self.rock_inventory) + 1}"

            # Verificar si ya está registrada (por proximidad)
            for r in self.rock_inventory:
                if math.dist((r['x'], r['y']), (data['x'], data['y'])) < 0.15:
                    return  # Ya registrada

            rock = {
                'id': rock_id,
                'color': data.get('color', 'unknown'),
                'x': data['x'],
                'y': data['y'],
                'size_cm3': data.get('size_cm3', 7.0),
                'weight_g': data.get('weight_g', 30.0),
                'collected': False
            }
            self.rock_inventory.append(rock)

            # Puntos de exploración por información de roca
            self.score['exploracion'] += 10
            self.get_logger().info(
                f'🪨 Roca detectada: {rock_id} | color={rock["color"]} | '
                f'pos=({rock["x"]:.2f},{rock["y"]:.2f}) | '
                f'+10 pts exploración (total rocas: {len(self.rock_inventory)})')

            # Bono por mapa con ≥3 objetos
            if len(self.rock_inventory) == 3:
                self.score['exploracion'] += 30
                self.get_logger().info('🗺️  Mapa con ≥3 objetos georreferenciados: +30 pts!')

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Error parseando detección: {e}')

    def _on_fin_detected(self, msg: Bool):
        if msg.data and not self.fin_confirmed:
            self.fin_confirmed = True
            self.get_logger().info('🏁 Letrero FIN detectado por cámara!')

    def _on_arm_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info('🦾 Brazo: acción completada')

    # ================================================================
    #  NAVEGACIÓN (Nav2 Action Client)
    # ================================================================
    def _navigate_to(self, x: float, y: float, yaw: float = 0.0):
        """Envía goal de navegación a Nav2."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 no disponible!')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # Orientación en quaternion desde yaw
        import math
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._nav_in_progress = True
        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback
        )
        future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal de navegación rechazado')
            self._nav_in_progress = False
            return
        self._nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        self._nav_in_progress = False
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✅ Navegación completada')
        else:
            self.get_logger().warn(f'⚠️  Navegación terminó con status: {status}')

    def _nav_feedback(self, feedback_msg):
        pass  # Podría usarse para monitoreo en tiempo real

    # ================================================================
    #  UTILIDADES
    # ================================================================
    def _transition_to(self, new_state: MissionState):
        self.get_logger().info(
            f'📍 Estado: {self.state.name} → {new_state.name}')
        self.prev_state = self.state
        self.state = new_state
        self._nav_in_progress = False

    def _spin_in_place(self, duration: float = 2.0):
        """Girar en el lugar para señalar localización de FIN."""
        twist = Twist()
        twist.angular.z = 0.8
        end_time = self.get_clock().now().nanoseconds + int(duration * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            self.cmd_vel_pub.publish(twist)
        self.cmd_vel_pub.publish(Twist())

    def _spin_slow(self):
        """Girar lentamente buscando el letrero FIN."""
        twist = Twist()
        twist.angular.z = 0.3
        self.cmd_vel_pub.publish(twist)

    def _publish_rock_markers(self):
        """Publica marcadores de rocas en RViz."""
        marker_array = MarkerArray()
        for i, rock in enumerate(self.rock_inventory):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'rocks'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = rock['x']
            marker.pose.position.y = rock['y']
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            scale = 0.08
            marker.scale.x = marker.scale.y = marker.scale.z = scale
            color_map = {
                'rojo':  (1.0, 0.0, 0.0, 1.0),
                'azul':  (0.0, 0.0, 1.0, 1.0),
                'verde': (0.0, 1.0, 0.0, 1.0),
            }
            r, g, b, a = color_map.get(rock['color'], (1.0, 1.0, 0.0, 1.0))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def _publish_status(self):
        status = {
            'state': self.state.name,
            'rocks_detected': len(self.rock_inventory),
            'rocks_collected': self.rocks_collected,
            'fin_confirmed': self.fin_confirmed,
            'score': self.score,
            'waypoint': self.waypoint_index
        }
        self.status_pub.publish(String(data=json.dumps(status, indent=2)))
        self.score_pub.publish(String(data=json.dumps(self.score)))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission Manager detenido por usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
