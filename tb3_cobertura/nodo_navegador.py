#!/usr/bin/env python3
"""Nodo navegador: monitoreo autónomo del escenario (percepción + decisión + control).

Suscribe:  /odom (nav_msgs/Odometry), /scan (sensor_msgs/LaserScan)
Publica:   /cmd_vel (geometry_msgs/TwistStamped; Twist si usar_twist_stamped=false)
           /cobertura/mapa_lidar (nav_msgs/OccupancyGrid)  -> RViz
           /cobertura/camino     (nav_msgs/Path)           -> RViz
           /cobertura/estado     (std_msgs/String)         -> estado de la máquina
           /mision/fin           (std_msgs/Bool, latched)  -> True al terminar

Toda la lógica vive en módulos sin ROS (mapa_ocupacion, planificador, mision),
los mismos que se validan en herramientas/sim2d.py.
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from .grilla_cobertura import Cobertura
from .fusion import FusionOdomImu
from .mapa_ocupacion import MapaOcupacion
from .mision import FIN, Mision, Parametros


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Navegador(Node):
    def __init__(self):
        super().__init__('navegador_cobertura')
        # --- parámetros: todos los de Parametros se exponen como parámetros ROS
        self.p = Parametros()
        for nombre in [a for a in dir(Parametros) if not a.startswith('_')]:
            valor = self.declare_parameter(nombre, getattr(Parametros, nombre)).value
            setattr(self.p, nombre, valor)
        self.usar_stamped = self.declare_parameter('usar_twist_stamped', True).value
        self.retardo = self.declare_parameter('retardo_inicio', 3.0).value
        self.frec = self.declare_parameter('frecuencia_control', 10.0).value
        self.usar_imu = self.declare_parameter('usar_imu', True).value
        self.fusion = FusionOdomImu()
        self._imu_orient_ok = False

        self.mapa = MapaOcupacion()
        self.cobertura = Cobertura()   # celdas propias visitadas (sin dato del escenario)
        self.mision = Mision(self.mapa, self.cobertura, self.p, log=self.get_logger().info)

        self.odoms = deque(maxlen=60)   # (t, x, y, th) para asociar cada scan a su pose
        self.w_odom = 0.0
        self.t_scan_ref = None
        self.sim_corriendo = False
        self.scan = None
        self.t_primer_scan = None
        self.fin_publicado = False

        tipo = TwistStamped if self.usar_stamped else Twist
        self.pub_cmd = self.create_publisher(tipo, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mapa = self.create_publisher(OccupancyGrid, '/cobertura/mapa_lidar', latched)
        self.pub_camino = self.create_publisher(Path, '/cobertura/camino', 10)
        self.pub_estado = self.create_publisher(String, '/cobertura/estado', 10)
        self.pub_fin = self.create_publisher(Bool, '/mision/fin', latched)
        self.pub_pose = self.create_publisher(Odometry, '/cobertura/pose', 20)
        # corrección de la odometría como transformación map -> odom (igual que
        # un nodo de localización): RViz dibuja todo, scan incluido, en la pose
        # corregida con la IMU y no en la de ruedas, que se va girando
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        if self.usar_imu:
            self.create_subscription(Imu, '/imu', self.cb_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / self.frec, self.control)
        self.create_timer(1.0, self.publicar_mapa)
        self.get_logger().info('Navegador listo: esperando /odom y /scan...')

    # ------------------------------------------------------------ callbacks
    def cb_imu(self, msg):
        q = msg.orientation
        norma = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norma > 0.5:            # la IMU publica orientación: se usa directo
            self._imu_orient_ok = True
            self.fusion.imu_orientacion(_yaw(q))
        elif not self._imu_orient_ok:  # si no, se integra el giróscopo
            self.fusion.imu_giroscopo(_t(msg.header.stamp), msg.angular_velocity.z)

    def cb_odom(self, msg):
        p = msg.pose.pose
        xo, yo, tho = p.position.x, p.position.y, _yaw(p.orientation)
        if self.usar_imu:
            x, y, th = self.fusion.odom(xo, yo, tho)
        else:
            x, y, th = xo, yo, tho
        t = _t(msg.header.stamp)
        self.odoms.append((t, x, y, th))
        self.w_odom = msg.twist.twist.angular.z
        # la navegación decide sobre la pose fusionada
        self.cobertura.actualizar(x, y)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = float(x)
        out.pose.pose.position.y = float(y)
        out.pose.pose.orientation.z = math.sin(th / 2.0)
        out.pose.pose.orientation.w = math.cos(th / 2.0)
        self.pub_pose.publish(out)
        self.publicar_correccion(msg.header.stamp, (xo, yo, tho), (x, y, th))

    def publicar_correccion(self, stamp, odom, corr):
        """TF map -> odom tal que (map -> odom) * (odom -> robot) = pose corregida."""
        xo, yo, tho = odom
        x, y, th = corr
        dth = th - tho
        c, s = math.cos(dth), math.sin(dth)
        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = 'map'
        tr.child_frame_id = 'odom'
        tr.transform.translation.x = float(x - (c * xo - s * yo))
        tr.transform.translation.y = float(y - (s * xo + c * yo))
        tr.transform.rotation.z = math.sin(dth / 2.0)
        tr.transform.rotation.w = math.cos(dth / 2.0)
        self.tf.sendTransform(tr)

    def _pose_en(self, t):
        """Pose de odometría interpolada al instante t (el del scan)."""
        od = list(self.odoms)
        if t <= od[0][0]:
            return od[0]
        if t >= od[-1][0]:
            return od[-1]
        for a, b in zip(od, od[1:]):
            if a[0] <= t <= b[0]:
                k = (t - a[0]) / max(b[0] - a[0], 1e-9)
                dth = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                return (t, a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]), a[3] + k * dth)
        return od[-1]

    def cb_scan(self, msg):
        if not self.odoms:
            return
        # La percepción empieza cuando la simulación empieza a correr. Con Gazebo
        # en pausa, el LiDAR puede publicar un barrido al inicializarse (o no,
        # según los tiempos de arranque); se descartan los barridos hasta que el
        # tiempo avance, para que el mapa arranque siempre vacío y se construya
        # a la vista durante el giro inicial.
        t_scan = _t(msg.header.stamp)
        if not self.sim_corriendo:
            if self.t_scan_ref is None:
                self.t_scan_ref = t_scan
                return
            # al inicializarse, Gazebo publica algún barrido con tiempos como
            # 0.000 y 0.002 s antes de quedar en pausa: se exige que el tiempo
            # avance medio segundo para considerar que la simulación corre
            if t_scan < self.t_scan_ref + 0.5:
                return
            self.sim_corriendo = True
        ts, x, y, th = self._pose_en(_t(msg.header.stamp))
        rangos = np.asarray(msg.ranges, dtype=np.float64)
        self.mapa.actualizar(x, y, th, rangos, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max, w=self.w_odom)
        # r >= range_min; lo que está por debajo se descarta (el simulador lo marca inf)
        r = np.where(rangos < msg.range_min, np.inf, rangos)
        self.scan = (r, msg.angle_min, msg.angle_increment)
        if self.t_primer_scan is None:
            self.t_primer_scan = self.get_clock().now()

    # ------------------------------------------------------------ control
    def control(self):
        if self.scan is None or not self.odoms or self.fin_publicado:
            return
        espera = (self.get_clock().now() - self.t_primer_scan).nanoseconds * 1e-9
        if espera < self.retardo:
            return
        t, x, y, th = self.odoms[-1]
        v, w = self.mision.paso(t, x, y, th, self.scan)
        self.enviar(v, w)

        est = String()
        est.data = f'{self.mision.estado} | {len(self.cobertura)} celdas | obj={self.mision.objetivo}'
        self.pub_estado.publish(est)
        if self.mision.wps:
            self.publicar_camino(x, y)

        if self.mision.estado == FIN:
            self.enviar(0.0, 0.0)
            self.get_logger().info(
                f'MISIÓN TERMINADA ({self.mision.motivo_fin}): '
                f'{len(self.cobertura)} celdas visitadas')
            self.pub_fin.publish(Bool(data=True))
            self.fin_publicado = True

    def enviar(self, v, w):
        if self.usar_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = float(w)
        else:
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    # ------------------------------------------------------------ visualización
    def publicar_camino(self, x, y):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for px, py in [(x, y)] + list(self.mision.wps):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_camino.publish(path)

    def publicar_mapa(self):
        m = self.mapa
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = float(m.res)
        g.info.width = m.n
        g.info.height = m.n
        g.info.origin.position.x = float(m.ox)
        g.info.origin.position.y = float(m.oy)
        g.info.origin.orientation.w = 1.0
        datos = np.full((m.n, m.n), -1, dtype=np.int8)
        datos[m.libre()] = 0
        datos[m.ocupado()] = 100
        # OccupancyGrid es fila-mayor en y: data[j * ancho + i]
        g.data = datos.T.flatten().tolist()
        self.pub_mapa.publish(g)

    def detener(self):
        try:
            self.enviar(0.0, 0.0)
        except Exception:  # noqa: BLE001  (el contexto puede estar cerrándose)
            pass


def main(args=None):
    rclpy.init(args=args)
    nodo = Navegador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.detener()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
