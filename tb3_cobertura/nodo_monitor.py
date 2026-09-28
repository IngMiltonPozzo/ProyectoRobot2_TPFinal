#!/usr/bin/env python3
"""Nodo monitor: el "juez" de la misión.

El robot no conoce el escenario; el juez sí, porque para medir qué porcentaje
se monitoreó hace falta saber cuánto había para monitorear. Al arrancar, lee el
archivo .world de Gazebo y calcula las celdas de 25 cm RECORRIBLES (donde el
centro del robot puede estar sin chocar y a las que se puede llegar). Ese es
el denominador. El numerador son las celdas recorribles por las que pasó el
robot, según tres fuentes de posición:

  - /verdad/poses   : posición verdadera de Gazebo  -> resultado de la misión
  - /cobertura/pose : pose estimada por el robot (ruedas + IMU)
  - /odom           : odometría de ruedas cruda

Ninguno de estos datos vuelve al robot: la navegación no se suscribe a nada
de lo que publica el monitor.
"""
import csv
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray

from . import escenario_verdad as ev
from .grilla_cobertura import rectangulo


class Monitor(Node):
    def __init__(self):
        super().__init__('monitor_cobertura')
        mundo = self.declare_parameter('archivo_mundo', '').value
        rutas = self.declare_parameter('rutas_modelos', ['']).value
        radio = self.declare_parameter('radio_robot', 0.13).value
        cilindros = self.declare_parameter('incluir_cilindros', True).value
        obst = ev.leer_mundo(mundo, [r for r in rutas if r], incluir_cilindros=cilindros)
        self.recorribles = ev.celdas_recorribles(obst, radio_robot=radio)
        self.get_logger().info(
            f'Escenario de evaluación: {len(obst.cajas)} paredes, {len(obst.cilindros)} cilindros, '
            f'{len(self.recorribles)} celdas recorribles de 25 cm (radio del robot {radio} m)')
        # matriz para mostrar en la terminal: filas de arriba (y mayor) hacia abajo,
        # columnas de izquierda (x menor) a derecha, igual que la vista superior
        self.imprimir_matriz = self.declare_parameter('imprimir_matriz', True).value
        # la matriz también se escribe en un archivo, para verla actualizarse
        # "en el lugar" desde otra terminal:  watch -t -n 0.5 cat <archivo>
        self.archivo_matriz = self.declare_parameter('archivo_matriz', '/tmp/tb3_matriz.txt').value
        ii = [c[0] for c in self.recorribles]
        jj = [c[1] for c in self.recorribles]
        self._rango_i = range(min(ii) - 1, max(ii) + 2)
        self._rango_j = range(max(jj) + 1, min(jj) - 2, -1)
        self.metrica = ev.MetricaReal(self.recorribles)      # sobre /odom
        self.metrica_fus = ev.MetricaReal(self.recorribles)  # sobre la pose estimada
        self.metrica_ver = ev.MetricaReal(self.recorribles)  # sobre la posición verdadera
        self.hay_fus = False
        self.hay_ver = False
        self.escribir_matriz(self.metrica_ver, 0.0)
        carpeta = self.declare_parameter('carpeta_log', os.path.expanduser('~/cobertura_logs')).value
        try:
            os.makedirs(carpeta, exist_ok=True)
            open(os.path.join(carpeta, '.prueba'), 'w').close()
        except OSError:
            self.get_logger().warn(f'No se puede escribir en {carpeta}; uso ~/cobertura_logs')
            carpeta = os.path.expanduser('~/cobertura_logs')
            os.makedirs(carpeta, exist_ok=True)
        nombre = time.strftime('cobertura_%Y%m%d_%H%M%S.csv')
        self.ruta_csv = os.path.join(carpeta, nombre)
        self.csv = open(self.ruta_csv, 'w', newline='')
        self.w = csv.writer(self.csv)
        self.w.writerow(['t_sim', 'pct_real', 'pct_estimada', 'pct_odom', 'x_odom', 'y_odom'])
        self.t0 = None
        self.ultimo_t = None
        self.terminada = False
        self.ultima_pose = (0.0, 0.0)

        self.pub_pct = self.create_publisher(Float32, '/cobertura/porcentaje', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/cobertura/celdas', 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.create_subscription(Bool, '/mision/fin', self.cb_fin, 10)
        self.create_subscription(Odometry, '/cobertura/pose', self.cb_fusion, 20)
        self.create_subscription(TFMessage, '/verdad/poses', self.cb_verdad, 20)
        self.create_timer(1.0, self.periodico)
        self.get_logger().info(f'Monitor listo. Registro en {self.ruta_csv}')

    def cb_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.ultima_pose = (x, y)
        self.ultimo_t = t - self.t0
        self.metrica.actualizar(x, y)
        ref = self.metrica_ver if self.hay_ver else (self.metrica_fus if self.hay_fus else self.metrica)
        if len(ref.visitadas) != getattr(self, '_n_log', 0):
            self._n_log = len(ref.visitadas)
            pct = ref.porcentaje()
            self.w.writerow([f'{t - self.t0:.2f}',
                             f'{self.metrica_ver.porcentaje():.2f}' if self.hay_ver else '',
                             f'{self.metrica_fus.porcentaje():.2f}',
                             f'{self.metrica.porcentaje():.2f}', f'{x:.3f}', f'{y:.3f}'])
            self.get_logger().info(
                f'Celda nueva ... cobertura del escenario: {pct:.1f}%  (t={t - self.t0:.0f}s)')
            if self.imprimir_matriz:
                self.get_logger().info(self.matriz_texto(ref))
            self.escribir_matriz(ref, t - self.t0)

    def matriz_texto(self, ref):
        """Matriz del escenario: 0 = no recorrible, 1 = recorrible pendiente, 2 = visitada."""
        filas = []
        for j in self._rango_j:
            fila = []
            for i in self._rango_i:
                c = (i, j)
                fila.append('2' if c in ref.visitadas else ('1' if c in self.recorribles else '0'))
            filas.append('[' + ' '.join(fila) + ']')
        return ('Matriz del escenario (0 = no recorrible, 1 = pendiente, 2 = visitada):\n'
                + '\n'.join(filas))

    def escribir_matriz(self, ref, t):
        if not self.archivo_matriz:
            return
        cartel = ''
        if getattr(self, 'terminada', False):
            linea = '=' * 58
            cartel = (f'{linea}\n'
                      f'   MISIÓN TERMINADA\n'
                      f'   Cobertura real del escenario: {ref.porcentaje():.1f} % '
                      f'({len(ref.visitadas)} de {len(self.recorribles)} celdas)\n')
            if self.hay_ver and getattr(self, 'err_n', 0):
                cartel += (f'   Error de localización: medio {100 * self.err_sum / self.err_n:.1f} cm, '
                           f'máx {100 * self.err_max:.1f} cm\n')
            cartel += f'{linea}\n\n'
        txt = cartel + (f'Cobertura del escenario: {ref.porcentaje():5.1f} %   '
               f'({len(ref.visitadas)} de {len(self.recorribles)} celdas)   t = {t:4.0f} s\n\n'
               + self.matriz_texto(ref) + '\n')
        try:
            tmp = self.archivo_matriz + '.tmp'
            with open(tmp, 'w') as f:
                f.write(txt)
            os.replace(tmp, self.archivo_matriz)   # reemplazo atómico: nunca se lee a medias
        except OSError:
            pass

    def cb_fusion(self, msg):
        self.hay_fus = True
        self.pose_fus = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.metrica_fus.actualizar(*self.pose_fus)

    def cb_verdad(self, msg):
        """Pose verdadera de Gazebo (solo validación).

        El puente no transmite los nombres de las entidades (child_frame_id
        vacío), así que se identifica al robot como la entidad a ras del piso
        más cercana a la pose estimada. Los cilindros quedan descartados porque
        su centro está elevado (z = 0.25 m), y las piezas del robot vienen
        expresadas respecto del modelo, no del mundo.
        Si la estimación se alejara más de 0.35 m de la verdad, la validación
        dejaría de actualizarse (y eso mismo sería evidencia de un error).
        """
        if getattr(self, 'pose_fus', None) is None:
            return
        fx, fy = self.pose_fus
        mejor, dmin = None, 0.35
        for tr in msg.transforms:
            p = tr.transform.translation
            if abs(p.z) > 0.1:
                continue
            d = ((p.x - fx) ** 2 + (p.y - fy) ** 2) ** 0.5
            if d < dmin:
                mejor, dmin = p, d
        if mejor is None:
            return
        self.hay_ver = True
        self.metrica_ver.actualizar(mejor.x, mejor.y)
        self.err_max = max(getattr(self, 'err_max', 0.0), dmin)
        self.err_sum = getattr(self, 'err_sum', 0.0) + dmin
        self.err_n = getattr(self, 'err_n', 0) + 1

    def resumen(self):
        n = len(self.recorribles)
        partes = []
        if self.hay_ver:
            partes.append(f'COBERTURA REAL {self.metrica_ver.porcentaje():.1f}% '
                          f'({len(self.metrica_ver.visitadas)} de {n} celdas recorribles)')
        if self.hay_fus:
            partes.append(f'según la pose estimada: {self.metrica_fus.porcentaje():.1f}%')
        partes.append(f'según /odom crudo: {self.metrica.porcentaje():.1f}%')
        if self.hay_ver:
            partes.append(f'error de localización: medio {100 * self.err_sum / self.err_n:.1f} cm, '
                          f'máx {100 * self.err_max:.1f} cm')
        return ' | '.join(partes)

    def cb_fin(self, msg):
        if msg.data:
            self.terminada = True
            self.escribir_matriz(self.metrica_ver if self.hay_ver else self.metrica_fus,
                                 (self.ultimo_t or 0.0))
            self.get_logger().info(f'RESULTADO FINAL: {self.resumen()}  (CSV: {self.ruta_csv})')
            self.csv.flush()

    def periodico(self):
        ref_pct = self.metrica_ver if self.hay_ver else self.metrica_fus
        self.pub_pct.publish(Float32(data=float(ref_pct.porcentaje())))
        self.csv.flush()
        arr = MarkerArray()
        ref = self.metrica_ver if self.hay_ver else (self.metrica_fus if self.hay_fus else self.metrica)
        for k, c in enumerate(sorted(self.recorribles)):
            x0, x1, y0, y1 = rectangulo(*c)
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.ns = 'celdas'
            mk.id = k
            mk.type = Marker.CUBE
            mk.pose.position.x = (x0 + x1) / 2
            mk.pose.position.y = (y0 + y1) / 2
            mk.pose.position.z = -0.01
            mk.pose.orientation.w = 1.0
            mk.scale.x = (x1 - x0) * 0.92
            mk.scale.y = (y1 - y0) * 0.92
            mk.scale.z = 0.01
            mk.color.a = 0.45
            if c in ref.visitadas:
                mk.color.g = 0.8
            else:
                mk.color.r = 0.9
                mk.color.g = 0.3
            arr.markers.append(mk)
        self.pub_mk.publish(arr)

    def cerrar(self):
        self.get_logger().info(f'Cobertura al cerrar: {self.resumen()}')
        self.csv.close()


def main(args=None):
    rclpy.init(args=args)
    nodo = Monitor()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
