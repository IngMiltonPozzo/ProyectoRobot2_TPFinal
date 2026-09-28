"""Fusión de odometría de ruedas con la IMU (sin dependencias de ROS).

Problema observado en Gazebo: al girar en el lugar las ruedas patinan un poco y
la odometría de ruedas acumula error de RUMBO. Con unos grados de error, todo el
mapa del LiDAR queda rotado y la métrica sobre /odom cuenta celdas en lugares
donde el robot no estuvo.

Solución (dead reckoning con rumbo inercial):
  - la TRASLACIÓN se toma de las ruedas (buena en tramos rectos),
  - el RUMBO se toma de la IMU (orientación del sensor, o integración del
    giróscopo si la orientación no viene disponible), que no se ve afectada
    por el patinamiento.
"""
import math


def _ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class FusionOdomImu:
    def __init__(self):
        self.pose = None          # (x, y, th) fusionada
        self._odom_prev = None    # (x, y, th) de /odom en la actualización anterior
        self._offset = None       # th_odom0 - yaw_imu0 (alinea ambos marcos al inicio)
        self.yaw_imu = None       # último rumbo inercial disponible
        self._t_gyro = None

    # ------------------------------------------------------------ entradas IMU
    def imu_orientacion(self, yaw):
        self.yaw_imu = yaw

    def imu_giroscopo(self, t, wz):
        """Alternativa si la IMU no publica orientación: integra el giróscopo."""
        if self._t_gyro is not None and self.yaw_imu is not None:
            dt = t - self._t_gyro
            if 0.0 < dt < 0.5:
                self.yaw_imu = _ang(self.yaw_imu + wz * dt)
        elif self.yaw_imu is None:
            self.yaw_imu = 0.0
        self._t_gyro = t

    # ------------------------------------------------------------ odometría
    def odom(self, x, y, th):
        """Actualiza con una nueva lectura de /odom y devuelve la pose fusionada."""
        if self.pose is None:
            self.pose = (x, y, th)
            self._odom_prev = (x, y, th)
            if self.yaw_imu is not None:
                self._offset = _ang(th - self.yaw_imu)
            return self.pose
        if self._offset is None and self.yaw_imu is not None:
            # la IMU llegó después: se alinea con el rumbo fusionado actual
            self._offset = _ang(self.pose[2] - self.yaw_imu)

        xo, yo, tho = self._odom_prev
        dx, dy = x - xo, y - yo
        # desplazamiento en el marco del robot, según la odometría
        c, s = math.cos(tho), math.sin(tho)
        dxr = c * dx + s * dy
        dyr = -s * dx + c * dy
        self._odom_prev = (x, y, th)

        px, py, pth = self.pose
        if self._offset is not None:
            th_new = _ang(self.yaw_imu + self._offset)
        else:
            th_new = _ang(pth + _ang(th - tho))
        th_mid = _ang(pth + 0.5 * _ang(th_new - pth))
        cm, sm = math.cos(th_mid), math.sin(th_mid)
        self.pose = (px + cm * dxr - sm * dyr, py + sm * dxr + cm * dyr, th_new)
        return self.pose
