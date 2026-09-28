"""Verdad del escenario, SOLO para evaluar (la usa el monitor, nunca el robot).

El robot no tiene ningún dato precargado del escenario: descubre todo con el
LiDAR. Para saber qué porcentaje del escenario monitoreó hace falta un juez
que sí conozca el mundo, igual que un árbitro. Este módulo:

1. lee el archivo .world de Gazebo y los modelos que incluye, y extrae los
   obstáculos (cajas y cilindros) con sus poses en el plano;
2. calcula qué celdas de la grilla de 25 cm son RECORRIBLES: aquellas que
   contienen al menos un punto donde el centro del robot puede estar sin
   chocar (distancia a todo obstáculo >= radio del robot) y al que se puede
   llegar desde la posición inicial.

El denominador del porcentaje sale de acá, de la geometría real del mundo.
"""
import math
import os
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

from .grilla_cobertura import TAM_CELDA, celda


# ------------------------------------------------------------------ geometría 2D
def _pose(elem):
    """(x, y, yaw) de un <pose> SDF (ignora z, roll y pitch)."""
    if elem is None or not (elem.text or '').strip():
        return (0.0, 0.0, 0.0)
    v = [float(t) for t in elem.text.split()]
    v += [0.0] * (6 - len(v))
    return (v[0], v[1], v[5])


def _componer(a, b):
    """Pose b expresada en el marco a -> pose en el marco de a's padre."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


class Obstaculos:
    """Cajas (cx, cy, yaw, largo, ancho) y cilindros (cx, cy, radio)."""

    def __init__(self):
        self.cajas = []
        self.cilindros = []

    def distancia(self, px, py):
        """Distancia (vectorizada) de los puntos (px, py) al obstáculo más cercano."""
        d = np.full(np.shape(px), np.inf)
        for cx, cy, yaw, lx, ly in self.cajas:
            c, s = math.cos(-yaw), math.sin(-yaw)
            qx = c * (px - cx) - s * (py - cy)
            qy = s * (px - cx) + c * (py - cy)
            dx = np.maximum(np.abs(qx) - lx / 2, 0.0)
            dy = np.maximum(np.abs(qy) - ly / 2, 0.0)
            d = np.minimum(d, np.hypot(dx, dy))
        for cx, cy, r in self.cilindros:
            d = np.minimum(d, np.maximum(np.hypot(px - cx, py - cy) - r, 0.0))
        return d


# ------------------------------------------------------------------ lectura SDF
def _resolver_uri(uri, rutas_modelos):
    if not uri.startswith('model://'):
        return None  # p. ej. el piso de Fuel: no aporta obstáculos
    rel = uri[len('model://'):]
    for base in rutas_modelos:
        f = os.path.join(base, rel, 'model.sdf')
        if os.path.isfile(f):
            return f
    return None


def _leer_modelo(archivo, pose_modelo, obst, incluir_cilindros, rutas_modelos):
    raiz = ET.parse(archivo).getroot()
    modelo = raiz.find('model')
    if modelo is None:
        return
    pose = _componer(pose_modelo, _pose(modelo.find('pose')))
    _leer_elemento_modelo(modelo, pose, obst, incluir_cilindros, rutas_modelos)


def _leer_elemento_modelo(modelo, pose, obst, incluir_cilindros, rutas_modelos):
    for link in modelo.findall('link'):
        pl = _componer(pose, _pose(link.find('pose')))
        for col in link.findall('collision'):
            pc = _componer(pl, _pose(col.find('pose')))
            geo = col.find('geometry')
            if geo is None:
                continue
            caja = geo.find('box')
            cil = geo.find('cylinder')
            if caja is not None:
                lx, ly, _ = [float(t) for t in caja.find('size').text.split()]
                obst.cajas.append((pc[0], pc[1], pc[2], lx, ly))
            elif cil is not None and incluir_cilindros:
                obst.cilindros.append((pc[0], pc[1], float(cil.find('radius').text)))
    for sub in modelo.findall('model'):
        _leer_elemento_modelo(sub, _componer(pose, _pose(sub.find('pose'))),
                              obst, incluir_cilindros, rutas_modelos)
    for inc in modelo.findall('include'):
        _leer_include(inc, pose, obst, incluir_cilindros, rutas_modelos)


def _leer_include(inc, pose_padre, obst, incluir_cilindros, rutas_modelos):
    uri = (inc.findtext('uri') or '').strip()
    archivo = _resolver_uri(uri, rutas_modelos)
    if archivo is None:
        return
    pose = _componer(pose_padre, _pose(inc.find('pose')))
    _leer_modelo(archivo, pose, obst, incluir_cilindros, rutas_modelos)


def leer_mundo(archivo_world, rutas_modelos, incluir_cilindros=True):
    """Obstáculos del mundo. incluir_cilindros=False si los cilindros se mueven."""
    obst = Obstaculos()
    mundo = ET.parse(archivo_world).getroot().find('world')
    origen = (0.0, 0.0, 0.0)
    for inc in mundo.findall('include'):
        _leer_include(inc, origen, obst, incluir_cilindros, rutas_modelos)
    for mod in mundo.findall('model'):
        _leer_elemento_modelo(mod, _componer(origen, _pose(mod.find('pose'))),
                              obst, incluir_cilindros, rutas_modelos)
    return obst


# ------------------------------------------------------------------ celdas recorribles
def celdas_recorribles(obst, radio_robot=0.13, inicio=(0.0, 0.0), paso=0.02, extension=8.0):
    """Conjunto de celdas (i, j) de la grilla de 25 cm que el robot puede pisar.

    Una celda es recorrible si contiene un punto con distancia a todo obstáculo
    >= radio_robot y conectado (por puntos igualmente seguros) con el inicio.
    """
    n = int(round(2 * extension / paso))
    xs = inicio[0] - extension + (np.arange(n) + 0.5) * paso
    ys = inicio[1] - extension + (np.arange(n) + 0.5) * paso
    px, py = np.meshgrid(xs, ys, indexing='ij')
    libre = obst.distancia(px, py) >= radio_robot
    i0 = int(np.argmin(np.abs(xs - inicio[0])))
    j0 = int(np.argmin(np.abs(ys - inicio[1])))
    if not libre[i0, j0]:
        raise ValueError('la posición inicial no es libre')
    alcanzable = np.zeros_like(libre)
    alcanzable[i0, j0] = True
    cola = deque([(i0, j0)])
    while cola:
        i, j = cola.popleft()
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < n and 0 <= b < n and libre[a, b] and not alcanzable[a, b]:
                alcanzable[a, b] = True
                cola.append((a, b))
    # si el escenario no es cerrado, la región alcanzable toca el borde: error
    if alcanzable[0, :].any() or alcanzable[-1, :].any() or alcanzable[:, 0].any() or alcanzable[:, -1].any():
        raise ValueError('el escenario no está cerrado: la región recorrible no tiene límite')
    ii, jj = np.nonzero(alcanzable)
    return {celda(xs[i], ys[j]) for i, j in zip(ii, jj)}


class MetricaReal:
    """Porcentaje de celdas recorribles del escenario por las que pasó el robot."""

    def __init__(self, recorribles):
        self.recorribles = set(recorribles)
        self.visitadas = set()

    def actualizar(self, x, y):
        c = celda(x, y)
        if c in self.recorribles and c not in self.visitadas:
            self.visitadas.add(c)
            return True
        return False

    def porcentaje(self):
        return 100.0 * len(self.visitadas) / max(1, len(self.recorribles))


def rutas_modelos_por_defecto(share_turtlebot3_gazebo):
    rutas = [os.path.join(share_turtlebot3_gazebo, 'models')]
    for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        rutas += [p for p in os.environ.get(var, '').split(':') if p]
    return rutas


__all__ = ['Obstaculos', 'leer_mundo', 'celdas_recorribles', 'rutas_modelos_por_defecto', 'MetricaReal',
           'TAM_CELDA']
