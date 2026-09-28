#!/usr/bin/env python3
"""Simulador 2D del escenario turtlebot3_dqn_stage4 (sin ROS, sin Gazebo).

Sirve para probar y ajustar la lógica de misión en segundos, antes de ir a Gazebo.
Reproduce lo que importa del mundo real:
  - paredes exteriores e interiores con las medidas del .world de Jazzy
  - los 2 cilindros móviles con sus trayectorias y velocidad (0.1 m/s); son
    cinemáticos: si tocan al robot lo EMPUJAN (como SetWorldPoseCmd en Gazebo)
  - LiDAR 360 muestras, 5 Hz, 0.12-3.5 m, con ruido
  - odometría por integración de ruedas: un empujón NO aparece en /odom (deriva)

Uso:
    python3 herramientas/sim2d.py --tiempo 900 --png resultado.png
    python3 herramientas/sim2d.py --sin-obstaculos
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tb3_cobertura import escenario_verdad as ev  # noqa: E402
from tb3_cobertura.grilla_cobertura import Cobertura, rectangulo  # noqa: E402
from tb3_cobertura.mapa_ocupacion import MapaOcupacion  # noqa: E402
from tb3_cobertura.mision import FIN, Mision, Parametros  # noqa: E402
from tb3_cobertura.fusion import FusionOdomImu  # noqa: E402

# (cx, cy, largo, ancho, yaw) tomados de turtlebot3_dqn_world/model.sdf e inner_walls
PAREDES = [(2.425, 0, 5, 0.15, 1.5708), (0, 2.425, 5, 0.15, 0),
           (-2.425, 0, 5, 0.15, 1.5708), (0, -2.425, 5, 0.15, 0),
           (-2.0, -1.5, 1, 0.15, 0), (-0.5, -2.0, 1, 0.15, -1.5708),
           (1.0, -1.0, 1, 0.15, 1.5708), (1.2, 1.9, 1, 0.15, -1.5708),
           (1.9, 0.4, 1, 0.15, 0), (-0.5, 1.5, 1, 0.15, 0),
           (-1.2, 0.092, 1, 0.15, -1.5708)]
OBST1 = [(2.0, 2.0), (1.5, 1.0), (-1.5, 1.0), (-1.7, -1.0), (-1.5, 1.0), (1.5, 1.0), (2.0, 2.0)]
OBST2 = [(-2.0, -2.0), (-1.3, -1.8), (0.5, 2.0), (-2.0, 1.5), (1.5, -0.2), (1.5, -2.0),
         (0.0, -1.5), (-0.5, -1.0), (-1.0, -1.5), (-1.5, -1.9), (-2.0, -2.0)]
R_OBST = 0.12
R_ROBOT = 0.105       # círculo que envuelve la caja de colisión (centrado en el LiDAR)
OFF_LIDAR = -0.032


def _rects():
    out = []
    for cx, cy, L, W, yaw in PAREDES:
        if abs(math.sin(yaw)) > 0.5:
            L, W = W, L
        out.append((cx - L / 2, cx + L / 2, cy - W / 2, cy + W / 2))
    return np.array(out)


RECTS = _rects()


def obstaculos_juez(con_cilindros):
    """Obstáculos del escenario para el juez (mismo formato que lee el .world)."""
    o = ev.Obstaculos()
    for cx, cy, L, W, yaw in PAREDES:
        o.cajas.append((cx, cy, yaw, L, W))
    if con_cilindros:
        o.cilindros += [(OBST1[0][0], OBST1[0][1], R_OBST), (OBST2[0][0], OBST2[0][1], R_OBST)]
    return o


def pos_obstaculo(wps, t, v=0.1):
    seg = [math.dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1)]
    d = (t * v) % sum(seg)
    for i, s in enumerate(seg):
        if d <= s:
            a = d / s
            return (wps[i][0] + (wps[i + 1][0] - wps[i][0]) * a,
                    wps[i][1] + (wps[i + 1][1] - wps[i][1]) * a)
        d -= s
    return wps[-1]


def raycast(ox, oy, angs, circulos, rmax=3.5):
    c, s = np.cos(angs), np.sin(angs)
    best = np.full(angs.size, np.inf)
    # rectángulos (método de slabs)
    for x0, x1, y0, y1 in RECTS:
        with np.errstate(divide='ignore', invalid='ignore'):
            tx1, tx2 = (x0 - ox) / c, (x1 - ox) / c
            ty1, ty2 = (y0 - oy) / s, (y1 - oy) / s
        tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
        tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
        hit = (tmax >= tmin) & (tmin > 0)
        best = np.where(hit & (tmin < best), tmin, best)
    for cx, cy in circulos:
        dx, dy = ox - cx, oy - cy
        b = dx * c + dy * s
        disc = b * b - (dx * dx + dy * dy - R_OBST ** 2)
        with np.errstate(invalid='ignore'):
            t = -b - np.sqrt(disc)
        hit = (disc >= 0) & (t > 0)
        best = np.where(hit & (t < best), t, best)
    best = np.where(best > rmax, np.inf, best)
    return best


def dist_paredes(x, y):
    dx = np.maximum.reduce([RECTS[:, 0] - x, np.zeros(len(RECTS)), x - RECTS[:, 1]])
    dy = np.maximum.reduce([RECTS[:, 2] - y, np.zeros(len(RECTS)), y - RECTS[:, 3]])
    return float(np.min(np.hypot(dx, dy)))


def simular(t_total=900.0, obstaculos=True, semilla=0, verbose=True, params=None, v_obst=0.1,
            desfase=0.0, obst_quietos=False, patinamiento=0.0, usar_imu=True):
    rng = np.random.default_rng(semilla)
    dt = 0.05
    # estado real y odométrico
    x = y = th = 0.0
    ox_ = oy_ = oth = 0.0
    v_act = w_act = 0.0
    mapa = MapaOcupacion()
    recorribles = ev.celdas_recorribles(obstaculos_juez(obstaculos and obst_quietos))
    cobertura = Cobertura()                  # la del robot: sin datos del escenario
    metrica = ev.MetricaReal(recorribles)       # juez, sobre la pose de navegación
    metrica_odom = ev.MetricaReal(recorribles)  # juez, sobre /odom crudo
    metrica_real = ev.MetricaReal(recorribles)  # juez, sobre la pose verdadera
    fusion = FusionOdomImu()
    nx_, ny_, nth_ = 0.0, 0.0, 0.0
    mision = Mision(mapa, cobertura, p=params or Parametros(),
                    log=print if verbose else (lambda *a: None))
    angs_rel = np.arange(360) * (2 * math.pi / 360)
    scan = None
    v_cmd = w_cmd = 0.0
    contactos, en_contacto = 0, False
    historia, tray = [], []
    hist_odom = []
    n_evadir = 0
    estado_prev = None
    t = 0.0
    k = 0
    while t < t_total:
        obs = ([pos_obstaculo(OBST1, 0 if obst_quietos else t, v_obst),
                pos_obstaculo(OBST2, 0 if obst_quietos else t, v_obst)]
               if obstaculos else [])
        # LiDAR a 5 Hz
        if k % 4 == 0:
            lx, ly = x + OFF_LIDAR * math.cos(th), y + OFF_LIDAR * math.sin(th)
            r = raycast(lx, ly, th + angs_rel, obs)
            r = np.where(np.isfinite(r), r + rng.normal(0, 0.01, r.size), r)
            r = np.where(r < 0.12, np.inf, r)   # por debajo de range_min
            scan = r
            lag = int(round(desfase / dt))
            px_, py_, pth_ = hist_odom[-1 - lag] if len(hist_odom) > lag else (nx_, ny_, nth_)
            mapa.actualizar(px_, py_, pth_, r, 0.0, 2 * math.pi / 360, 0.12, 3.5, w=w_act)
        # control a 10 Hz
        if k % 2 == 0 and scan is not None:
            v_cmd, w_cmd = mision.paso(t, nx_, ny_, nth_, (scan, 0.0, 2 * math.pi / 360))
            if mision.estado == 'EVADIR' and estado_prev != 'EVADIR':
                n_evadir += 1
            estado_prev = mision.estado
            if mision.estado == FIN:
                break
        # dinámica con límites de aceleración
        v_act += max(-1.0 * dt, min(1.0 * dt, v_cmd - v_act))
        w_act += max(-3.0 * dt, min(3.0 * dt, w_cmd - w_act))
        nth = th + w_act * dt
        nx, ny = x + v_act * math.cos(nth) * dt, y + v_act * math.sin(nth) * dt
        cx, cy = nx + OFF_LIDAR * math.cos(nth), ny + OFF_LIDAR * math.sin(nth)
        choca_pared = dist_paredes(cx, cy) < R_ROBOT
        if not choca_pared:
            dx, dy, dth = nx - x, ny - y, nth - th
            x, y, th = nx, ny, nth
            # odometría: integra lo que giraron las ruedas (con leve error)
            ds = math.hypot(dx, dy) * np.sign(v_act) * (1 + rng.normal(0, 0.002))
            # el rumbo de las ruedas patina al girar (sobreestima el giro)
            oth += dth * (1 + patinamiento + rng.normal(0, 0.002))
            ox_ += ds * math.cos(oth)
            oy_ += ds * math.sin(oth)
        # empujón de obstáculos móviles (no lo ve la odometría)
        toca = choca_pared
        for ocx, ocy in obs:
            cx, cy = x + OFF_LIDAR * math.cos(th), y + OFF_LIDAR * math.sin(th)
            d = math.hypot(cx - ocx, cy - ocy)
            if d < R_ROBOT + R_OBST:
                toca = True
                emp = (R_ROBOT + R_OBST - d) + 1e-3
                ux, uy = (cx - ocx) / max(d, 1e-6), (cy - ocy) / max(d, 1e-6)
                if dist_paredes(cx + ux * emp, cy + uy * emp) >= R_ROBOT:
                    x, y = x + ux * emp, y + uy * emp
        if toca and not en_contacto:
            contactos += 1
            if verbose and obs:
                dmin = min(math.hypot(x - a, y - b) for a, b in obs)
                _, dd = mision._dinamico_mas_cercano(ox_, oy_)
                print(f'  CONTACTO t={t:.1f} estado={mision.estado} d_obst_real={dmin:.2f} '
                      f'd_dinamico_detectado={dd:.2f} pared={choca_pared} v={v_act:.2f}')
        en_contacto = toca
        # pose de navegación: odometría sola o fusionada con la IMU
        if usar_imu:
            fusion.imu_orientacion(th + rng.normal(0, 0.001))
            nx_, ny_, nth_ = fusion.odom(ox_, oy_, oth)
        else:
            nx_, ny_, nth_ = ox_, oy_, oth
        cobertura.actualizar(nx_, ny_)
        metrica.actualizar(nx_, ny_)
        metrica_odom.actualizar(ox_, oy_)
        metrica_real.actualizar(x, y)
        if k % 20 == 0:
            historia.append((t, metrica_real.porcentaje(), math.hypot(x - nx_, y - ny_)))
        tray.append((x, y))
        hist_odom.append((nx_, ny_, nth_))
        k += 1
        t = k * dt
    return dict(historia=historia, tray=np.array(tray), metrica=metrica, mapa=mapa,
                contactos=contactos, t_fin=t, motivo=mision.motivo_fin or 'tiempo de simulación',
                mision=mision, n_evadir=n_evadir,
                metrica_odom=metrica_odom, metrica_real=metrica_real)


def graficar(res, archivo):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.6))
    # 1) celdas recorribles del escenario (juez), según la pose verdadera
    a = ax[0]
    mr = res['metrica_real']
    for c in mr.recorribles:
        x0, x1, y0, y1 = rectangulo(*c)
        col = '#2a9d8f' if c in mr.visitadas else '#e76f51'
        a.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=col, ec='w', lw=0.5, alpha=0.8))
    for x0, x1, y0, y1 in RECTS:
        a.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc='k'))
    tr = res['tray']
    a.plot(tr[:, 0], tr[:, 1], lw=0.6, c='#264653')
    a.set_xlim(-2.6, 2.6)
    a.set_ylim(-2.6, 2.6)
    a.set_aspect('equal')
    a.set_title(f"Cobertura real: {mr.porcentaje():.1f}% de {len(mr.recorribles)} celdas recorribles")
    # 2) mapa del LiDAR
    a = ax[1]
    m = res['mapa']
    img = np.where(m.ocupado(), 0, np.where(m.libre(), 1, 0.6))
    a.imshow(img.T, origin='lower', cmap='gray',
             extent=[m.ox, m.ox + m.n * m.res, m.oy, m.oy + m.n * m.res])
    a.set_title('Mapa de ocupación construido con el LiDAR')
    # 3) curva de cobertura
    a = ax[2]
    h = np.array(res['historia'])
    a.plot(h[:, 0], h[:, 1], label='cobertura (%)')
    a.plot(h[:, 0], 100 * h[:, 2], label='deriva odometría (cm)')
    a.set_xlabel('tiempo simulado (s)')
    a.grid(alpha=0.3)
    a.legend()
    a.set_title(f"contactos: {res['contactos']}  fin: {res['motivo']}")
    plt.tight_layout()
    plt.savefig(archivo, dpi=110)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--tiempo', type=float, default=900.0)
    ap.add_argument('--sin-obstaculos', action='store_true')
    ap.add_argument('--semilla', type=int, default=0)
    ap.add_argument('--png', default='')
    ap.add_argument('--desfase', type=float, default=0.0,
                    help='retardo (s) entre scan y pose asociada, como en Gazebo')
    ap.add_argument('--obst-quietos', action='store_true')
    ap.add_argument('--patinamiento', type=float, default=0.0,
                    help='error relativo del rumbo de las ruedas al girar (ej. 0.01)')
    ap.add_argument('--sin-imu', action='store_true')
    ap.add_argument('--v-obst', type=float, default=0.1,
                    help='velocidad de los cilindros (simula real-time factor < 1)')
    a = ap.parse_args()
    res = simular(a.tiempo, not a.sin_obstaculos, a.semilla, v_obst=a.v_obst,
                  desfase=a.desfase, obst_quietos=a.obst_quietos,
                  patinamiento=a.patinamiento, usar_imu=not a.sin_imu)
    h = res['historia']
    for umbral in (50, 70, 80, 85):
        t = next((t for t, c, _ in h if c >= umbral), None)
        print(f'{umbral}% alcanzado en: {t if t is not None else "-"} s')
    print(f"FINAL real={res['metrica_real'].porcentaje():.1f}% (nav={res['metrica'].porcentaje():.1f}%)  t={res['t_fin']:.0f}s  "
          f"contactos={res['contactos']}  motivo={res['motivo']}  "
          f"deriva_final={h[-1][2]*100:.1f} cm  evasiones={res['n_evadir']}  "
          f"| /odom={res['metrica_odom'].porcentaje():.1f}%")
    if a.png:
        graficar(res, a.png)
