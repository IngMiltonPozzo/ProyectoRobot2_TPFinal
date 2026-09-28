"""Lógica de misión independiente de ROS (se usa igual en el nodo y en el simulador 2D).

Máquina de estados:
    INICIO     -> giro de 360° en el lugar para construir el primer mapa.
    PLANIFICAR -> pide al planificador la próxima celda y su camino.
    SEGUIR     -> sigue waypoints (control de rumbo P + velocidad adaptativa).
    EVADIR     -> un obstáculo MÓVIL se acerca: se aleja hacia el lado más libre.
    RECUPERAR  -> sin progreso / bloqueado: retrocede un poco y replanifica.
    FIN        -> no quedan celdas alcanzables (o se agotó el tiempo): se detiene.

Capa de seguridad (siempre al final, gana sobre todo lo demás):
    - distancia frontal < d_stop  -> v = 0 (solo se permite girar)
    - entre d_stop y d_lento      -> v escalada linealmente
"""
import math

import numpy as np

from .planificador import Planificador

INICIO, PLANIFICAR, SEGUIR, EVADIR, RECUPERAR, FIN = (
    'INICIO', 'PLANIFICAR', 'SEGUIR', 'EVADIR', 'RECUPERAR', 'FIN')


def _ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Rastreador:
    """Agrupa los puntos dinámicos del LiDAR en obstáculos y estima su velocidad."""

    def __init__(self, dist_grupo=0.30, dist_asoc=0.35, alfa=0.5, olvido=1.0,
                 min_obs=3, v_min=0.04, v_max=0.5, min_edad=1.0, min_desp=0.15,
                 cerca_de_pared=None):
        self.dist_grupo = dist_grupo
        self.dist_asoc = dist_asoc
        self.alfa = alfa
        self.olvido = olvido
        self.min_obs = min_obs
        self.v_min = v_min
        self.v_max = v_max
        self.min_edad = min_edad
        self.min_desp = min_desp
        self.cerca_de_pared = cerca_de_pared
        self.pistas = []  # dicts: p (x,y), v (vx,vy), t

    def actualizar(self, t, puntos, x, y):
        grupos = []
        for px, py in puntos:
            for g in grupos:
                if math.hypot(px - g[0] / g[2], py - g[1] / g[2]) < self.dist_grupo:
                    g[0] += px
                    g[1] += py
                    g[2] += 1
                    break
            else:
                grupos.append([px, py, 1])
        centros = []
        for gx, gy, n in grupos:
            if n < 2:
                continue  # un punto suelto suele ser ruido
            cx, cy = gx / n, gy / n
            # los impactos están en la cara visible: el centro está ~r más lejos
            d = math.hypot(cx - x, cy - y)
            if d > 1e-3:
                cx += 0.08 * (cx - x) / d
                cy += 0.08 * (cy - y) / d
            centros.append((cx, cy))
        nuevas = []
        for cx, cy in centros:
            mejor, md = None, self.dist_asoc
            for tr in self.pistas:
                d = math.hypot(cx - tr['p'][0], cy - tr['p'][1])
                if d < md:
                    mejor, md = tr, d
            if mejor is not None:
                dt = max(t - mejor['t'], 1e-3)
                vx = (cx - mejor['p'][0]) / dt
                vy = (cy - mejor['p'][1]) / dt
                a = self.alfa
                mejor['v'] = (a * vx + (1 - a) * mejor['v'][0], a * vy + (1 - a) * mejor['v'][1])
                mejor['p'], mejor['t'] = (cx, cy), t
                mejor['n'] += 1
                nuevas.append(mejor)
                self.pistas.remove(mejor)
            else:
                nuevas.append({'p': (cx, cy), 'v': (0.0, 0.0), 't': t, 'n': 1,
                               'p0': (cx, cy), 't0': t})
        # conserva un rato las que no se vieron (oclusión breve)
        nuevas += [tr for tr in self.pistas if t - tr['t'] < self.olvido]
        self.pistas = nuevas

    def confirmadas(self):
        """Pistas que se comportan como un obstáculo que REALMENTE se mueve.

        Un borde de pared visto desde otro ángulo "tiembla" (su centroide salta
        algunos cm) pero no se traslada. Un cilindro en movimiento sí: avanza de
        forma sostenida. Por eso se exige:
          - haberla visto en varios scans y durante un tiempo mínimo,
          - desplazamiento neto desde que apareció >= min_desplazamiento,
          - velocidad media (desplazamiento / tiempo) de un objeto móvil real,
          - no estar pegada a una pared firme del mapa (lo decide el nodo).
        """
        res = []
        for tr in self.pistas:
            edad = tr['t'] - tr['t0']
            if tr['n'] < self.min_obs or edad < self.min_edad:
                continue
            desp = math.hypot(tr['p'][0] - tr['p0'][0], tr['p'][1] - tr['p0'][1])
            v_media = desp / max(edad, 1e-3)
            if desp < self.min_desp or not (self.v_min <= v_media <= self.v_max):
                continue
            if self.cerca_de_pared is not None and self.cerca_de_pared(*tr['p']):
                continue
            res.append(tr)
        return res

    def riesgo(self, x, y, vx_r=0.0, vy_r=0.0, horizonte=3.0, t=None):
        """Mínima distancia prevista a cualquier obstáculo en el horizonte."""
        dmin = float('inf')
        for tr in self.confirmadas():
            px, py = tr['p'][0] - x, tr['p'][1] - y
            if t is not None:  # extrapola a "ahora"
                dt = t - tr['t']
                px += tr['v'][0] * dt
                py += tr['v'][1] * dt
            ux, uy = tr['v'][0] - vx_r, tr['v'][1] - vy_r
            uu = ux * ux + uy * uy
            ts = 0.0 if uu < 1e-6 else min(max(-(px * ux + py * uy) / uu, 0.0), horizonte)
            dmin = min(dmin, math.hypot(px + ux * ts, py + uy * ts))
        return dmin


class Parametros:
    v_max = 0.20          # m/s  (Burger admite 0.22)
    w_max = 1.2           # rad/s (girar más lento = menos patinamiento de ruedas)
    acel_w = 2.5          # rad/s²: rampa de velocidad angular
    acel_v = 0.5          # m/s²: rampa de aceleración lineal (frenar es inmediato)
    k_rumbo = 2.2         # ganancia P de rumbo
    giro_en_lugar = 0.55  # rad: con más error que esto, gira sin avanzar
    d_stop = 0.17         # m (medido desde el LiDAR) -> freno total
    d_lento = 0.40        # m -> empieza a frenar
    d_lateral = 0.12      # m -> nada puede estar tan cerca en el semiplano de avance
    cono_frontal = 0.45   # rad (±26°)
    tol_waypoint = 0.08   # m
    tol_objetivo = 0.06   # m
    evitar_moviles = True  # False: escenario sin obstáculos móviles (no se evade)
    w_max_deteccion = 0.5  # rad/s: girando más rápido no se detectan móviles
    d_pared_movil = 0.15   # m: una pista más cerca que esto de una pared firme se ignora
    min_desp_movil = 0.08  # m: desplazamiento neto mínimo para confirmar un móvil
    min_edad_movil = 0.5   # s: tiempo mínimo de seguimiento antes de confirmarlo
    d_peligro = 0.42      # m: distancia mínima prevista a un obstáculo móvil -> evadir
    horizonte = 3.0       # s: horizonte de predicción
    v_escape = 0.18       # m/s
    t_calma = 1.0         # s sin peligro para salir de EVADIR
    t_evadir_max = 8.0    # s
    t_sin_progreso = 6.0  # s
    t_recuperar = 1.5     # s
    t_mision_max = 1200.0  # s
    w_giro_inicial = 0.6  # rad/s (debajo de w_max_mapeo: mapea durante el giro)
    r_paso = 0.16
    r_objetivo = 0.16
    # fase de repaso: al terminar, se intentan las celdas pegadas a paredes con
    # menos margen y más despacio (0 = desactivada)
    r_repaso = 0.15
    t_repaso_max = 60.0  # s: el repaso no se estira indefinidamente
    margen_repaso = 0.03  # m: margen dentro de la celda en la fase de repaso
    margen_celda = 0.03   # m: el punto objetivo queda al menos esto adentro de su celda
    v_repaso = 0.10


class Mision:
    def __init__(self, mapa, cobertura, p=None, log=print):
        self.p = p or Parametros()
        self.mapa = mapa
        self.cobertura = cobertura
        self.plan = Planificador(mapa, r_paso=self.p.r_paso, r_objetivo=self.p.r_objetivo,
                                 margen_celda=self.p.margen_celda)
        self.log = log
        self.estado = INICIO
        self.t_estado = None
        self.giro_acum = 0.0
        self.th_prev = None
        self.objetivo = None
        self.wps = []
        self.mejor_dist = float('inf')
        self.t_mejor = 0.0
        self.t_inicio = None
        self.evadir_dir = 0.0
        self.evadir_sentido = 1.0
        self.rastreador = Rastreador(cerca_de_pared=self._cerca_de_pared,
                                     min_desp=self.p.min_desp_movil,
                                     min_edad=self.p.min_edad_movil)
        self._scan_previo = None
        self._t_scan_prev = None
        self._th_scan_prev = None
        self.t_ultimo_peligro = 0.0
        self.motivo_fin = ''
        self.repaso = False
        self.t_repaso = None
        self.scan = None
        self._cmd_prev = (0.0, 0.0, None)

    # ------------------------------------------------------------ utilidades
    def _cerca_de_pared(self, x, y):
        """True si (x, y) está a menos de d_pared_movil de una pared firme del mapa."""
        firme = self.mapa.firme_cache()
        r = int(math.ceil(self.p.d_pared_movil / self.mapa.res))
        i, j = self.mapa.a_celda(x, y)
        i0, i1 = max(0, i - r), min(self.mapa.n, i + r + 1)
        j0, j1 = max(0, j - r), min(self.mapa.n, j + r + 1)
        return bool(firme[i0:i1, j0:j1].any()) if i0 < i1 and j0 < j1 else False

    def _cambiar(self, estado, t):
        if estado != self.estado and {estado, self.estado} != {SEGUIR, PLANIFICAR}:
            self.log(f'[{t:7.1f}s] {self.estado} -> {estado}')
        self.estado = estado
        self.t_estado = t

    def _sector_min(self, centro, semiancho):
        """Mínima distancia del scan en un sector (ángulo relativo al robot)."""
        r, a0, inc = self.scan
        n = r.size
        ang = a0 + inc * np.arange(n)
        d = np.abs(_ang_vec(ang - centro))
        sel = r[(d <= semiancho) & np.isfinite(r)]
        return float(sel.min()) if sel.size else float('inf')

    def _dinamico_mas_cercano(self, x, y):
        pts = self.mapa.dinamicos
        if pts.shape[0] == 0:
            return None, float('inf')
        d = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
        k = int(np.argmin(d))
        return pts[k], float(d[k])

    def _elegir_escape(self, t, x, y, th):
        """Rumbo y sentido (adelante/atrás) que maximizan la distancia prevista al
        obstáculo, con espacio libre suficiente según el LiDAR. Incluye quedarse quieto."""
        p = self.p
        mejor = (None, 0.0, self.rastreador.riesgo(x, y, horizonte=p.horizonte, t=t) - 0.05)
        for k in range(16):
            h = -math.pi + k * (2 * math.pi / 16)
            libre = self._sector_min(_ang(h - th), 0.30)
            if libre < 0.30:
                continue
            vx, vy = p.v_escape * math.cos(h), p.v_escape * math.sin(h)
            d = self.rastreador.riesgo(x, y, vx, vy, p.horizonte, t)
            # cuánto hay que girar: adelante o marcha atrás, lo que sea menos
            e_f = abs(_ang(h - th))
            e_r = abs(_ang(h + math.pi - th))
            giro = min(e_f, e_r)
            score = min(d, 1.0) + 0.15 * min(libre, 1.0) - 0.12 * giro
            if score > mejor[2]:
                mejor = (h, 1.0 if e_f <= e_r else -1.0, score)
        return mejor[0], mejor[1]

    # ------------------------------------------------------------ paso principal
    def paso(self, t, x, y, th, scan):
        """scan = (rangos np.array, angle_min, angle_increment). Devuelve (v, w)."""
        self.scan = scan
        if self.t_inicio is None:
            self.t_inicio = t
            self.t_estado = t
            self.th_prev = th
        p = self.p

        if self.estado != FIN and t - self.t_inicio > p.t_mision_max:
            self.motivo_fin = 'tiempo máximo'
            self._cambiar(FIN, t)
        if self.estado != FIN and self.repaso and t - self.t_repaso > p.t_repaso_max:
            self.motivo_fin = 'fin del tiempo de repaso'
            self._cambiar(FIN, t)

        # percepción de obstáculos móviles (solo cuando llega un scan nuevo)
        if scan[0] is not self._scan_previo:
            self._scan_previo = scan[0]
            # girando rápido, el desfase scan/odometría "corre" las paredes:
            # esos puntos no se usan para detectar obstáculos móviles
            dt_s = t - self._t_scan_prev if self._t_scan_prev is not None else 1.0
            w_est = abs(_ang(th - self._th_scan_prev)) / max(dt_s, 1e-3) \
                if self._th_scan_prev is not None else 0.0
            self._t_scan_prev, self._th_scan_prev = t, th
            pts = self.mapa.dinamicos if w_est < p.w_max_deteccion else np.zeros((0, 2))
            if p.evitar_moviles:
                self.rastreador.actualizar(t, pts, x, y)
        riesgo = (self.rastreador.riesgo(x, y, horizonte=p.horizonte, t=t)
                  if p.evitar_moviles else float('inf'))
        if riesgo < p.d_peligro:
            self.t_ultimo_peligro = t
        # prioridad: obstáculo móvil en curso de colisión
        if self.estado in (PLANIFICAR, SEGUIR, RECUPERAR) and riesgo < p.d_peligro:
            pistas = ', '.join(
                f"({tr['p'][0]:.2f},{tr['p'][1]:.2f}) v=({tr['v'][0]:.2f},{tr['v'][1]:.2f})"
                for tr in self.rastreador.confirmadas())
            self.log(f'[{t:7.1f}s] peligro {riesgo:.2f} m | robot ({x:.2f},{y:.2f}) '
                     f'| obstáculos: {pistas}')
            self._cambiar(EVADIR, t)

        v, w = 0.0, 0.0
        if self.estado == INICIO:
            self.giro_acum += abs(_ang(th - self.th_prev))
            self.th_prev = th
            w = p.w_giro_inicial
            if self.giro_acum >= 2 * math.pi:
                self._cambiar(PLANIFICAR, t)

        elif self.estado == PLANIFICAR:
            self.objetivo, self.wps = self.plan.planificar(x, y, th, self.cobertura, ahora=t)
            if self.objetivo is None and not self.repaso and 0 < p.r_repaso < p.r_objetivo:
                self.repaso = True
                self.plan.r_obj = self.plan.r_paso = p.r_repaso
                self.plan.margen = p.margen_repaso
                self.t_repaso = t
                self.plan.fallos.clear()
                self.plan.bloqueo.clear()
                self.log(f'[{t:7.1f}s] fase de repaso ({len(self.cobertura)} celdas visitadas)')
                self.objetivo, self.wps = self.plan.planificar(x, y, th, self.cobertura, ahora=t)
            if self.objetivo is None:
                self.motivo_fin = 'no quedan celdas alcanzables'
                self._cambiar(FIN, t)
            else:
                self.mejor_dist = float('inf')
                self.t_mejor = t
                self._cambiar(SEGUIR, t)

        elif self.estado == SEGUIR:
            v, w = self._seguir(t, x, y, th)

        elif self.estado == EVADIR:
            h, sentido = self._elegir_escape(t, x, y, th)
            if h is not None:
                obj = h if sentido > 0 else h + math.pi
                e = _ang(obj - th)
                w = p.k_rumbo * e
                if abs(e) < 0.9:  # avanza mientras termina de girar
                    v = sentido * p.v_escape * max(0.0, math.cos(e))
            if t - self.t_ultimo_peligro > p.t_calma or t - self.t_estado > p.t_evadir_max:
                self._cambiar(PLANIFICAR, t)

        elif self.estado == RECUPERAR:
            atras = self._sector_min(math.pi, 0.6)
            v = -0.08 if atras > 0.25 else 0.0
            w = 0.4
            if t - self.t_estado > p.t_recuperar:
                self._cambiar(PLANIFICAR, t)

        # -------- capa de seguridad (siempre al final)
        # roce lateral: algo a menos de d_lateral en el semiplano hacia donde avanza
        if v != 0.0 and self._sector_min(0.0 if v > 0 else math.pi, 1.4) < p.d_lateral:
            v = 0.0
        if v > 0:
            frente = self._sector_min(0.0, p.cono_frontal)
            if frente < p.d_stop:
                v = 0.0
            elif frente < p.d_lento:
                v *= (frente - p.d_stop) / (p.d_lento - p.d_stop)
        elif v < 0:
            atras = self._sector_min(math.pi, 0.6)
            if atras < p.d_stop + 0.08:  # el LiDAR está 3 cm atrás del eje
                v = 0.0
        w = max(-p.w_max, min(p.w_max, w))
        # rampas: giros suaves (la odometría patina menos); frenar nunca se demora
        v0, w0, t0 = self._cmd_prev
        if t0 is not None:
            dt = max(0.0, min(t - t0, 0.5))
            dw = p.acel_w * dt
            w = max(w0 - dw, min(w0 + dw, w))
            if abs(v) > abs(v0) and v * v0 >= 0:
                dv = p.acel_v * dt
                v = max(v0 - dv, min(v0 + dv, v))
        self._cmd_prev = (v, w, t)
        return v, w

    def _seguir(self, t, x, y, th):
        p = self.p
        # objetivo cumplido: la métrica ya contó la celda
        if self.cobertura.visitada(*self.objetivo):
            self._cambiar(PLANIFICAR, t)
            return 0.0, 0.0
        if not self.wps:
            self._cambiar(PLANIFICAR, t)
            return 0.0, 0.0
        wx, wy = self.wps[0]
        d = math.hypot(wx - x, wy - y)
        ultimo = len(self.wps) == 1
        if d < (p.tol_objetivo if ultimo else p.tol_waypoint):
            self.wps.pop(0)
            if ultimo:
                # llegó al punto pero la métrica no la contó: se intenta otra vez luego
                self.plan.registrar_fallo(self.objetivo, t)
                self._cambiar(PLANIFICAR, t)
            return 0.0, 0.0

        # vigilancia de progreso hacia el objetivo final
        fx, fy = self.wps[-1]
        dfin = math.hypot(fx - x, fy - y)
        if dfin < self.mejor_dist - 0.03:
            self.mejor_dist, self.t_mejor = dfin, t
        elif t - self.t_mejor > p.t_sin_progreso:
            self.log(f'[{t:7.1f}s] sin progreso hacia {self.objetivo}')
            self.plan.registrar_fallo(self.objetivo, t)
            self._cambiar(RECUPERAR, t)
            return 0.0, 0.0

        # el camino dejó de ser válido (apareció algo en el mapa) -> replanificar
        if not self.mapa.segmento_libre(x, y, wx, wy, p.r_paso * 0.8) and d > 0.25:
            if self._sector_min(_ang(math.atan2(wy - y, wx - x) - th), 0.3) < d:
                self._cambiar(PLANIFICAR, t)
                return 0.0, 0.0

        e = _ang(math.atan2(wy - y, wx - x) - th)
        if abs(e) > p.giro_en_lugar:
            return 0.0, p.k_rumbo * e
        # velocidad proporcional al alineamiento; no hace falta frenar al llegar:
        # la celda cuenta apenas el centro del robot entra en ella
        v = (p.v_repaso if self.repaso else p.v_max) * max(0.0, math.cos(e)) ** 2
        return v, p.k_rumbo * e


def _ang_vec(a):
    return (a + np.pi) % (2 * np.pi) - np.pi
