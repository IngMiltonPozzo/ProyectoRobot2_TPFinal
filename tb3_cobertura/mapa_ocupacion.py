"""Mapa de ocupación 2D construido en línea a partir del LiDAR y la odometría.

Es la capa de percepción: el robot NO usa la matriz del profesor para saber
por dónde puede pasar (esa matriz tiene paredes corridas respecto del mundo
real de Jazzy); usa lo que ve el sensor.

- Log-odds por celda (resolución 5 cm), con ray-tracing: las celdas atravesadas
  por un haz se vuelven "libres" y la celda del impacto "ocupada". Así, cuando
  un obstáculo móvil se va, su rastro se borra solo.
- Contador de observaciones libres por celda: si un impacto cae en un lugar que
  se vio libre muchas veces, ese punto se clasifica como obstáculo DINÁMICO.
- Clearance: distancia de cada celda al obstáculo más cercano (para planificar
  con margen de seguridad y para inflar el robot).
"""
import math
import numpy as np


class MapaOcupacion:
    def __init__(self, resolucion=0.05, tam=12.0, origen=None,
                 l_ocupado=0.9, l_libre=-0.4, l_max=4.0,
                 umbral_ocupado=0.6, umbral_libre=-0.6,
                 alcance_util=3.0, offset_lidar=-0.032,
                 min_libres_dinamico=8, clearance_max=0.4, w_max_mapeo=0.9):
        # El mapa no supone nada del escenario: es un lienzo amplio (12 x 12 m)
        # centrado en la posición inicial del robot, que se va llenando con lo
        # que el LiDAR descubre.
        self.res = resolucion
        self.n = int(round(tam / resolucion))
        if origen is None:
            origen = (-tam / 2.0, -tam / 2.0)
        self.ox, self.oy = origen
        self.lo = np.zeros((self.n, self.n), dtype=np.float32)
        self.libres = np.zeros((self.n, self.n), dtype=np.int16)
        self.impactos = np.zeros((self.n, self.n), dtype=np.int16)
        self.l_occ, self.l_free, self.l_max = l_ocupado, l_libre, l_max
        self.u_occ, self.u_free = umbral_ocupado, umbral_libre
        self.alcance = alcance_util
        self.off = offset_lidar
        self.min_libres_din = min_libres_dinamico
        self.cmax = clearance_max
        self.w_max_mapeo = w_max_mapeo
        self._version = 0
        self._pasos = np.arange(0.0, alcance_util + 1e-6, resolucion * 0.8)
        self._clear = np.zeros((self.n, self.n), dtype=np.float32)
        self._clear_valido = False
        # desplazamientos para la transformada de distancia (disco de radio cmax)
        r = int(math.ceil(clearance_max / resolucion))
        offs = [(i, j, math.hypot(i, j) * resolucion)
                for i in range(-r, r + 1) for j in range(-r, r + 1)
                if 0 < math.hypot(i, j) * resolucion <= clearance_max]
        offs.sort(key=lambda t: t[2])
        self._offs = offs
        self.dinamicos = np.zeros((0, 2))  # últimos puntos dinámicos (x, y) en odom

    # ------------------------------------------------------------ conversiones
    def a_celda(self, x, y):
        return int((x - self.ox) / self.res), int((y - self.oy) / self.res)

    def a_mundo(self, i, j):
        return self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res

    def dentro(self, i, j):
        return 0 <= i < self.n and 0 <= j < self.n

    # ------------------------------------------------------------ actualización
    def actualizar(self, x, y, th, rangos, ang_min, ang_inc, r_min, r_max, w=0.0):
        """Integra un LaserScan tomado en la pose (x, y, th) de base_footprint.

        w: velocidad angular del robot (rad/s). Girando rápido, cualquier desfase
        entre el instante del scan y la pose asociada "corre" las paredes; esos
        scans no se integran (el robot recibe otro scan 0.2 s después).
        """
        if abs(w) > self.w_max_mapeo:
            self.dinamicos = np.zeros((0, 2))
            return False
        rangos = np.asarray(rangos, dtype=np.float64)
        ang = th + ang_min + ang_inc * np.arange(rangos.size)
        sx = x + self.off * math.cos(th)
        sy = y + self.off * math.sin(th)
        finito = np.isfinite(rangos)
        valido = (finito & (rangos >= r_min)) | (~finito & (rangos > 0))  # +inf = sin eco
        impacto = finito & (rangos >= r_min) & (rangos < min(r_max, self.alcance))
        r_libre = np.where(finito, np.minimum(rangos, self.alcance), self.alcance)
        r_libre = np.where(valido, r_libre, 0.0)

        # --- espacio libre (todas las celdas atravesadas antes del impacto)
        c, s = np.cos(ang), np.sin(ang)
        t = self._pasos[None, :]
        mask = t < (r_libre[:, None] - self.res)
        px = (sx + c[:, None] * t)[mask]
        py = (sy + s[:, None] * t)[mask]
        ii = ((px - self.ox) / self.res).astype(np.int32)
        jj = ((py - self.oy) / self.res).astype(np.int32)
        ok = (ii >= 0) & (ii < self.n) & (jj >= 0) & (jj < self.n)
        lin = np.unique(ii[ok] * self.n + jj[ok])
        fi, fj = lin // self.n, lin % self.n

        # --- impactos
        hx = sx + c[impacto] * rangos[impacto]
        hy = sy + s[impacto] * rangos[impacto]
        hi = ((hx - self.ox) / self.res).astype(np.int32)
        hj = ((hy - self.oy) / self.res).astype(np.int32)
        ok = (hi >= 0) & (hi < self.n) & (hj >= 0) & (hj < self.n)
        hx, hy, hi, hj = hx[ok], hy[ok], hi[ok], hj[ok]

        # clasificación dinámica ANTES de actualizar: el impacto cae donde
        # históricamente se vio libre muchas veces y NO hay una pared "firme"
        # (muchos impactos acumulados) en la vecindad inmediata.
        firme = self._pared_firme()
        din = ((self.libres[hi, hj] >= self.min_libres_din)
               & (2 * self.impactos[hi, hj] < self.libres[hi, hj])
               & ~firme[hi, hj])
        self.dinamicos = np.stack([hx[din], hy[din]], axis=1) if din.any() else np.zeros((0, 2))

        self.lo[fi, fj] = np.maximum(self.lo[fi, fj] + self.l_free, -self.l_max)
        self.libres[fi, fj] = np.minimum(self.libres[fi, fj] + 1, 1000)
        hl = np.unique(hi * self.n + hj)
        hi2, hj2 = hl // self.n, hl % self.n
        self.lo[hi2, hj2] = np.minimum(self.lo[hi2, hj2] + self.l_occ + abs(self.l_free), self.l_max)
        self.impactos[hi2, hj2] = np.minimum(self.impactos[hi2, hj2] + 1, 30000)
        self._clear_valido = False
        self._version += 1
        return True

    def firme_cache(self):
        """Celdas de pared firme (sin dilatar), recalculadas como mucho 1 vez por scan."""
        if getattr(self, '_firme_version', -1) != self._version:
            self._firme = (self.impactos.astype(np.int32) - self.libres) >= 10
            self._firme_version = self._version
        return self._firme

    def _pared_firme(self):
        """Celdas a <= 3 celdas de algo impactado muchas más veces que visto libre."""
        f = (self.impactos.astype(np.int32) - self.libres) >= 10
        g = f.copy()
        for di in (-3, -2, -1, 1, 2, 3):
            g[max(0, di):self.n + min(0, di), :] |= f[max(0, -di):self.n + min(0, -di), :]
        h = g.copy()
        for dj in (-3, -2, -1, 1, 2, 3):
            h[:, max(0, dj):self.n + min(0, dj)] |= g[:, max(0, -dj):self.n + min(0, -dj)]
        return h

    # ------------------------------------------------------------ consultas
    def ocupado(self):
        return self.lo > self.u_occ

    def libre(self):
        return self.lo < self.u_free

    def clearance(self):
        """Distancia (m) al obstáculo más cercano, saturada en clearance_max."""
        if self._clear_valido:
            return self._clear
        # Lo DESCONOCIDO también cuenta como posible obstáculo: el LiDAR solo ve
        # la cara de un objeto que tiene enfrente (de un cilindro, medio
        # contorno); su interior y su parte de atrás quedan sin observar. Si se
        # ignorara lo desconocido, la distancia al objeto quedaría subestimada.
        occ = ~self.libre()
        # el borde del mapa cuenta como obstáculo
        occ = occ.copy()
        occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
        d = np.full((self.n, self.n), self.cmax, dtype=np.float32)
        d[occ] = 0.0
        pend = ~occ
        for di, dj, dist in self._offs:
            if not pend.any():
                break
            sh = np.zeros_like(occ)
            src = occ[max(0, -di):self.n - max(0, di), max(0, -dj):self.n - max(0, dj)]
            sh[max(0, di):self.n - max(0, -di), max(0, dj):self.n - max(0, -dj)] = src
            hit = pend & sh
            d[hit] = dist
            pend &= ~hit
        self._clear = d
        self._clear_valido = True
        return d

    def clearance_en(self, x, y):
        i, j = self.a_celda(x, y)
        if not self.dentro(i, j):
            return 0.0
        return float(self.clearance()[i, j])

    def segmento_libre(self, x0, y0, x1, y1, r_min):
        """True si todo el segmento tiene clearance >= r_min y es espacio libre conocido."""
        cl = self.clearance()
        lib = self.libre()
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(d / (self.res * 0.5)))
        for k in range(n + 1):
            t = k / n
            i, j = self.a_celda(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if not self.dentro(i, j) or cl[i, j] < r_min or not lib[i, j]:
                return False
        return True
