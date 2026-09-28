"""Planificador de cobertura (capa de decisión).

El robot no conoce el escenario de antemano. Lleva la cuenta de su cobertura
en una grilla regular de 25 cm anclada en su posición inicial, y en cada
decisión mira el mapa que va construyendo con el LiDAR:

1. Una celda de la grilla es CANDIDATA si, según lo descubierto hasta ahora,
   contiene un punto libre con clearance suficiente (el cuerpo del robot entra)
   y un poco adentro del borde (para que la celda se cuente con seguridad).
   A medida que el LiDAR descubre más escenario, aparecen nuevas candidatas.
2. Dijkstra de una sola fuente desde el robot sobre el mapa de 5 cm (costo
   extra cerca de paredes) -> distancia real a todas las candidatas.
3. Elige la candidata de menor costo = distancia + penalización de giro
   - premio por "borde" (vecinos ya visitados o sin lugar posible). Ese premio
   produce un recorrido tipo barrido, sin huecos aislados que obliguen a volver.
4. Reconstruye el camino y lo simplifica por línea de vista.

La misión termina cuando no queda ninguna candidata alcanzable.
"""
import heapq
import math
import time

import numpy as np

from .grilla_cobertura import TAM_CELDA


def _ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Planificador:
    def __init__(self, mapa, r_paso=0.16, r_objetivo=0.16, margen_celda=0.03,
                 peso_giro=0.25, peso_borde=0.12, peso_pared=2.0,
                 max_fallos=3, espera_fallo=25.0):
        self.mapa = mapa
        self.r_paso = r_paso
        self.r_obj = r_objetivo
        self.margen = margen_celda
        self.w_giro = peso_giro
        self.w_borde = peso_borde
        self.w_pared = peso_pared
        self.max_fallos = max_fallos
        self.espera_fallo = espera_fallo
        self.fallos = {}      # celda -> cantidad de fallos
        self.bloqueo = {}     # celda -> instante hasta el que no se intenta
        # relación entre el mapa de 5 cm y la grilla de 25 cm
        self.k = int(round(TAM_CELDA / mapa.res))
        off_x = mapa.ox / TAM_CELDA
        off_y = mapa.oy / TAM_CELDA
        if abs(off_x - round(off_x)) > 1e-6 or abs(off_y - round(off_y)) > 1e-6 or \
                abs(TAM_CELDA / mapa.res - self.k) > 1e-6 or mapa.n % self.k:
            raise ValueError('el mapa debe estar alineado con la grilla de cobertura')
        self.i0, self.j0 = int(round(off_x)), int(round(off_y))
        self.m = mapa.n // self.k   # celdas de cobertura por lado

    def _mascara_margen(self):
        """Celdas del mapa cuyo centro está al menos `margen` adentro de su celda de 25 cm."""
        off = (np.arange(self.mapa.n) % self.k + 0.5) * self.mapa.res
        ok = (off >= self.margen) & (off <= TAM_CELDA - self.margen)
        return ok[:, None] & ok[None, :]

    def candidatas(self, clear, libre):
        """dict {(i, j) celda de cobertura: (a, b) mejor celda del mapa} según lo descubierto."""
        valido = libre & (clear >= self.r_obj) & self._mascara_margen()
        m, k = self.m, self.k
        c = np.where(valido, clear, -1.0).reshape(m, k, m, k).transpose(0, 2, 1, 3).reshape(m, m, k * k)
        mejor = c.argmax(axis=2)
        val = np.take_along_axis(c, mejor[..., None], axis=2)[..., 0]
        res = {}
        for I, J in zip(*np.nonzero(val > 0)):
            a = I * k + mejor[I, J] // k
            b = J * k + mejor[I, J] % k
            res[(int(I) + self.i0, int(J) + self.j0)] = (int(a), int(b))
        return res

    def registrar_fallo(self, rc, ahora=None):
        ahora = time.monotonic() if ahora is None else ahora
        self.fallos[rc] = self.fallos.get(rc, 0) + 1
        self.bloqueo[rc] = ahora + self.espera_fallo

    def descartada(self, rc):
        return self.fallos.get(rc, 0) >= self.max_fallos

    # --------------------------------------------------------------------
    def _dijkstra(self, inicio, transitable, clear):
        n = self.mapa.n
        dist = np.full((n, n), np.inf)
        padre = np.full((n, n, 2), -1, dtype=np.int32)
        dist[inicio] = 0.0
        pq = [(0.0, inicio)]
        vecinos = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                   (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        res = self.mapa.res
        while pq:
            d, (i, j) = heapq.heappop(pq)
            if d > dist[i, j]:
                continue
            for di, dj, w in vecinos:
                a, b = i + di, j + dj
                if not (0 <= a < n and 0 <= b < n) or not transitable[a, b]:
                    continue
                # penaliza pasar cerca de paredes -> caminos por el centro de pasillos
                pen = 1.0 + self.w_pared * max(0.0, 0.35 - clear[a, b]) / 0.35
                nd = d + w * res * pen
                if nd < dist[a, b]:
                    dist[a, b] = nd
                    padre[a, b] = (i, j)
                    heapq.heappush(pq, (nd, (a, b)))
        return dist, padre

    def _inicio(self, x, y, transitable):
        """Celda de arranque: la del robot, o la transitable más cercana (<= 0.25 m)."""
        i, j = self.mapa.a_celda(x, y)
        if self.mapa.dentro(i, j) and transitable[i, j]:
            return (i, j)
        r = int(0.25 / self.mapa.res)
        mejor, md = None, 1e9
        for a in range(i - r, i + r + 1):
            for b in range(j - r, j + r + 1):
                if self.mapa.dentro(a, b) and transitable[a, b]:
                    d = (a - i) ** 2 + (b - j) ** 2
                    if d < md:
                        mejor, md = (a, b), d
        return mejor

    def _borde(self, rc, pendientes):
        """Cuántos de los 4 vecinos de rc NO están pendientes (hechos, pared, fuera)."""
        f, c = rc
        k = 0
        for df, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if (f + df, c + dc) not in pendientes:
                k += 1
        return k

    def planificar(self, x, y, th, cobertura, ahora=None, excluir=()):
        """Devuelve (celda, [waypoints (x, y)]) o (None, []) si no queda nada alcanzable."""
        ahora = time.monotonic() if ahora is None else ahora
        clear = self.mapa.clearance()
        libre = self.mapa.libre()
        transitable = libre & (clear >= self.r_paso)
        inicio = self._inicio(x, y, transitable)
        if inicio is None:
            return None, []
        dist, padre = self._dijkstra(inicio, transitable, clear)

        cand = self.candidatas(clear, libre)
        pendientes = {rc for rc in cand
                      if not cobertura.visitada(*rc) and not self.descartada(rc)}
        mejor, mejor_costo, mejor_pt = None, float('inf'), None
        for rc in pendientes:
            if rc in excluir or self.bloqueo.get(rc, 0.0) > ahora:
                continue
            pt = cand[rc]
            if not np.isfinite(dist[pt]):
                continue
            px, py = self.mapa.a_mundo(*pt)
            giro = abs(_ang(math.atan2(py - y, px - x) - th))
            costo = dist[pt] + self.w_giro * giro - self.w_borde * self._borde(rc, pendientes)
            if costo < mejor_costo:
                mejor, mejor_costo, mejor_pt = rc, costo, pt
        if mejor is None:
            return None, []

        # reconstrucción del camino
        camino = [mejor_pt]
        cur = mejor_pt
        while cur != inicio:
            p = tuple(padre[cur])
            if p[0] < 0:
                break
            camino.append(p)
            cur = p
        camino.reverse()
        pts = [self.mapa.a_mundo(i, j) for i, j in camino]
        return mejor, self._simplificar((x, y), pts)

    def _simplificar(self, origen, pts):
        """Poda el camino por línea de vista (menos waypoints = marcha más suave)."""
        if not pts:
            return []
        res, ancla, k = [], origen, 0
        while k < len(pts):
            lejos = k
            for m in range(len(pts) - 1, k - 1, -1):
                if self.mapa.segmento_libre(ancla[0], ancla[1], pts[m][0], pts[m][1],
                                            self.r_paso * 0.9):
                    lejos = m
                    break
            res.append(pts[lejos])
            ancla = pts[lejos]
            k = lejos + 1
        return res

