"""Grilla de cobertura del robot: celdas regulares de 25 cm ancladas en el inicio.

No contiene ningún dato del escenario. Es solo una forma de discretizar el
plano para llevar la cuenta de por dónde pasó el robot: la celda (i, j) cubre
[i*0.25, (i+1)*0.25) x [j*0.25, (j+1)*0.25) en el marco de odometría, cuyo
origen es la posición inicial. Qué celdas existen, cuáles son alcanzables y
cuáles son pared lo descubre el robot con el LiDAR.
"""
import math

TAM_CELDA = 0.25


def celda(x, y):
    return (int(math.floor(x / TAM_CELDA)), int(math.floor(y / TAM_CELDA)))


def rectangulo(i, j):
    """(xmin, xmax, ymin, ymax) de la celda."""
    return (i * TAM_CELDA, (i + 1) * TAM_CELDA, j * TAM_CELDA, (j + 1) * TAM_CELDA)


class Cobertura:
    """Celdas visitadas por el centro del robot."""

    def __init__(self):
        self.visitadas = set()

    def actualizar(self, x, y):
        c = celda(x, y)
        if c in self.visitadas:
            return False
        self.visitadas.add(c)
        return True

    def visitada(self, i, j):
        return (i, j) in self.visitadas

    def __len__(self):
        return len(self.visitadas)
