"""Pruebas de la lógica sin ROS:  cd /ros2_ws/src/tb3_cobertura && python3 -m pytest -q test"""
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'herramientas'))

from tb3_cobertura import escenario_verdad as ev  # noqa: E402
from tb3_cobertura.grilla_cobertura import Cobertura, celda, rectangulo  # noqa: E402

PAQ = os.path.join(os.path.dirname(__file__), '..', 'tb3_cobertura')


def test_el_robot_no_tiene_datos_del_escenario():
    """Ningún módulo que usa el robot importa la verdad del escenario ni la matriz guía."""
    for f in ('planificador.py', 'mision.py', 'mapa_ocupacion.py', 'fusion.py',
              'grilla_cobertura.py', 'nodo_navegador.py'):
        codigo = open(os.path.join(PAQ, f)).read()
        assert not re.search(r'escenario_verdad|grilla_evaluacion|MAPA_REFERENCIA', codigo), f


def test_grilla_regular():
    assert celda(0.0, 0.0) == (0, 0)
    assert celda(-0.01, 0.26) == (-1, 1)
    x0, x1, y0, y1 = rectangulo(-1, 1)
    assert (x0, x1, y0, y1) == (-0.25, 0.0, 0.25, 0.5)
    c = Cobertura()
    assert c.actualizar(0.1, 0.1) and not c.actualizar(0.2, 0.2) and len(c) == 1


def test_juez_celdas_recorribles():
    import sim2d
    o = sim2d.obstaculos_juez(con_cilindros=True)
    rec = ev.celdas_recorribles(o, radio_robot=0.13)
    assert 300 <= len(rec) <= 330          # escenario stage4 con grilla de 25 cm
    assert celda(0.1, 0.1) in rec          # el inicio es recorrible
    assert celda(2.39, 0.0) not in rec     # pegado a la pared exterior no


def test_mision_sin_obstaculos_sin_contactos():
    import sim2d
    res = sim2d.simular(t_total=250.0, obstaculos=False, verbose=False)
    assert res['contactos'] == 0
    assert res['metrica_real'].porcentaje() > 30.0
