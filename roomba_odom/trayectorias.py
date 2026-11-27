"""Perfiles (v, w) en lazo abierto para el experimento de odometría (sin ROS).

  recta    5 m en línea recta
  circulo  una vuelta a un círculo de radio R (girando a la izquierda)
  ocho     una vuelta a la izquierda y otra a la derecha: dos círculos tangentes

Cada perfil es una lista de tramos (duración, v, w). El robot vuelve al punto
de partida en el círculo y en el ocho, y acaba en (5, 0) en la recta.
"""
import math

V = 0.25        # m/s
R = 1.0         # m


def perfil(nombre, v=V, radio=R):
    if nombre == 'recta':
        return [(5.0 / v, v, 0.0)]
    w = v / radio
    T = 2.0 * math.pi / w                       # una vuelta
    if nombre == 'circulo':
        return [(T, v, w)]
    if nombre == 'ocho':
        return [(T, v, w), (T, v, -w)]
    raise ValueError(f'trayectoria desconocida: {nombre!r} (recta, circulo, ocho)')


def consigna(tramos, t):
    """(v, w) en el instante t; (0, 0) al terminar."""
    for dur, v, w in tramos:
        if t < dur:
            return v, w
        t -= dur
    return 0.0, 0.0


def duracion(tramos):
    return sum(d for d, _, _ in tramos)
