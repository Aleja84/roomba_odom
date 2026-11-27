"""Cinemática del robot diferencial e integración de la odometría (sin ROS).

Se puede importar desde un script normal (ver tools/odometria_offline.py).

Convenio (Klančar, cap. 2):
  r  radio de rueda, L separación entre ruedas
  v = r (w_R + w_L) / 2          w = r (w_R - w_L) / L        (directa)
  w_R = (v + w L/2) / r          w_L = (v - w L/2) / r        (inversa)

Integración de un periodo a partir de los incrementos de encoder:
  ds = r (dth_R + dth_L) / 2     dphi = r (dth_R - dth_L) / L
"""
import math

METODOS = ('euler', 'rk2', 'exact')


def wrap(a):
    """Normaliza un ángulo a (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def inversa(v, w, r, L):
    """(v, w) del robot -> (w_R, w_L) en rad/s."""
    return (v + w * L / 2.0) / r, (v - w * L / 2.0) / r


def directa(w_R, w_L, r, L):
    """(w_R, w_L) en rad/s -> (v, w) del robot."""
    return r * (w_R + w_L) / 2.0, r * (w_R - w_L) / L


def incrementos(dth_R, dth_L, r, L):
    """Giros de rueda en un periodo [rad] -> (ds, dphi)."""
    return r * (dth_R + dth_L) / 2.0, r * (dth_R - dth_L) / L


def paso(x, y, phi, ds, dphi, metodo):
    """Integra un periodo suponiendo v y w constantes en él.

    euler : rumbo al principio del periodo (ec. 2.5)
    rk2   : rumbo a mitad del periodo, phi + dphi/2 (ec. 2.6, «trapezoidal»)
    exact : arco de circunferencia (ec. 2.7). Con dphi -> 0 el radio ds/dphi
            se va a infinito, así que por debajo de 1e-6 se usa rk2, que en ese
            límite coincide con la exacta.
    La orientación se actualiza igual en los tres: phi += dphi.
    """
    if metodo == 'euler':
        x += ds * math.cos(phi)
        y += ds * math.sin(phi)
    elif metodo == 'rk2' or (metodo == 'exact' and abs(dphi) < 1e-6):
        m = phi + 0.5 * dphi
        x += ds * math.cos(m)
        y += ds * math.sin(m)
    elif metodo == 'exact':
        R = ds / dphi
        x += R * (math.sin(phi + dphi) - math.sin(phi))
        y -= R * (math.cos(phi + dphi) - math.cos(phi))
    else:
        raise ValueError(f'metodo desconocido: {metodo!r} (usa {METODOS})')
    return x, y, wrap(phi + dphi)
