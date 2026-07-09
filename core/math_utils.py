"""Math helpers for robot poses, angles, and TCP-local movement."""

import math
from core.helpers import add_vectors


def wrap_angle_deg(angle: float) -> float:
    """Wrap an angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def angle_error_deg(measured: float, target: float) -> float:
    return wrap_angle_deg(float(measured) - float(target))


def rpy_deg_to_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> list[list[float]]:
    """
    Convert RX/RY/RZ degrees to a rotation matrix.

    Convention:
        R = Rz * Ry * Rx

    This is used for converting TCP-local XYZ deltas to global/base deltas.
    """
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ]


def tcp_delta_to_global_delta(current_pose: list[float], tcp_delta: list[float]) -> list[float]:
    dx_tcp, dy_tcp, dz_tcp, drx, dry, drz = tcp_delta
    rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]

    r = rpy_deg_to_matrix(rx, ry, rz)

    dx_global = r[0][0] * dx_tcp + r[0][1] * dy_tcp + r[0][2] * dz_tcp
    dy_global = r[1][0] * dx_tcp + r[1][1] * dy_tcp + r[1][2] * dz_tcp
    dz_global = r[2][0] * dx_tcp + r[2][1] * dy_tcp + r[2][2] * dz_tcp

    return [dx_global, dy_global, dz_global, drx, dry, drz]


def tcp_delta_to_pose(current_pose: list[float], tcp_delta: list[float]) -> list[float]:
    return add_vectors(current_pose, tcp_delta_to_global_delta(current_pose, tcp_delta))
