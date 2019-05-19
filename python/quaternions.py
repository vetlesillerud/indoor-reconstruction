import numpy as np


def make_rotation_matrix_from_quaternion(quaternion):
    """
    Convert quaternions to 3x3 rotation matrix.
    Args:
        quaternion (numpy.ndarray): array containing qw, qx, qy and qz

    Returns:
        numpy.ndarray: array of 3x3 rotation matrix
    """
    qw, qx, qy, qz, = quaternion
    rotation_matrix = np.asarray([
        [1 - 2 * qy ** 2 - 2 * qz ** 2, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw],
        [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx ** 2 - 2 * qz ** 2, 2 * qy * qz - 2 * qx * qw],
        [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx ** 2 - 2 * qy ** 2]
    ])
    return rotation_matrix


def invert_quaternion(quaternion):
    """
    Inverts quaternion.
    Args:
        quaternion (numpy.ndarray): array containing qw, qx, qy and qz

    Returns:
        numpy.ndarray: the inverse of input quaternion
    """
    n2 = np.sum(np.power(quaternion, 2))
    qw, qx, qy, qz = quaternion
    quaternion_inverse = qw / n2, -qx / n2, -qy / n2, -qz / n2
    return quaternion_inverse


def conjugate_quaternion(quaternion):
    """
    Converts input quaternions to the conjugate. Same as inverting for unit quaternions.
    Args:
        quaternion (numpy.ndarray): array containing qw, qx, qy and qz

    Returns:
        numpy.ndarray: the conjugate of input quaternions
    """
    qw, qx, qy, qz = quaternion
    quaternion_conjugate = qw, -qx, -qy, -qz
    return quaternion_conjugate


def normalize_quaternion(quaternion):
    """
    Normalizes quaternion.
    Args:
        quaternion (numpy.ndarray): array containing qw, qx, qy and qz

    Returns:
        numpy.array: normalized input quaternion
    """
    n = np.sqrt(np.sum(np.power(quaternion, 2)))
    normalized_quaternion = quaternion / n
    return normalized_quaternion


def multiply_quaternions(quaternion1, quaternion0):
    """
    Multiplies input quaternions: quaternion0*quaternion1
    Args:
        quaternion1 (numpy.ndarray): array containing qw1, qx1, qy1 and qz1
        quaternion0 (numpy.ndarray): array containing qw0, qx0, qy0 and qz0

    Returns:
        numpy.ndarray: the product of input quaternions
    """
    qw0, qx0, qy0, qz0 = quaternion0
    qw1, qx1, qy1, qz1 = quaternion1
    product = np.array([-qx1 * qx0 - qy1 * qy0 - qz1 * qz0 + qw1 * qw0,
                        qx1 * qw0 + qy1 * qz0 - qz1 * qy0 + qw1 * qx0,
                        -qx1 * qz0 + qy1 * qw0 + qz1 * qx0 + qw1 * qy0,
                        qx1 * qy0 - qy1 * qx0 + qz1 * qw0 + qw1 * qz0], dtype=np.float64)
    return product
