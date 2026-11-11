import numpy as np

from twist2_vr.providers.openvr import _matrix34


class Matrix:
    def __init__(self):
        self.m = [[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3]]


def test_openvr_matrix_conversion():
    position, quat = _matrix34(Matrix())
    np.testing.assert_allclose(position, [1, 2, 3])
    np.testing.assert_allclose(quat, [0, 0, 0, 1])
