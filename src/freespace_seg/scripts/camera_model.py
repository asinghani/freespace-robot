import numpy as np

class CameraConfig:
    def __init__(self, cx, cy, fx, fy, height):
        self.cx = cx
        self.cy = cy

        self.fx = fx
        self.fy = fy

        self.height = height

    @classmethod
    def FromCameraMatrix(cls, matrix, height):
        fx = float(matrix[0, 0])
        fy = float(matrix[1, 1])
        cx = float(matrix[0, 2])
        cy = float(matrix[1, 2])
        return cls(cx, cy, fx, fy, height)

    @classmethod
    def Identity(cls, height):
        fx = 1.0
        fy = 1.0
        cx = 0.0
        cy = 0.0
        return cls(cx, cy, fx, fy, height)


def multi_raycast(coords, config):
    """
    Given a list of coordinate pairs (u, v) and a CameraConfig object, returns a list of vectors that go through the given points and end at ground level
    """

    c = np.array(coords, dtype=np.float32)

    z_values = np.array([[1.0] * c.shape[0]], dtype=np.float32).T

    rays = (c - np.array([config.cx, config.cy], dtype=np.float32)) / np.array([config.fx, config.fy], dtype=np.float32)

    rays = np.concatenate((rays, z_values), axis=1)

    rays_scaled = (rays.T / rays[:, 1]).T * -config.height

    return rays_scaled

def get_segmentation_border(seg, every_n=1):
    """
    Given a 2D segmentation mask containing 0 = obstacle and 1 = freespace, get the list of coordinates marking the "nearest" non-freespace region
    """

    seg = 1 - seg
    mask = (seg != 0)
    height = seg.shape[0] - np.flip(mask, axis=0).argmax(axis=0) - 1
    coords = np.where(mask.any(axis=0), height, 0) + 1

    coords = coords.reshape((len(coords), 1))
    x_coords = np.arange(0, len(coords))
    x_coords = x_coords.reshape((len(x_coords), 1))

    coords = np.concatenate((x_coords, coords), axis=1)

    coords = coords[::every_n]

    return coords
