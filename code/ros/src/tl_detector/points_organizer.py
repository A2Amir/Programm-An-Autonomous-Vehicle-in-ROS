import numpy as np
from scipy.spatial import KDTree

class PointsOrganizer(object):
    """
    Auxiliary object that helps to organize 2D points in order to
    optimaze operations to find the closest points
    """

    def __init__(self, points):
        self.points_tree = None
        if points is not None and len(points) > 0:
            self.points_tree = KDTree(points)

    def get_closest_point_idx(self, x, y, look_mode='CLOSEST'):
        """
        Retrieves the closest points in a 2D plane, given one of the following modes:
          - AHEAD
          - BEFORE
          - CLOSEST (can be ahead or before, just considers the distance)
        """
        if self.points_tree is None:
            return None

        closest_idx = self.points_tree.query([x, y], 1)[1]

        # Checking if closest is ahead or behind the reference point
        closest_point = self.points_tree.data[closest_idx]
        prev_idx = (closest_idx - 1) if closest_idx > 0 else (len(self.points_tree.data) - 1)
        prev_point = self.points_tree.data[prev_idx]

        # Equation for hyperplane through closest coords
        cl_vect = np.array(closest_point)
        prev_vect = np.array(prev_point)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if look_mode == 'AHEAD' and val > 0:
            closest_idx = (closest_idx + 1) % len(self.points_tree.data)
        elif look_mode == 'BEFORE' and val < 0:
            closest_idx = prev_idx

        return closest_idx
