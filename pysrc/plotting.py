import numpy as np


def position_bounding_box(data_dim1, data_dim2):

    # find the bounding box of two-dimensional data

    dim1_max = np.max(data_dim1)
    dim1_min = np.min(data_dim1)
    dim2_max = np.max(data_dim2)
    dim2_min = np.min(data_dim2)

    return np.array([dim1_min, dim2_min]), np.array([dim1_max, dim2_max])
