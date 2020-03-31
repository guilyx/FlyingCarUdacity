import numpy as np
from skimage.morphology import medial_axis
from skimage.util import invert


def create_medial_axis(grid):
    return medial_axis(invert(grid))


def find_start_goal(skel, start, goal):
    # return position of start and goal on the nearest medial axis
    skel_cells = np.transpose(skel.nonzero())

    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]

    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    return near_start, near_goal


def back_to_grid(skel):
    return(invert(skel).astype(np.int))
