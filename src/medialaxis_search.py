import numpy as np 
import matplotlib.pyplot as plt
from skimage.morphology import medial_axis
from skimage.util import invert

from os import path
import sys
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from src.grid_search import generateGrid
from lib.planning_utils import a_star, prunePath, heuristic, create_grid
from lib.medialaxis_utils import create_medial_axis, find_start_goal, back_to_grid

plt.rcParams['figure.figsize'] = 12, 12

def plotMedialAxis(grid, skeleton, start=None, goal=None):
    plt.imshow(grid, origin='lower')
    plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
    
    if start is not None:
        plt.plot(start[1], start[0], 'rx')
    if goal is not None:
        plt.plot(goal[1], goal[0], 'rx')

    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

def plotPath(grid, skeleton, path, path2=None):
    plt.imshow(grid, cmap='Greys', origin='lower')
    plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)

    plt.plot(path[0][1], path[0][0], 'x')
    plt.plot(path[-1][1], path[-1][0], 'x')

    pp = np.array(path)
    plt.plot(pp[:, 1], pp[:, 0], 'g')

    if path2:
        pp2 = np.array(path2)
        plt.plot(pp2[:, 1], pp2[:, 0], 'r')

    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()
 
def planMedialPath(grid, skeleton, start=(218, 371), goal=(419, 520), diagonals=False):
    medial_start, medial_goal = find_start_goal(skeleton, start, goal)
    print("medial_start : ", medial_start, "; medial_target : ", medial_goal)
    medial_axis_grid = back_to_grid(skeleton)
    path, cost = a_star(medial_axis_grid, heuristic, tuple(medial_start), tuple(medial_goal), diagonals)

    return path, cost, medial_start, medial_goal
    

if __name__ == "__main__":
    grid = generateGrid('worlds/colliders.csv', drone_altitude=50, safe_distance=2)
    skeleton = create_medial_axis(grid)
    # plotMedialAxis(grid, skeleton)
    diagonals = True
    path, cost, medial_start, medial_goal = planMedialPath(grid, skeleton)
    path2, cost2 = a_star(grid, heuristic, tuple(medial_start), tuple(medial_goal), diagonals=diagonals)
    
    if path and path2:
        prunned_path = prunePath(path)
        prunned_path2 = prunePath(path2)
        plotPath(grid, skeleton, prunned_path, prunned_path2)
    elif path2:
        prunned_path = prunePath(path2)
        plotPath(grid, skeleton, prunned_path)
    else:
        plotMedialAxis(grid, skeleton, medial_start, medial_goal)