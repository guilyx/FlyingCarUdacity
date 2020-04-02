import numpy as np
import matplotlib.pyplot as plt

from os import path
import sys
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from lib.planning_utils import create_grid, a_star, prunePath, heuristic

def plotGrid(grid):
    plt.rcParams['figure.figsize'] = 12, 12
    plt.imshow(grid, origin='lower')
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

def plotPath(grid, path):
    plt.imshow(grid, cmap='Greys', origin='lower')
    plt.plot(path[0][1], path[0][0], 'x')
    plt.plot(path[-1][1], path[-1][0], 'x')

    pp = np.array(path)
    plt.plot(pp[:,1], pp[:, 0], 'g')
    plt.scatter(pp[:, 1], pp[:, 0])
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

def generateGrid(filename, drone_altitude=5, safe_distance=5):
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
    grid, northOffset, eastOffset = create_grid(data, drone_altitude, safe_distance)
    return grid

def planPath(grid, start=(25, 100), goal=(650, 500), diagonals=False):
    path, cost = a_star(grid, heuristic, start, goal, diagonals)
    return path, cost

if __name__ == "__main__":
    grid = generateGrid('worlds/colliders.csv')
    path, cost = planPath(grid, diagonals=True)
    prunned_path = prunePath(path)
    plotPath(grid, prunned_path)