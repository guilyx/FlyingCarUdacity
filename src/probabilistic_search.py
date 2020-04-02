import networkx as nx
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import numpy as np
from os import path
import sys
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from lib.sampling import Sampler
from lib.probabilistic_roadmap import can_connect, createGraphFromNodes, generateProbabilisticGraph, closest_node
from lib.voronoi_utils import a_star_graph, heuristic
from lib.planning_utils import create_grid

plt.rcParams['figure.figsize'] = 12, 12

def plotProbabilisticPath(data, grid, graph, path):
    fig = plt.figure()

    plt.imshow(grid, cmap='Greys', origin='lower')

    nmin = np.min(data[:, 0])
    emin = np.min(data[:, 1])

    # draw nodes
    for n1 in graph.nodes:
        plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
        
    # draw edges
    for (n1, n2) in graph.edges:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black')

    # draw path
    path_pairs = zip(path[:-1], path[1:])
    for (n1, n2) in path_pairs:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')


    plt.xlabel('NORTH')
    plt.ylabel('EAST')

    plt.show()


def plotGraphEdges(data, grid, graph):
    fig = plt.figure()

    plt.imshow(grid, cmap='Greys', origin='lower')

    nmin = np.min(data[:, 0])
    emin = np.min(data[:, 1])

    # draw edges
    for (n1, n2) in graph.edges:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black' , alpha=0.5)

    # draw all nodes
    for n1 in nodes:
        plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')
        
    # draw connected nodes
    for n1 in graph.nodes:
        plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
        


    plt.xlabel('NORTH')
    plt.ylabel('EAST')

    plt.show()

if __name__ == "__main__":
    filename = 'worlds/colliders.csv'
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

    start = (25., 100., 20.)
    goal = (750., 370., 30.)

    sampler = Sampler(data)
    polygons = sampler._polygons

    n_nodes = 450
    nodes = sampler.sample(n_nodes)

    grid, no, eo = create_grid(data, sampler._zmax, 1)
    graph = createGraphFromNodes(nodes, polygons, 10)

    start_node = list(graph.nodes)[0]
    start_node = closest_node(graph, start)
    k = np.random.randint(len(graph.nodes))
    goal_node = list(graph.nodes)[k]
    goal_node = closest_node(graph, goal)

    path, cost = a_star_graph(graph, start_node, goal_node, heuristic)

    plotProbabilisticPath(data, grid, graph, path)