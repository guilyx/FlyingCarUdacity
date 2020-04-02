import numpy as np 
import matplotlib.pyplot as plt
import networkx as nx

from os import path
import sys
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from lib.planning_utils import prunePath
from lib.voronoi_utils import a_star_graph, closest_node, create_grid_and_edges, create_graph, heuristic

def plotGridEdges(grid, edges):
    plt.imshow(grid, origin='lower', cmap='Greys') 

    for e in edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

def plotVoronoiPath(grid, edges, path, start, goal, start_graph, goal_graph):
    plt.imshow(grid, origin='lower', cmap='Greys') 

    for e in edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
        
    plt.plot([start[1], start_graph[1]], [start[0], start_graph[0]], 'r-')
    for i in range(len(path)-1):
        p1 = path[i]
        p2 = path[i+1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
    plt.plot([goal[1], goal_graph[1]], [goal[0], goal_graph[0]], 'r-')
        
    plt.plot(start[1], start[0], 'gx')
    plt.plot(goal[1], goal[0], 'gx')

    plt.xlabel('EAST', fontsize=20)
    plt.ylabel('NORTH', fontsize=20)
    plt.show()

if __name__ == "__main__":
    filename = 'worlds/colliders.csv'
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

    grid, edges, no, eo = create_grid_and_edges(data, drone_altitude=5, safety_distance=2)
    
    start = (25, 100)
    goal = (750, 370)

    graph = create_graph(edges)

    start_graph = closest_node(graph, start)
    goal_graph = closest_node(graph, goal)

    path, cost = a_star_graph(graph, start_graph, goal_graph, heuristic)

    plotVoronoiPath(grid, edges, path, start, goal, start_graph, goal_graph)

