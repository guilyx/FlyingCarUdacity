import numpy as np
from lib.sampling import Sampler
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree
import networkx as nx

def can_connect(n1, n2, polygons):
    line = LineString([n1, n2])
    for p in polygons:
        if p.crosses(line) and p.height >= min(n1[2], n2[2]):
            return False
    return True

def closest_node(graph, current_position):
    '''
    Compute the closest node in the graph to the current position
    '''
    closest_node = None
    dist = 100000
    xyz_position = (current_position[0], current_position[1], current_position[2])
    for p in graph.nodes:
        d = ((p[0]-xyz_position[0])**2 + (p[1]-xyz_position[1])**2 + (p[2]-xyz_position[2]))
        if d < dist:
            closest_node = p
            dist = d
    return closest_node

def createGraphFromNodes(nodes, polygons_, k):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue
                
            if can_connect(n1, n2, polygons_):
                g.add_edge(n1, n2, weight=1)
    return g

def generateProbabilisticGraph(data, n_sampling_points, k):
    '''
    Graph generation for probabilistic roadmap
    n_sampling_points : sampling n points on random position
    k : number of nearest nodes each node try to connect to
    '''
    print("Generating probabilistic graph...")
    sampler = Sampler(data)
    polygons = sampler._polygons

    nodes = sampler.sample(n_sampling_points)

    g = nx.Graph()
    tree = KDTree(nodes)
    print("Connecting nodes...")
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue
            
            if can_connect(n1, n2, polygons):
                g.add_edge(n1, n2, weight=1)
    return g
