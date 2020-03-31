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

def create_probabilistic_graph(data, n_sampling_points, k):
    '''
    Graph generation for probabilistic roadmap
    n_sampling_points : sampling n points on random position
    k : number of nearest nodes each node try to connect to
    '''
    sampler = Sampler(data)
    polygons = sampler._polygons

    nodes = sampler.sample(n_sampling_points)

    g = nx.Graph()
    tree = KDTree(nodes)
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
