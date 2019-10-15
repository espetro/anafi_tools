from __future__ import print_function, absolute_import
from random import random
from time import time

# a clear improvement would be to use IDA* from the pathfinding package
# https://github.com/brean/python-pathfinding

class PathNode:
    """A node structure used for A* path finding"""
    def __init__(self, parent=None, position=None, cell_type="", weight=0):
        self.parent = parent
        self.pos = position
        self.weight = weight
        self.cell_type = cell_type
        
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.pos == other.pos
    
    def __repr__(self):
        return "Node({}, {}, {})".format(self.parent, self.pos, self.f)
    
    def __str__(self):
        return self.__repr__()

    def __hash__(self):
        """Returns an unique hash for each object instance. Useful for set hashing"""
        return hash(repr(self))

# =====================

class PathFinder:
    """A class to wrap an A* path-finding algorithm"""
    def __init__(self, grid, init, end):
        """
        :param grid:
        :param init:
        :param end:
        """
        self.grid = grid
        self.init = init
        self.end = end
        self.path = None
        
        # self.a_star()
        
    @staticmethod
    def walkable_node_neighbors(grid, curr_node, rad=1):
        """
        Gets the set of surrounding tiles given a tile and its set radius.
        Drops the 4 diagonal surrounding tiles so not to move in diagonal.
        """
        
        max_X, max_Y = grid.shape
        my_X, my_Y = curr_node.pos
        area = [(a + my_X, b + my_Y) for (a,b) in [(-1,0), (1,0), (0,-1), (0,1)]]

        within_range = (
            p for p in area if (p[0] in range(max_X) and p[1] in range(max_Y))
        )
        cell_types = (
            PathFinder.check_type(grid, cell) for cell in within_range
        )

        return [
            PathNode(curr_node, pos, ctype, weight) for
                (pos, walkable, weight, ctype) in cell_types if walkable
        ]
        
    @staticmethod
    def check_type(grid, cell):
        """
        Gets information about the cell in (x,y). Walls and start pose are not
        traversable.
        
        :param cell: A tuple (x,y)
        :returns: A tuple (is_traversable, weight)
        """
        if "P" in grid[cell]:
            cell_info = (cell, True, 0, "P")
        else:
            cell_info = {
                "T": (cell, True, 0.5, "T"),
                "D": (cell, True, 1.5, "D"),
                "S": (cell, False, None, "S"),
                "G": (cell, True, 0, "G"),
                "": (cell, True, 0, ""),
                "o": (cell, True, 0, "o"),
                "W": (cell, False, None, "W")
            }.get(grid[cell])
        
        return cell_info
    
    @staticmethod
    def get_path_from(node):
        """Computes the upward path from a leaf node."""
        # print(node)
        path = [(node.pos, node.cell_type)]
        curr_node = node.parent
        while curr_node is not None:
            # print(curr_node)
            path.append((curr_node.pos, curr_node.cell_type))
            curr_node = curr_node.parent
            
        return path[::-1]  # same as .reverse()
    
    @staticmethod
    def heuristic_mhd(ptA, ptB):
        """Uses the Manhattan distance between two 2D points as heuristic"""
        return sum([abs(a-b) for (a,b) in zip(ptA, ptB)])

    @staticmethod
    def heuristic_ecd(ptA, ptB):
        """Uses the Euclidean distance between two 2D points as heuristic"""
        return sum([(a-b) ** 2 for (a,b) in zip(ptA, ptB)])

    @staticmethod
    def heuristic_chv(ptA, ptB):
        """Uses the Chebyshev distance between two 2D points as heuristic"""
        return max([(a-b) for (a,b) in zip(ptA, ptB)])

    def a_star(self, time_limit):
        """Runs the A* search for the grid setup"""
        tstart = time()

        node0 = PathNode(None, self.init, cell_type="S")
        nodeX = PathNode(None, self.end, cell_type="G")
        
        opened, closed = list(), list()
        opened.append(node0)
        
        tdiff = time() - tstart

        # Get the node with smallest F cost
        while (time_limit >= tdiff) and len(opened) > 0:
            if random() > 0.999:
                print("\r", tdiff)

            if random() > 0.99999:
                print("\n\n\nOpened set: ", opened)
                print("\n\n\nClosed set: ", closed)

            minimal_cost = min([n.f for n in opened])
            curr_node = [node for node in opened if node.f == minimal_cost][0]
            
            # Switch it to 'closed'
            try:
                opened.remove(curr_node)
                closed.append(curr_node)
            except ValueError:
                print("{} not in opened".format(curr_node))
            
            if curr_node == nodeX:
                # The path has been found
                path = PathFinder.get_path_from(curr_node)
                print("The path has been found: ", path)
                return path
                
            # Create new neighbor cell nodes and check if they're walkable (in range, no walls)
            block = PathFinder.walkable_node_neighbors(self.grid, curr_node, rad=1)
            
            # If the cell is not already closed and is walkable, proceed
            for node in block:
                if node not in closed:
                    node.g = curr_node.g + 1
                    # Euclidean distance used as metric (+ cell type weight)
                    node.h = node.weight + PathFinder.heuristic_mhd(
                        node.pos, curr_node.pos
                    )
                    node.f = node.g + node.h
                    
                    # If the node is in the "open" list and its new weight
                    # is more than the prev one, discard it
                    # here's the opposed case: there's no node like that
                    if [nopen for nopen in opened if (nopen == node and node.g > nopen.g)] == []:
                        opened.append(node)

            tdiff = time() - tstart

        return []
