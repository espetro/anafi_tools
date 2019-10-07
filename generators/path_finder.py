from __future__ import print_function

class PathNode:
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

# =====================

class PathFinder:
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
        """Gets the set of surrounding tiles given a tile and its set radius"""
        
        max_X, max_Y = grid.shape
        my_X, my_Y = curr_node.pos
        # Drops the 4 diagonal surrounding tiles
        area = list(map(lambda (a,b): (a + my_X, b + my_Y), [(-1,0), (1,0), (0,-1), (0,1)]))

        within_range = (p for p in area if (p[0] in range(max_X) and p[1] in range(max_Y)))
        cell_types = (PathFinder.check_type(grid, cell) for cell in within_range)
        return [PathNode(curr_node, pos, ctype, weight) for (pos, walkable, weight, ctype) in cell_types if walkable]
        
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
                "T": (cell, True, 0, "T"),
                "D": (cell, True, 2, "D"),
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
    
    def a_star(self):
        """Runs the A* search for the grid setup"""
        node0 = PathNode(None, self.init, cell_type="S")
        nodeX = PathNode(None, self.end, cell_type="G")
        
        opened, closed = [node0], []
        
        while len(opened) > 0:
            # print("[0]: {} | [C]: {}".format(len(opened), len(closed)))
            # Get the node with smallest F cost
            costs = map(lambda p: p.f, opened)
            curr_node, curr_idx = [(n,i) for i, (n, f) in enumerate(zip(opened, costs)) if f == min(costs)][0]
            
            # Switch it to 'closed'
            closed.append(curr_node)
            opened.pop(curr_idx)
            
            if curr_node == nodeX:
                # The path has been found!
                # print(curr_node)
                
                path = PathFinder.get_path_from(curr_node)
                # print("A path has been found!\n{}".format(self.path))
                return path
                
            # Looks at the neighbor cells who are within grid range and are walkable (no walls).
            # Also cast each into a node
            block = PathFinder.walkable_node_neighbors(self.grid, curr_node, rad=1)
            
            # If the cell is not already closed and is walkable, proceed
            for node in block:
                if node not in closed:
                    node.g = curr_node.g + 1
                    # Euclidean distance used as metric (+ cell type weight)
                    node.h = sum([(a - b)**2 for (a,b) in zip(node.pos, curr_node.pos)]) + node.weight
                    node.f = node.g + node.h
                    
                    # If the node is in the "open" list and its new weight
                    # is more than the prev one, discard it
                    # here's the opposed case: there's no node like that
                    if [nopen for nopen in opened if (nopen == node and node.g > nopen.g)] == []:                            
                        opened.append(node)
        return []

if __name__ == "__main__":
    # from random_world import RandomWorld
    # world = RandomWorld.load_from("example_grid.pk")
    import pickle as pk

    with open("example_grid.pk", "rb") as f:
        world = pk.load(f)

    (grid, goal_pos, subj_pos, drone_pos, peds, trees, doors, walls) = world

    path = PathFinder(grid, subj_pos, goal_pos)
    print(path.a_star())