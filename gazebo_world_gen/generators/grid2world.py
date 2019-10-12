# -*- coding: utf-8 -*-
from __future__ import print_function, absolute_import
from tempfile import NamedTemporaryFile, mkdtemp
from generators.templates import WORLD_TEMPLATE_START, WORLD_TEMPLATE_END

class WorldGen:
    def __init__(self, fdir, grid, goal_pos, subj_pos, drone_pos, peds_paths=None, tree_pos=None, door_pos=None, wall_pos=None):
        """
        Creates a .world file with the given parameters for objects and
        pedestrians. Pedestrians and objects are optional.

        :param grid: A 2d numpy array of dtype str with NxN shape
        :param goal_pos: A tuple (x,y)
        :param subj_pos: A tuple (x,y)
        :param drone_pos: A tuple (x,y)
        :param peds_paths: A dictionary with K-V {PedX: {pos: [(x0,y0),...], loop:True/False}}
        :param tree_pos: A list of (x,y) points
        :param door_pos: A list of (x,y, dir) points
        :param wall_pos: A list of (x,y) points
        """
        self.f = NamedTemporaryFile(mode="w", suffix=".world", dir=fdir, delete=False)
        
        self.goal_pos = WorldGen.world_pose(goal_pos[0], goal_pos[1])  # "paint it" in red
        self.subj_pos = WorldGen.world_pose(subj_pos[0], subj_pos[1])  # "paint it" in green
        self.drone_pos = WorldGen.world_pose(drone_pos[0], drone_pos[1], 0.1)

        if peds_paths is not None:
            self.peds_poses = []
            for opts in peds_paths.values():
                # opts has "pos" and "loop"
                # pos is (poses, row/col, list of x/y points)
                init_pos = opts["pos"][0][0]
                end_pos = opts["pos"][0][-1]
                self.peds_poses.append(WorldGen.world_pose(init_pos[0], init_pos[1]))
                self.peds_poses.append(WorldGen.world_pose(end_pos[0], end_pos[1]))
        
        # By default, 'None or []' is []
        self.tree_pos = tree_pos or []
        self.door_pos = door_pos or []
        self.wall_pos = wall_pos or []
            
        self.write_to_template()
        
    @staticmethod
    def world_pose(x, y, z=0.0):
        return "{} {} {} 0 0 0".format(x, y, z)
    
    def _object_tag(self, name, model, pose):
        """Returns an object 'include' tag"""

        return """
        <include>
          <name>{}</name>
          <uri>model://{}</uri>
          <pose>{}</pose>
        </include>
        """.format(name, model, pose)

    def _tree_tag(self, name, point):
        """"""
        canopy_pos = WorldGen.world_pose(point[0], point[1], 2.5)
        canopy_nm = "{}_canopy".format(name)
        wood_pos = WorldGen.world_pose(point[0], point[1])
        wood_nm = "{}_wood".format(name)
        
        tag = []
        tag.append(self._object_tag(canopy_nm, "tree_canopy", canopy_pos))
        tag.append(self._object_tag(wood_nm, "vertical_bar", wood_pos))
        return "\n".join(tag)
        
    def _door_tag(self, name, center):
        door_center = WorldGen.world_pose(center[0], center[1], 2.5)
        bar1 = WorldGen.world_pose(center[0] - 0.6, center[1])
        bar2 = WorldGen.world_pose(center[0] + 0.6, center[1])
        
        tag = []
        tag.append(self._object_tag("{}_rail".format(name), "top_door_rail", door_center))
        tag.append(self._object_tag("{}_bar1".format(name), "vertical_bar", bar1))
        tag.append(self._object_tag("{}_bar2".format(name), "vertical_bar", bar2))
        return "\n".join(tag)
    
    def _wall_tag(self, name, points):
        tag = []
        for (i, point) in enumerate(points):
            nm = "{}{}".format(name, i)
            box_pose = WorldGen.world_pose(point[0], point[1])
            tag.append(self._object_tag(nm, "wall_box", box_pose))
            
        return "\n".join(tag)
            
    def write_to_template(self):
        """"""
        includes = []

        includes.append(
            self._object_tag("start", "ground_red", self.subj_pos)
        )
        includes.append(
            self._object_tag("goal", "ground_goal", self.goal_pos)
        )
        
        for (i, point) in enumerate(self.peds_poses):
            includes.append(self._object_tag("PX{}".format(i), "ground_tour", point))
        for (i, point) in enumerate(self.tree_pos):
            includes.append(self._tree_tag("T{}".format(i), point))
            
        for (i, point) in enumerate(self.door_pos):
            includes.append(self._door_tag("D{}".format(i), point))
            
        for (i, points) in enumerate(self.wall_pos):
            # for each wall, being a wall a list of points
            includes.append(self._wall_tag("W{}".format(i), points))
        
        objs_includes = "\n".join(includes)

        template = WORLD_TEMPLATE_START.format(self.drone_pos)
        temp_txt = template + objs_includes + WORLD_TEMPLATE_END
        self.f.write(temp_txt)
        self.f.close()
        
    def get_path(self):
        return self.f.name
