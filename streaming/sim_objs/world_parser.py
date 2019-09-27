#!/usr/bin/env python
from __future__ import print_function

# from streaming.sim_objs.simple_wall import SimpleWallShape
# from streaming.sim_objs.top_door_rail import TopDoorRailShape
# from streaming.sim_objs.vbar import VerticalBarShape
# from streaming.sim_objs.tree_canopy import TreeCanopyShape
from simple_wall import SimpleWallShape
from top_door_rail import TopDoorRailShape
from vbar import VerticalBarShape
from tree_canopy import TreeCanopyShape

import xml.etree.ElementTree as ET
import pickle as pk


class WorldParser(object):

    def __init__(self, fpath):
        """
        Parses a .world file to get the information about static world objects
        
        :param fpath: The absolute path of the .world file
        """
        (self.objects, _) = WorldParser._parse_tags(fpath)
        self.models = []
        self._match()
        
    @staticmethod
    def _as_dict(model):
        """
        Transforms a list of XML tags in a dictionary
        :param model: A list of XML .world tags
        :returns: A dictionary of the .world tags
        """
        d = dict()
        for tag in model:
            d[tag.tag] = tag.text
        return d

    @staticmethod
    def _parse_tags(fpath):
        """
        Extract the world object tags 'include' as dictionaries. Object tags
        have more than one sub-tag, including the 'pose' tag.
        :param fpath:
        :returns:
        """
        world = ET.parse(fpath).getroot().find("world")
        tags = [tag for tag in world.getchildren() if tag.tag == "include" and tag.find("pose") is not None]
        objects = [WorldParser._as_dict(obj.getchildren()) for obj in tags]
        return (objects, tags)

    def _match(self):
        """
        Matches the extracted object tags with their equivalent model shapes.
        :returns: A list of Gazebo models as Python objects.
        """

        for m in self.objects:
            _pose = m["pose"].split(" ")
            model = {
                SimpleWallShape.model: SimpleWallShape(m["name"], _pose[:3], _pose[3:]),
                TopDoorRailShape.model: TopDoorRailShape(m["name"], _pose[:3], _pose[3:]),
                VerticalBarShape.model: VerticalBarShape(m["name"], _pose[:3]),
                TreeCanopyShape.model: TreeCanopyShape(m["name"], _pose[:3]),
            }.get(m["uri"], None)

            if model is not None:
                self.models.append(model)


    # def write_to_file(self, fpath):
    #     with open(fpath, "w") as f:
    #         for o in self.objs:
    #             f.write("{} {}".format(o.pose, o.volume))

    # def save_to_pk(self, fpath):
    #     with open(fpath, "w") as f:
    #         pk.dump(self.objs, f)


if __name__ == "__main__":
    print("""Choose a world to parse:
    1: Wall with People
    2: Trees with People
    3: Door with People
    """)

    choice = int(raw_input("Choose an option: "))

    if choice == 1:
        fpath = "/opt/parrot-sphinx/usr/share/sphinx/worlds/goal_stop_after_wall_people.world"
        w = WorldParser(fpath)
        print(w)
        for m in w.models:
            print(m)

    if choice == 2:
        fpath = "/opt/parrot-sphinx/usr/share/sphinx/worlds/goal_pass_around_trees.world"
        w = WorldParser(fpath)
        
        w.objects
        for m in w.models:
            print(m)

    if choice == 3:
        fpath = "/opt/parrot-sphinx/usr/share/sphinx/worlds/goal_cross_door.world"
        w = WorldParser(fpath)
        print(w.models)
        for m in w.models:
            print(m)