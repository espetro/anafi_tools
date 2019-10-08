# -*- coding: utf-8 -*-
from __future__ import absolute_import
from models.simple_wall import SimpleWallShape
from models.top_door_rail import TopDoorRailShape
from models.vbar import VerticalBarShape
from models.tree_canopy import TreeCanopyShape
from models.wall_box import WallBoxShape

import xml.etree.ElementTree as ET


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
                WallBoxShape.model: WallBoxShape(m["name"], _pose[:3])
            }.get(m["uri"], None)

            if model is not None:
                self.models.append(model)
