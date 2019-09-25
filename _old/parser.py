from __future__ import print_function
from scanning.block import BlockShape
from scanning.cylinder import CylinderShape
from scanning.sphere import SphereShape

import xml.etree.ElementTree as ET
import pickle as pk


class WorldParser(object):

    def __init__(self, fpath):
        """"""
        models = self._get_all_models(fpath)
        self.objs = self._match_with_shapes(models)
        
    @staticmethod
    def _as_dict(model):
        d = dict()
        for tag in model:
            d[tag.tag] = tag.text

    def _get_all_models(self, fpath):
        root = ET.parse(fpath).getroot()
        elems = root.find("world").getchildren()
        models = [el.getchildren() for el in elems if el.tag == "include"]
        return [self._as_dict(m) for m in models if len(m) > 1]

    def _match_with_shapes(self, models):
        shapes = []
        for m in models:
            pose = [float(coord) for coord in m["pose"].split(" ")]
            if m["shape"] == "sphere":
                shapes.append(SphereShape(pose, m["radius"]))
            elif m["shape"] == "block":
                shapes.append(BlockShape(pose, m["dims"]))
            elif m["shape"] == "cylinder":
                shapes.append(CylinderShape(pose, m["radius"], m["height"]))
            
        return shapes


    def write_to_file(self, fpath):
        with open(fpath, "w") as f:
            for o in self.objs:
                f.write("{} {}".format(o.pose, o.volume))

    def save_to_pk(self, fpath):
        with open(fpath, "w") as f:
            pk.dump(self.objs, f)

# /opt/parrot-sphinx/usr/share/sphinx/worlds/simple_cross_door.world
