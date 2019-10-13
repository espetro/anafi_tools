# -*- coding: utf-8 -*-
from __future__ import print_function, absolute_import
from tempfile import mkdtemp
from random import randint
from generators.list2path import PathGen
from generators.grid2world import WorldGen
from generators.path_finder import PathFinder
from generators.random_world import RandomWorld
from models.world_parser import WorldParser

class WorldBuilder:
    def __init__(self, config):
        """
        Builds a new world given the configuration dictionary
        """
        min_peds, max_peds = config["num_peds"]

        self.probs = config["object_probs"]
        self.shape = config["world_shape"]
        self.peds = randint(min_peds, max_peds)
        self.max_objs = config["maximum_objects"]
        self.sj2gl = config["subj_to_goal_dist"]
        
        max_dist = (self.shape[0] * 2) - 2  # given Manhattan distance
        if self.sj2gl > max_dist:
            self.sj2gl = max_dist

        self.world = RandomWorld(
            self.shape, self.sj2gl, self.peds, self.probs, self.max_objs
        )

        print("Finding a path for the subject")
        path_model = PathFinder(
            self.world.grid, self.world.subj_pos, self.world.goal_pos
        )
        print("Path found")
        self.path = path_model.a_star()

        self.world_fpath, self.subj_fpath, self.peds_fpath = self._gen2file(config["delay_start"])

    def get_num_peds(self):
        return self.peds
        
    def _gen2file(self, delay_start=20):
        """"""
        self.fdir = mkdtemp(prefix="sphinx_", suffix="_worldconf")

        # Matches to ((x,y), type, vel, stop_duration)
        path_points = [(point, ctype, 1.0, 0.0) for (point, ctype) in self.path]

        tree_locs = [tuple(val["pos"]) for val in self.world.trees.values()]
        door_locs = [door["pos"] for door in self.world.doors.values()]
        wall_locs = [wall["pos"][0] for wall in self.world.walls.values()]

        subj_fpath = PathGen(
            self.fdir, path_points, delay_start=delay_start
        ).get_path()
        world_fpath = WorldGen(
            self.fdir, self.world.grid, self.world.goal_pos, self.world.subj_pos,
            self.world.drone_pos, self.world.peds, tree_locs, door_locs, wall_locs
        ).get_path()

        peds_fpath = []

        for (name, data) in self.world.peds.items():
            points = [(p, "P", 1.0, 0.0) for p in data["pos"][0]]
            generator = PathGen(
                self.fdir, points, loop=True, delay_start=0.0, is_ped=True
            )
            peds_fpath.append((name, generator.get_path()))

        return (world_fpath, subj_fpath, peds_fpath)

    def get_object_models(self):
        """
        Returns the object models available in the current world
        """
        parser = WorldParser(self.world_fpath)
        return parser.models
        

    def get_paths(self):
        """
        Returns a tuple with the world filepath, the subject's path filepath
        and the path filepath for each of the pedestrians
        """
        return (self.world_fpath, self.subj_fpath, self.peds_fpath)