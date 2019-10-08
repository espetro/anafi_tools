#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
from __future__ import print_function
from tempfile import mkdtemp
from subprocess import Popen
from generators.list2path import PathGen
from generators.grid2world import WorldGen
from generators.path_finder import PathFinder
from generators.random_world import RandomWorld

def run_sphinx(world, path):
    fdir = mkdtemp(prefix="sphinx_", suffix="_worldconf")

    points = [(point, ctype, 1.0, 0.0) for (point, ctype) in path]
    tree_pts = [tuple(val["pos"]) for val in world.trees.values()]
    wall_pts = [world.walls.values()[0]["pos"][0]]  # a list of walls

    subj_gen = PathGen(fdir, points)
    world_gen = WorldGen(
        fdir, world.grid, world.goal_pos, world.subj_pos, world.drone_pos,
        world.peds, tree_pts, None, wall_pts
    )

    print(subj_gen.get_path())
    print(world_gen.get_path())

    DRONE = "/opt/parrot-sphinx/usr/share/sphinx/drones/local_bebop2.drone"
    ACTOR = "/opt/parrot-sphinx/usr/share/sphinx/actors/pedestrian.actor::name={}::path={}"

    PED_FPATHS = []

    for (name, data) in world.peds.items():
        points = [(p, "P", 1.0, 0.0) for p in data["pos"][0]]
        generator = PathGen(fdir, points, loop=data["loop"], delay_start=0.0, is_ped=True)
        PED_FPATHS.append((name, generator.get_path()))

    cmd = "sphinx {} {} {} {}".format(
        world_gen.get_path(),
        DRONE,
        ACTOR.format("subject", subj_gen.get_path()),
        " ".join([ACTOR.format(nm, pth) for (nm, pth) in PED_FPATHS])
    )

    Popen(cmd.split(" "))
    

if __name__ == "__main__":

    obj_probs = {"tree": 0.4, "door": 0.0, "wall": 0.2}

    world = RandomWorld(
        shape=(5,5),
        sub2goal_dist=4,
        no_peds=1,
        obj_probs=obj_probs,
        max_objs=5
    )

    path_finder = PathFinder(world.grid, world.subj_pos, world.goal_pos)
    path = path_finder.a_star()

    print("\nThe obtained world grid is:")
    print(world.grid)
    print()
    print("\nThe subject's path to the goal is: " + str(path) + "\n")

    # run_sphinx(world, path)

    # config = {
    #     "object_probs": {"tree": 0.4, "door": 0.0, "wall": 0.2},
    #     "world_shape": (5,5),
    #     "number_peds": 1,
    #     "maximum_objects": 5,
    #     "subj_to_goal_dist": 4
    # }

    # config = { "object_probs": {"tree": 0.4, "door": 0.0, "wall": 0.2}, "world_shape": (5,5), "number_peds": 1, "maximum_objects": 5, "subj_to_goal_dist": 4}