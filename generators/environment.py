
from __future__ import print_function
from tempfile import mkdtemp
from random_world import RandomWorld
from path_finder import PathFinder
from list2path import PathGen
from grid2world import WorldGen
from subprocess import Popen
# from generators.random_world import RandomWorld
# from generators.path_finder import PathFinder
# from generators.list2path import PathGen
# from generators.grid2world import WorldGen

if __name__ == "__main__":

    obj_probs = obj_probs = {"tree": 0.4, "door": 0.0, "wall": 0.2}

    world = RandomWorld(
        shape=(5,5),
        sub2goal_dist=4,
        no_peds=1,
        obj_probs=obj_probs,
        max_objs=5
    )

    path_finder = PathFinder(world.grid, world.subj_pos, world.goal_pos)
    path = path_finder.a_star()

    print(world.grid)
    print(path)

    FDIR = mkdtemp(prefix="sphinx_", suffix="_worldconf")

    points = [(point, ctype, 1.0, 0.0) for (point, ctype) in path]
    tree_pts = [tuple(val["pos"]) for val in world.trees.values()]
    wall_pts = [world.walls.values()[0]["pos"][0]]  # a list of walls

    subj_gen = PathGen(FDIR, points)
    world_gen = WorldGen(
        FDIR, world.grid, world.goal_pos, world.subj_pos, world.drone_pos,
        world.peds, tree_pts, None, wall_pts
    )

    print(subj_gen.get_path())
    print(world_gen.get_path())

    DRONE = "/opt/parrot-sphinx/usr/share/sphinx/drones/local_bebop2.drone"
    ACTOR = "/opt/parrot-sphinx/usr/share/sphinx/actors/pedestrian.actor::name={}::path={}"

    PED_FPATHS = []

    for (name, data) in world.peds.items():
        points = [(p, "P", 1.0, 0.0) for p in data["pos"][0]]
        generator = PathGen(FDIR, points, loop=data["loop"], delay_start=0.0, is_ped=True)
        PED_FPATHS.append((name, generator.get_path()))

    cmd = "sphinx {} {} {} {}".format(
        world_gen.get_path(),
        DRONE,
        ACTOR.format("subject", subj_gen.get_path()),
        " ".join([ACTOR.format(nm, pth) for (nm, pth) in PED_FPATHS])
    )

    # Popen(["code", subj_gen.get_path()])
    Popen(cmd.split(" "))