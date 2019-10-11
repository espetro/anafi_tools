# Write out all the possible simulation configurations
# This is used for both creating and running the simulated world

# A environment: simple path, no drone variations
# B environment: simple path with drone height-pose variations
# D environment: path with bots
# (J environment: path with just doors)
# K environment: path with just trees
# L environment: path with trees and walls (and doors)
# M environment: mix of K,L for testing
# C environment: mix of D,K,L for testing
# 'Lab' environment: space-limited D,K,Ls

CONFIG = {
    "runs": 1,
    "delay_start": 30,
    "object_probs": {"tree": 0.4, "door": 0.0, "wall": 0.2},
    "world_shape": (5,5),
    "number_peds": 1,
    "maximum_objects": 5,
    "subj_to_goal_dist": 4,
    "simulated": True,
    "model": None,
    "ip": None,
    "logfile": random_log_file(DATA_DIR, "csv")
}


A_CONFIG = {
    "name": "envA",
    "datapath": "/home/pachacho/Documents/anafi_tools/data/train/A",
    "reps": 80,
    "objs": {"tree": 0.0, "door": 0.0, "wall": 0.0},
    "num_peds": (0,0),  # between 0 and 0 peds (varies within each config)
    "peds_loop": True,
    "sample_rate": 1000,  # 1000 ms
    "samples_per_s": 1,  # 1 sample per 1s
    "delay_start": 30,  # take into account if the drone's height is setup
    "set_height": False,

}

B_CONFIG = {}

D_CONFIG = {}

K_CONFIG = {}

L_CONFIG = {}

M_CONFIG = {}

C_CONFIG = {}

LAB_CONFIG = {}