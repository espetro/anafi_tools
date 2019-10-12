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

# def random_log_file(fdir, ext="csv"):
#     rnd = datetime.now().strftime("%y%m%d_%H%M%S")
#     return "{}/telemetry{}.{}".format(fdir, rnd, ext)

# CONFIG = {
#     "runs": 1,
#     "delay_start": 30,
#     "object_probs": {"tree": 0.4, "door": 0.0, "wall": 0.2},
#     "world_shape": (5,5),
#     "number_peds": 1,
#     "maximum_objects": 5,
#     "subj_to_goal_dist": 4,
#     "simulated": True,
#     "model": None,
#     "ip": None,
#     "logfile": random_log_file(DATA_DIR, "csv")
# }

# ~2s to generate a (10,10) world with dist=10

A_CONFIG = {
    # general options
    "name": "envA",
    "reps": 1,
    "delay_start": 20,  # take into account if the drone's height is setup
    "set_height": False,
    
    # world-generation options
    "world_shape": (10,10),  # (MxM)
    "object_probs": {"tree": 0.0, "door": 0.0, "wall": 0.0},
    "maximum_objects": 5,
    "subj_to_goal_dist": 10,  # maximum is (Mx2)-2, minimum is M
    "num_peds": (0,0),  # between 0 and 0 peds (varies within each config)
    "peds_loop": True,

    # data-logger options
    "sample_rate": 1000,  # 1000 ms
    "samples_per_s": 1,  # 1 sample per 1s
    "datapath": "/home/pachacho/Documents/anafi_tools/data/train/A",  # make sure it exists!
    
    # overwritten by the runtime
    "final_peds": 0,  
    "run_name": "run0"

}

B_CONFIG = {}

D_CONFIG = {}

K_CONFIG = {}

L_CONFIG = {}

M_CONFIG = {}

C_CONFIG = {}

LAB_CONFIG = {}