# -*- coding: utf-8 -*-

# ==========================================================
# Functions
# ==========================================================

# ~2s to generate a (10,10) world with dist=8
# Note sometimes it can take more than 30s, due to the brute-force path-finding approach

def default_config():
    """Generates the default setup for a workflow run"""
    return {
        # general options
        "name": "environment",
        "reps": 20,
        "delay_start": 16,  # take into account if the drone's height is setup
        "set_height": False,
        "simulated": True,  # not implemented - diff between simulated and real
        "ip": None,  # not implemented - diff between simulated and real
        "model": None,  # not implemented - point the neural engine to use

        # overwritten by the runtime
        "final_peds": 0,  
        "run_name": "run0",

        # world-generation options
        "world_shape": (5,5),  # (MxM)
        "object_probs": {"tree": 0.5, "door": 0.0, "wall": 0.5},  # sum is 1
        "maximum_objects": 5,
        "subj_to_goal_dist": 8,  # maximum is (Mx2)-2, minimum is M
        "subject_vel": 0.6,  # defaults to 1.0, go slower if you want more samples
        "num_peds": (0,2),  # between 0 and 0 peds (varies within each run)
        "peds_loop": True,
        "time_limit": 30,  # drop the iteration if the world generation lasts more (s)

        # controller options - (65, 0.2) recommended for beginners
        "speed": 60,
        "refresh": 0.25,

        # data-logger options
        "sample_rate": 1000,  # 1000 ms
        "samples_per_s": 1,  # 1 sample per 1s (not implemented yet)
        "datapath": "/home/pachacho/Documents/anafi_tools/data/train"  # make sure it exists!
    }

# ==========================================================
# Scenarios' setups
# ==========================================================

# A environment: simple path, no drone variations
# B environment: simple path with drone height-pose variations
# D environment: path with bots
# (J environment: path with just doors)
# K environment: path with just trees
# L environment: path with trees and walls (and doors)
# M environment: mix of K,L for testing
# C environment: mix of D,K,L for testing
# 'Lab' environment: space-limited D,K,Ls

A_CONFIG = {
    # general options
    "name": "envA",
    "reps": 20,  # running in batches of length 20 seems the best
    "delay_start": 16,
    "set_height": False,
    "simulated": True,  # not implemented - diff between simulated and real
    "ip": None,  # not implemented - diff between simulated and real
    "model": None,  # not implemented - point the neural engine to use

    # overwritten by the runtime
    "final_peds": 0,  
    "run_name": "run0",

    # world-generation options
    "world_shape": (15,15),
    "object_probs": {"tree": 0.0, "door": 0.0, "wall": 0.0},
    "maximum_objects": 5,
    "subj_to_goal_dist": 7,
    "subject_vel": 0.6,
    "num_peds": (0,0),
    "peds_loop": True,
    "early_stop": 10,  # drop the iteration if the world generation lasts more

    # controller options - (65, 0.2) recommended for beginners
    "speed": 66,
    "refresh": 0.2,

    # data-logger options
    "sample_rate": 1000,
    "samples_per_s": 1,
    "datapath": "/home/pachacho/Documents/anafi_tools/data/train/A/"
}

B_CONFIG = default_config()

D_CONFIG = default_config()

K_CONFIG = default_config()

L_CONFIG = default_config()

M_CONFIG = default_config()

C_CONFIG = default_config()

LAB_CONFIG = default_config()