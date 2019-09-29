# Automated testing with Sphinx

SIMULATED_IP=10.204.0.1
WORLD_FPATH=$(python generate_world.py --n-peds 2)

load_olympe  # in-shell Olympe load

python control_anafi --sim \
    --ip $SIMULATED_IP \
    --world $WORLD_FPATH