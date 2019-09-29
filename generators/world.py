
from __future__ import print_function
from tempfile import NamedTemporaryFile
from generators.templates import WORLD_TEMPLATE_START, WORLD_TEMPLATE_END

class WorldGen:
    template = """
    """

    def __init__(self):
        """
        Creates a .world file with the given parameters for objects and
        pedestrians.

        :param ___:
        """
        
        self.drone_init_pos = "0 0 0.2 0 0 0"


    def _object_tag(self):
        """Returns an object 'include' tag"""
        pose = "0 1.5 0 0 0 0"
        obj_name = "washer_1"
        obj_model = "washer"

        return """
        <include>
            <name>{}</name>
            <uri>model://{}</uri>
            <pose>{}</pose>
        </include>
        """.format(obj_name, obj_model, pose)

    def write_to_template(self):
        """"""
        includes = []

        # for loop
        
        objs_includes = "\n".join(includes)

        template = WORLD_TEMPLATE_START.format(self.drone_pos) + \
            objs_includes + WORLD_TEMPLATE_END