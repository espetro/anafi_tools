from __future__ import print_function
from templates import PATH_TEMPLATE_START, PATH_TEMPLATE_END
from tempfile import NamedTemporaryFile
from random import randint
# from generators.templates import PATH_TEMPLATE_START, PATH_TEMPLATE_END

class PathGen:
    def __init__(self, fdir, points, track_models=False,
                 loop=False, delay_start=5.0, auto_start=True, is_ped=False):
        """
        Creates a pedestrian path. By default, the path is done by walking
        (speed is 1) without stop points.

        :param points: A list of tuples (position, cell_type, speed=1, stop_duration=0)
        :param track_models: If True, position is given by a model's name
        :param loop: If True, the path is played on a loop
        :param delay_start: An integer to set how late the path will be crossed
        :param auto_start: If True, the path is started right after the delay
        :param is_ped:
        """
        self.f = NamedTemporaryFile(mode="w", suffix=".path", dir=fdir, delete=False)
        
        self.points = points
        self.auto_start = str(auto_start).lower()
        self.delay_start = str(delay_start)
        self.loop = str(loop).lower()
        self.track_models = track_models
        self.is_ped = is_ped
        
        self.write_to_template()

    def _point_tag(self):
        """Returns a point tag"""
        return self.point_tag

    def _waypoint_tag(self, vel, point, stop=0.0):
        """
        Stringify a point.

        :param vel:
        :param point:
        :param stop:
        """
        txt = "walk"
        if vel > 1.5:
            txt = "run"

        return """
        <waypoint>
            <animation>{}</animation>
            <velocity>{}</velocity>
            {}
            <stop_duration>{}</stop_duration>
        </waypoint>""".format(txt, vel, point, stop)

    def get_point_tag(self, point):
        if self.track_models:
            return "<model>{}</model>".format(point)
        else:
            return "<xy>{} {}</xy>".format(point[0], point[1])
        
    def get_move_dir(self, p1, p2):
        if abs(p1[0] - p2[0]) == 1:
            return "N"
        elif abs(p1[1] - p2[1]) == 1:
            return "W"
        
    def make_turn_by(self, dist, prev_point, vel, point, stop):
        move_dir = self.get_move_dir(prev_point, point)
        dist *= [-1,1][(randint(0,1))]
        if move_dir == "N":
            p = (point[0], point[1] + dist)
        else:
            p = (point[0] + dist, point[1])
        return self._waypoint_tag(vel, self.get_point_tag(p), stop)
    
    def write_to_template(self):
        """Writes the input data to the file template"""
        waypoints = []

        for (i, (point, ctype, vel, stop)) in enumerate(self.points):
            point_tag = self.get_point_tag(point)
            
            if ctype == "T":
                # first and last cells are not "T" (trees)
                waypoint_tag = self.make_turn_by(0.5, self.points[i-1][0], float(vel), point, float(stop))
                waypoints.append(waypoint_tag)
            else:
                waypoint_tag = self._waypoint_tag(float(vel), point_tag, float(stop))
                waypoints.append(waypoint_tag)

        if self.is_ped:
            (point, _, vel, stop) = self.points[0]
            point_tag = self.get_point_tag(point)
            waypoint_tag = self._waypoint_tag(float(vel), point_tag, float(stop))
            waypoints.append(waypoint_tag)
                
        txt_waypoints = "\n".join(waypoints)
        template = PATH_TEMPLATE_START.format(
            "tempPath", self.auto_start, self.delay_start, self.loop
        )
        
        temp_txt = template + txt_waypoints + PATH_TEMPLATE_END
        self.f.write(temp_txt)
        self.f.close()

    def get_path(self):
        """Returns the filepath of the path file."""
        return self.f.name

