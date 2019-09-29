
from __future__ import print_function
from tempfile import NamedTemporaryFile
from generators.templates import PATH_TEMPLATE_END, PATH_TEMPLATE_START

class PathGen:
    def __init__(self, points, track_models=False,
                 loop=False, delay_start=0.0, auto_start=True):
        """
        Creates a pedestrian path. By default, the path is done by walking
        (speed is 1) without stop points.

        :param points: A list of tuples (position, speed=1, stop_duration=0)
        :param track_models: If True, position is given by a model's name
        :param loop: If True, the path is played on a loop
        :param delay_start: An integer to set how late the path will be crossed
        :param auto_start: If True, the path is started right after the delay
        """
        self.f = NamedTemporaryFile(mode="w", prefix=".path", delete=False)
        
        self.fpath = f.name
        self.points = points
        self.auto_start = str(auto_start).lower()
        self.delay_start = str(delay_start)
        self.loop = str(loop).lower()

        if track_models:
            self.point_tag = "<model>{}</model>"
        else:
            self.point_tag = "<xy>{} {}</xy>"

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

    def write_to_template(self):
        """Writes the input data to the file template"""
        waypoints = []

        for (point, vel, stop) in self.points:
            if type(point) == type(""):
                point_tag = self._point_tag().format(point)
            else:
                point_tag = self._point_tag().format(point[0], point[1])

            waypoint_tag = self._waypoint_tag(float(vel), point_tag, float(stop))
            waypoints.append(waypoint_tag)

        txt_waypoints = "\n".join(waypoints)
        template = PATH_TEMPLATE_START.format(
            "tempPath", self.auto_start, self.delay_start, self.loop_str
        )
        
        temp_txt = template + txt_waypoints + PATH_TEMPLATE_END
        self.f.write(temp_txt)
        self.f.close()

    def get_path(self):
        """Returns the filepath of the path file."""
        return self.fpath

