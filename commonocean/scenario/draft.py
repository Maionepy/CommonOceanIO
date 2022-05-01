from commonroad.geometry.shape import Shape

__author__ = "Hanna Krasowski, Benedikt Pfleiderer, Fabian Thomas-Barein"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = ["ConVeY"]
__version__ = "2022a"
__maintainer__ = "Hanna Krasowski"
__email__ = "commonocean@lists.lrz.de"
__status__ = "released"


class Draft:

    """Class to describe a draft with a defined shape and depth"""
    def __init__(self, shape: Shape, depth: float = 10.0):
        """
        :param shape: shape of the draft
        :param depth: depth of the draft in meters (default: 10.0)
        """

        self._shape = shape
        self._depth = depth

    @property
    def shape(self):
        return self._shape

    @shape.setter
    def shape(self, shape: Shape):
        assert isinstance(shape, Shape), \
            '<Draft/shape>: argument shape of wrong ' \
            'type. Expected type: %s. Got type: %s.' \
            % (Shape, type(shape))
        self._shape = shape

    @property
    def depth(self):
        return self._depth

    @depth.setter
    def depth(self, depth: float):
        assert isinstance(depth, float), \
            '<Draft/depth>: argument depth of wrong ' \
            'type. Expected type: %s. Got type: %s.' \
            % (float, type(depth))
        self._depth = depth

    def __str__(self):
        draft_str = "\n"
        draft_str += "Draft:\n"
        draft_str += "- Shape: {}\n".format(type(self._shape).__name__)
        draft_str += "- Center-Position: {}\n".format(str(self.shape.center))
        draft_str += "- Depth: {} Meters\n".format(str(self.depth))
        return draft_str
