import copy
import enum
from typing import *
import numpy as np

from commonroad.geometry.shape import Circle, Rectangle, Shape, Polygon, ShapeGroup
from commonroad.geometry.transform import translation_rotation_matrix
from commonroad.common.validity import *

from commonocean.scenario.obstacle import Obstacle
from commonocean.scenario.traffic_sign import TrafficSign, TrafficLight


__author__ = "Hanna Krasowski, Benedikt Pfleiderer, Fabian Thomas-Barein"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = ["ConVeY"]
__version__ = "2022a"
__maintainer__ = "Hanna Krasowski"
__email__ = "commonocean@lists.lrz.de"
__status__ = "released"


class LineMarking(enum.Enum):
    """
    Enum describing different types of water borders, i.e. land, border of water
    """
    LAND = 'land'


class WatersType(enum.Enum):
    """
    Enum describing different types of waters
    """
    UNKNOWN = 'unknown'
    OPENSEA = 'opensea'
    WATER = 'water'
    TRAFFICSEPARATIONZONE = 'trafficseparationzone'


class WatersUser(enum.Enum):
    """
    Enum describing different types of water users
    """
    VESSEL = 'vessel'
    MOTORVESSEL = 'motorvessel'
    SAILINGVESSEL = 'sailingvessel'
    CARGOSHIP = 'cargoship'
    SWIMMER = 'swimmer'


class Waters:
    """
    Class which describes a Waters entity according to the CommonOcean specification.
    """

    def __init__(self, left_vertices: np.ndarray, center_vertices: np.ndarray, right_vertices: np.ndarray,
                 waters_id: int, predecessor=None, successor=None,
                 waters_type=None,
                 traffic_signs=None,
                 traffic_lights=None
                 ):
        """
        Constructor of a Waters object
        :param left_vertices: The vertices of the left boundary of the Waters described as a
        polyline [[x0,x1,...,xn],[y0,y1,...,yn]]
        :param center_vertices: The vertices of the center line of the Waters described as a
        polyline [[x0,x1,...,xn],[y0,y1,...,yn]]
        :param right_vertices: The vertices of the right boundary of the Waters described as a
        polyline [[x0,x1,...,xn],[y0,y1,...,yn]]
        :param waters_id: The unique id (natural number) of the water
        :param predecessor: The list of predecessor waters (None if not existing)
        :param successor: The list of successor waters (None if not existing)
        :param traffic_signs: Traffic signs to be applied
        :param traffic_lights: Traffic lights to follow
        """

        # Set required properties
        self._left_vertices = None
        self._right_vertices = None
        self._center_vertices = None
        self._waters_id = None

        self.waters_id = waters_id
        self.left_vertices = left_vertices
        self.right_vertices = right_vertices
        self.center_vertices = center_vertices
        # check if length of each polyline is the same
        assert len(left_vertices[0]) == len(center_vertices[0]) == len(
            right_vertices[0]), '<Waters/init>: Provided polylines do not share the same length! {}/{}/{}'.format(
            len(left_vertices[0]), len(center_vertices[0]), len(right_vertices[0]))

        # Set predecessors and successors
        self._predecessor = None
        if predecessor is None:
            self._predecessor = []
        else:
            self.predecessor = predecessor
        self._successor = None
        if successor is None:
            self._successor = []
        else:
            self.successor = successor

        # create empty polygon
        self._polygon = None

        self._dynamic_obstacles_on_water = {}
        self._static_obstacles_on_water = set()

        self._waters_type = None
        if waters_type is None:
            self._waters_type = set()
        else:
            self.waters_type = waters_type

        # Set Traffic Rules
        self._traffic_signs = None
        if traffic_signs is None:
            self._traffic_signs = set()
        else:
            self.traffic_signs = traffic_signs

        self._traffic_lights = None
        if traffic_lights is None:
            self._traffic_lights = set()
        else:
            self.traffic_lights = traffic_lights


    @property
    def distance(self) -> np.ndarray:
        return self._distance

    @distance.setter
    def distance(self, dist: np.ndarray):
        warnings.warn('<Waters/distance> distance of waters is immutable')

    @property
    def waters_id(self) -> int:
        return self._waters_id

    @waters_id.setter
    def waters_id(self, f_id: int):
        if self._waters_id is None:
            assert is_natural_number(f_id), '<Waters/waters_id>: Provided waters_id is not valid! id={}'.format(f_id)
            self._waters_id = f_id
        else:
            warnings.warn('<Waters/waters_id>: waters_id of waters is immutable')

    @property
    def left_vertices(self) -> np.ndarray:
        return self._left_vertices

    @left_vertices.setter
    def left_vertices(self, polyline: np.ndarray):
        if self._left_vertices is None:
            assert is_valid_polyline(
                polyline), '<Waters/left_vertices>: The provided polyline is not valid! polyline = {}'.format(polyline)
            self._left_vertices = polyline
        else:
            warnings.warn('<Waters/left_vertices>: left_vertices of waters are immutable!')

    @property
    def right_vertices(self) -> np.ndarray:
        return self._right_vertices

    @right_vertices.setter
    def right_vertices(self, polyline: np.ndarray):
        if self._right_vertices is None:
            assert is_valid_polyline(
                polyline), '<Waters/right_vertices>: The provided polyline is not valid! polyline = {}'.format(
                polyline)
            self._right_vertices = polyline
        else:
            warnings.warn('<Waters/right_vertices>: right_vertices of waters are immutable!')

    @property
    def center_vertices(self) -> np.ndarray:
        return self._center_vertices

    @center_vertices.setter
    def center_vertices(self, polyline: np.ndarray):
        if self._center_vertices is None:
            assert is_valid_polyline(
                polyline), '<Waters/center_vertices>: The provided polyline is not valid! polyline = {}'.format(
                polyline)
            self._center_vertices = polyline
        else:
            warnings.warn('<Waters/center_vertices>: center_vertices of water are immutable!')

    @property
    def predecessor(self) -> list:
        return self._predecessor

    @predecessor.setter
    def predecessor(self, predecessor: list):
        if self._predecessor is None:
            assert (is_list_of_natural_numbers(predecessor) and len(predecessor) >= 0), '<Waters/predecessor>: ' \
                                                                                        'Provided list ' \
                                                                                        'of predecessors is not valid!' \
                                                                                        'predecessors = {}'.format(
                predecessor)
            self._predecessor = predecessor
        else:
            warnings.warn(
                '<Waters/predecessor>: predecessor of waters is immutable!')

    @property
    def successor(self) -> list:
        return self._successor

    @successor.setter
    def successor(self, successor: list):
        if self._successor is None:
            assert (is_list_of_natural_numbers(successor) and len(successor) >= 0), '<Waters/predecessor>: Provided ' \
                                                                                    'list of successors is not valid!' \
                                                                                    'successors = {}'.format(successor)
            self._successor = successor
        else:
            warnings.warn(
                '<Waters/successor>: successor of water is immutable!')

    @property
    def dynamic_obstacles_on_water(self) -> Dict[int, Set[int]]:
        return self._dynamic_obstacles_on_water

    @dynamic_obstacles_on_water.setter
    def dynamic_obstacles_on_water(self, obstacle_ids: Dict[int, Set[int]]):
        assert isinstance(obstacle_ids, dict), \
            '<Waters/obstacles_on_water>: provided dictionary of ids is not a ' \
            'dictionary! type = {}'.format(type(obstacle_ids))
        self._dynamic_obstacles_on_water = obstacle_ids

    @property
    def static_obstacles_on_water(self) -> Union[None, Set[int]]:
        return self._static_obstacles_on_water

    @static_obstacles_on_water.setter
    def static_obstacles_on_water(self, obstacle_ids: Set[int]):
        assert isinstance(obstacle_ids, set), \
            '<Waters/obstacles_on_water>: provided list of ids is not a ' \
            'set! type = {}'.format(type(obstacle_ids))
        self._static_obstacles_on_water = obstacle_ids

    @property
    def waters_type(self) -> Set[WatersType]:
        return self._waters_type

    @waters_type.setter
    def waters_type(self, waters_type: Set[WatersType]):
        if self._waters_type is None or len(self._waters_type) == 0:
            assert isinstance(waters_type, set) and all(isinstance(elem, WatersType) for elem in waters_type), \
                '<Waters/waters_type>: ''Provided type is not valid! type = {}'.format(type(waters_type))
            self._waters_type = waters_type
        else:
            warnings.warn(
                '<Water/waters_type>: type of water is immutable!')

    @property
    def traffic_signs(self) -> Set[int]:
        return self._traffic_signs

    @traffic_signs.setter
    def traffic_signs(self, traffic_sign_ids: Set[int]):
        if self._traffic_signs is None:
            assert isinstance(traffic_sign_ids, set), \
                '<Waters/traffic_signs>: provided list of ids is not a ' \
                'set! type = {}'.format(type(traffic_sign_ids))
            self._traffic_signs = traffic_sign_ids
        else:
            warnings.warn(
                '<Waters/traffic_signs>: traffic_signs of water is immutable!')

    @property
    def traffic_lights(self) -> Set[int]:
        return self._traffic_lights

    @traffic_lights.setter
    def traffic_lights(self, traffic_light_ids: Set[int]):
        if self._traffic_lights is None:
            assert isinstance(traffic_light_ids, set), \
                '<Waters/traffic_lights>: provided list of ids is not a ' \
                'set! type = {}'.format(type(traffic_light_ids))
            self._traffic_lights = traffic_light_ids
        else:
            warnings.warn(
                '<Waters/traffic_lights>: traffic_lights of water is immutable!')

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a water

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation,
                                     2), '<Waters/translate_rotate>: provided translation ' \
                                         'is not valid! translation = {}'.format(translation)
        assert is_valid_orientation(
            angle), '<Waters/translate_rotate>: provided angle is not valid! angle = {}'.format(angle)

        # create transformation matrix
        t_m = translation_rotation_matrix(translation,angle)
        # transform center vertices
        tmp = t_m.dot(np.vstack((self.center_vertices.transpose(),
                                 np.ones((1, self.center_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._center_vertices = tmp.transpose()

        # transform left vertices
        tmp = t_m.dot(np.vstack((self.left_vertices.transpose(),
                                 np.ones((1, self.left_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._left_vertices = tmp.transpose()

        # transform right vertices
        tmp = t_m.dot(np.vstack((self.right_vertices.transpose(),
                                 np.ones((1, self.right_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._right_vertices = tmp.transpose()

        # recreate polygon in case it existed
        if self._polygon is not None:
            self._polygon = None
            self._polygon = self.convert_to_polygon()

    def convert_to_polygon(self) -> Polygon:
        """
        Converts the given water to a polygon representation

        :return: The polygon of the water
        """
        if self._polygon is None:
            self._polygon = Polygon(np.concatenate((self.right_vertices,
                                                    np.flip(self.left_vertices, 0))))
        return self._polygon

    def contains_points(self, point_list: np.ndarray) -> List[bool]:
        """
        Checks if a list of points is enclosed in the water

        :param point_list: The list of points in the form [[px1,py1],[px2,py2,],...]
        :return: List of bools with True indicating point is enclosed and False otherwise
        """
        assert isinstance(point_list,
                          ValidTypes.ARRAY), '<Waters/contains_points>: provided list of points is not a list! type ' \
                                             '= {}'.format(type(point_list))
        assert is_valid_polyline(
            point_list), 'Waters/contains_points>: provided list of points is malformed! points = {}'.format(
            point_list)

        # output list
        res = list()

        # get polygon shape of water
        poly = self.convert_to_polygon()
        for p in point_list:
            res.append(poly.contains_point(p))

        return res

    def get_obstacles(self, obstacles: List[Obstacle], time_step: int = 0) -> List[Obstacle]:
        """
        Returns the subset of obstacles,  which are located in the water,  of a given candidate set

        :param obstacles: The set of obstacle candidates
        :param time_step: The time step for the occupancy to check
        :return:
        """

        assert isinstance(obstacles, list) and all(isinstance(o, Obstacle) for o in
                                                   obstacles), '<Waters/get_obstacles>: Provided list of obstacles' \
                                                               ' is malformed! obstacles = {}'.format(
            obstacles)

        # output list
        res = list()

        # look at each obstacle
        for o in obstacles:
            o_shape = o.occupancy_at_time(time_step).shape

            # vertices to check
            vertices = list()

            # distinguish between shape and shapegroup and extract vertices
            if isinstance(o_shape, ShapeGroup):
                for sh in o_shape.shapes:
                    # distinguish between type of shape (circle has no vertices)
                    if isinstance(sh, Circle):
                        vertices.append(sh.center)
                    else:
                        vertices.append(sh.vertices)
                        vertices = np.append(vertices, [o_shape.center], axis=0)
            else:
                # distinguish between type of shape (circle has no vertices)
                if isinstance(o_shape, Circle):
                    vertices = o_shape.center
                else:
                    vertices = o_shape.vertices
                    vertices = np.append(vertices, [o_shape.center], axis=0)

            # check if obstacle is in lane
            if any(self.contains_points(np.array(vertices))):
                res.append(o)

        return res

    ## here waterS is probably better

    def add_dynamic_obstacle_to_water(self, obstacle_id: int, time_step: int):
        """
        Adds a dynamic obstacle ID to water

        :param obstacle_id: obstacle ID to add
        :param time_step: time step at which the obstacle should be added
        """
        if self.dynamic_obstacles_on_water.get(time_step) is None:
            self.dynamic_obstacles_on_water[time_step] = set()
        self.dynamic_obstacles_on_water[time_step].add(obstacle_id)


    ## here waterS is probably better

    def add_static_obstacle_to_water(self, obstacle_id: int):
        """
        Adds a static obstacle ID to water

        :param obstacle_id: obstacle ID to add
        """
        self.static_obstacles_on_water.add(obstacle_id)

    def add_traffic_sign_to_water(self, traffic_sign_id: int):
        """
        Adds a traffic sign ID to water

        :param traffic_sign_id: traffic sign ID to add
        """
        self.traffic_signs.add(traffic_sign_id)

    def add_traffic_light_to_waters(self, traffic_light_id: int):
        """
        Adds a traffic light ID to water

        :param traffic_light_id: traffic light ID to add
        """
        self.traffic_lights.add(traffic_light_id)

    def dynamic_obstacle_by_time_step(self, time_step) -> Set[int]:
        """
        Returns all dynamic obstacles on water at specific time step

        :param time_step: time step of interest
        :returns: list of obstacle IDs
        """
        if self.dynamic_obstacles_on_water.get(time_step) is not None:
            return self.dynamic_obstacles_on_water.get(time_step)
        else:
            return set()

    def __str__(self):
        return 'Waters with id:' + str(self.waters_id)

    def add_traffic_light_to_water(self, traffic_light_id):
        pass


class WatersNetwork:
    """
    Class which represents a network of connected waters
    """

    def __init__(self):
        self._waters: Dict[int, Waters] = {}
        self._traffic_signs: Dict[int, TrafficSign] = {}
        self._traffic_lights: Dict[int, TrafficLight] = {}

    @property
    def waters(self) -> List[Waters]:
        return list(self._waters.values())

    @waters.setter
    def waters(self, waters: list):
        warnings.warn('<WatersNetwork/waters>: waters of network are immutable')

    @property
    def traffic_signs(self) -> List[TrafficSign]:
        return list(self._traffic_signs.values())

    @property
    def traffic_lights(self) -> List[TrafficLight]:
        return list(self._traffic_lights.values())

    @classmethod
    def create_from_waters_list(cls, waters: list, cleanup_ids: bool = False):
        """
        Creates a WatersNetwork object from a given list of waters

        :param waters: The list of waters
        :param cleanup_ids: cleans up unused ids
        :return: The WatersNetwork for the given list of waters
        """
        assert isinstance(waters, list) and all(isinstance(f, Waters) for f in
                                                waters), '<WatersNetwork/create_from_waters_list>:' \
                                                             'Provided list of waters is not valid! ' \
                                                             'waters = {}'.format(waters)

        # create water network
        waters_network = cls()

        # add each water to the water network
        for w in waters:
            waters_network.add_waters(copy.deepcopy(w))

        if cleanup_ids:
            waters_network.cleanup_fairway_references()
        return waters_network

    @classmethod
    def create_from_waters_network(cls, waters_network: 'WatersNetwork'):
        """
        Creates a water network from a given water network (copy)

        :param waters_network: The existing water network
        :return: The deep copy of the water network
        """
        new_waters_network = cls()
        for w in waters_network.waters:
            new_waters_network.add_waters(copy.deepcopy(w))
        return new_waters_network

    def find_waters_by_id(self, waters_id: int) -> Waters:
        """
        Finds a water for a given waters_id

        :param waters_id: The id of the water to find
        :return: The water object if the id exists and None otherwise
        """
        assert is_natural_number(
            waters_id), '<WatersNetwork/find_waters_by_id>: provided id is not valid! id = {}'.format(waters_id)

        return self._waters[waters_id] if waters_id in self._waters else None

    def find_traffic_sign_by_id(self, traffic_sign_id: int) -> TrafficSign:
        """
        Finds a traffic sign for a given traffic_sign_id

        :param traffic_sign_id: The id of the traffic sign to find
        :return: The traffic sign object if the id exists and None otherwise
        """
        assert is_natural_number(
            traffic_sign_id), '<WatersNetwork/find_traffic_sign_by_id>: provided id is not valid! ' \
                              'id = {}'.format(traffic_sign_id)

        return self._traffic_signs[traffic_sign_id] if traffic_sign_id in self._traffic_signs else None

    def find_traffic_light_by_id(self, traffic_light_id: int) -> TrafficLight:
        """
        Finds a traffic light for a given traffic_light_id

        :param traffic_light_id: The id of the traffic light to find
        :return: The traffic light object if the id exists and None otherwise
        """
        assert is_natural_number(
            traffic_light_id), '<WatersNetwork/find_traffic_light_by_id>: provided id is not valid! ' \
                               'id = {}'.format(traffic_light_id)

        return self._traffic_lights[traffic_light_id] if traffic_light_id in self._traffic_lights else None

    def add_waters(self, water: Waters):
        """
        Adds a waters to the WatersNetwork

        :param water: The waters to add
        :return: True if the waters has successfully been added to the network, false otherwise
        """

        assert isinstance(water, Waters), '<WatersNetwork/add_waters>: provided water is not of ' \
                                             'type water! type = {}'.format(type(water))

        # check if water already exists in network and warn user
        if water.waters_id in self._waters.keys():
            warnings.warn('Waters already exists in network! No changes are made.')
            return False
        else:
            self._waters[water.waters_id] = water
            return True

    def add_traffic_sign(self, traffic_sign: TrafficSign, waters_ids: Set[int]):
        """
        Adds a traffic sign to the WatersNetwork

        :param traffic_sign: The traffic sign to add
        :param waters_ids: Waters the traffic sign should be referenced from
        :return: True if the traffic sign has successfully been added to the network, false otherwise
        """

        assert isinstance(traffic_sign, TrafficSign), '<WatersNetwork/add_traffic_sign>: provided traffic sign is ' \
                                                      'not of type traffic_sign! type = {}'.format(type(traffic_sign))

        # check if traffic already exists in network and warn user
        if traffic_sign.traffic_sign_id in self._traffic_signs.keys():
            warnings.warn('Traffic sign with ID {} already exists in network! '
                          'No changes are made.'.format(traffic_sign.traffic_sign_id))
            return False
        else:
            self._traffic_signs[traffic_sign.traffic_sign_id] = traffic_sign
            for water_id in waters_ids:
                water = self.find_waters_by_id(water_id)
                if water is not None:
                    water.add_traffic_sign_to_water(traffic_sign.traffic_sign_id)
                else:
                    warnings.warn('Traffic sign cannot be referenced to water because the water does not exist.')
            return True

    def add_traffic_light(self, traffic_light: TrafficLight, waters_ids: Set[int]):
        """
        Adds a traffic light to the WatersNetwork

        :param traffic_light: The traffic light to add
        :param waters_ids: Waters the traffic sign should be referenced from
        :return: True if the traffic light has successfully been added to the network, false otherwise
        """

        assert isinstance(traffic_light, TrafficLight), '<WatersNetwork/add_traffic_light>: provided traffic light ' \
                                                        'is not of type traffic_light! ' \
                                                        'type = {}'.format(type(traffic_light))

        # check if traffic already exists in network and warn user
        if traffic_light.traffic_light_id in self._traffic_lights.keys():
            warnings.warn('Traffic light already exists in network! No changes are made.')
            return False
        else:
            self._traffic_lights[traffic_light.traffic_light_id] = traffic_light
            for waters_id in waters_ids:
                water = self.find_waters_by_id(waters_id)
                if water is not None:
                    water.add_traffic_light_to_water(traffic_light.traffic_light_id)
                else:
                    warnings.warn('Traffic light cannot be referenced to water because the water does not exist.')
            return True

    def add_waters_from_network(self, waters_network: 'WatersNetwork'):
        """
        Adds waters from a given network object to the current network

        :param waters_network: The water network
        :return: True if all waters have been added to the network, false otherwise
        """
        flag = True

        # add waters to the network
        for f in waters_network.waters:
            flag = flag and self.add_waters(f)

        return flag

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        Translates and rotates the complete waters network

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation,
                                     2), '<WatersNetwork/translate_rotate>: provided translation is not valid! ' \
                                         'translation = {}'.format(translation)
        assert is_valid_orientation(
            angle), '<WatersNetwork/translate_rotate>: provided angle is not valid! angle = {}'.format(angle)

        # rotate each water
        for water in self._waters.values():
            water.translate_rotate(translation, angle)
        for traffic_sign in self._traffic_signs.values():
            traffic_sign.translate_rotate(translation, angle)
        for traffic_light in self._traffic_lights.values():
            traffic_light.translate_rotate(translation, angle)

    def find_waters_by_position(self, point_list: List[np.ndarray]) -> List[List[int]]:
        """
        Finds the water id of a given position

        :param point_list: The list of positions to check
        :return: A list of water ids. If the position could not be matched to a water, an empty list is returned
        """
        assert isinstance(point_list,
                          ValidTypes.LISTS), '<Waters/contains_points>: provided list of points is not a list! ' \
                                             'type = {}'.format(type(point_list))
        # assert is_valid_polyline(
        #     point_list), 'Waters/contains_points>: provided list of points is malformed! points = {}'.format(
        #     point_list)

        # output list
        res = list()

        # look at each water
        polygons = [(f.waters_id, f.convert_to_polygon()) for f in self.waters]

        for point in point_list:
            mapped = list()
            for waters_id, poly in polygons:
                if poly.contains_point(point):
                    mapped.append(waters_id)
            res.append(mapped)

        return res

    def find_water_by_shape(self, shape: Shape) -> List[int]:
        """
        Finds the water id of a given shape

        :param shape: The shape to check
        :return: A list of water ids. If the position could not be matched to a water, an empty list is returned
        """
        assert isinstance(shape, (Circle, Polygon, Rectangle)), '<Waters/find_water_by_shape>: ' \
                                                                'provided shape is not a shape! ' \
                                                                'type = {}'.format(type(shape))

        # output list
        res = []

        # look at each water
        polygons = [(l.waters_id, l.convert_to_polygon()) for l in self.waters]

        for waters_id, poly in polygons:
            if poly.shapely_object.intersects(shape.shapely_object):
                res.append(waters_id)

        return res

    def filter_obstacles_in_network(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        """
        Returns the list of obstacles which are located in the water network

        :param obstacles: The list of obstacles to check
        :return: The list of obstacles which are located in the water network
        """

        res = list()

        map = self.map_obstacles_to_waters(obstacles)

        for k in map.keys():
            obs = map[k]
            for o in obs:
                if o not in res:
                    res.append(o)

        return res

    def map_obstacles_to_waters(self, obstacles: List[Obstacle]) -> Dict[int, List[Obstacle]]:
        """
        Maps a given list of obstacles to the waters of the water network

        :param obstacles: The list of CR obstacles
        :return: A dictionary with the water id as key and the list of obstacles on the water as a List[Obstacles]
        """
        mapping = {}

        for f in self.waters:
            # map obstacles to current water
            mapped_objs = f.get_obstacles(obstacles)

            # check if mapping is not empty
            if len(mapped_objs) > 0:
                mapping[f.waters_id] = mapped_objs

        return mapping

    def waters_in_proximity(self, point: list, radius: float) -> List[Waters]:
        """
        Finds all waters which intersect a given circle, defined by the center point and radius

        :param point: The center of the circle
        :param radius: The radius of the circle
        :return: The list of waters which intersect the given circle
        """

        assert is_real_number_vector(point,
                                     length=2), '<WatersNetwork/waters_in_proximity>: provided point is ' \
                                                'not valid! point = {}'.format(point)
        assert is_positive(
            radius), '<WatersNetwork/waters_in_proximity>: provided radius is not valid! radius = {}'.format(radius)

        # get list of water ids
        ids = self._waters.keys()

        # output list
        lanes = dict()

        rad_sqr = radius ** 2

        # distance dict for sorting
        distance_list = list()

        # go through list of waters
        for i in ids:

            # if current water has not already been added to lanes list
            if i not in lanes:
                water = self.find_waters_by_id(i)

                # compute distances (we are not using the sqrt for computational effort)
                distance = (water.center_vertices - point) ** 2.
                distance = distance[:, 0] + distance[:, 1]

                # check if at least one distance is smaller than the radius
                if any(np.greater_equal(rad_sqr, distance)):
                    lanes[i] = self.find_waters_by_id(i)
                    distance_list.append(np.min(distance))

                # check if adjacent waters can be added as well
                index_minDist = np.argmin(distance - rad_sqr)

        # sort list according to distance
        indices = np.argsort(distance_list)
        water = list(lanes.values())

        # return sorted list
        return [water[i] for i in indices]

    def __str__(self):
        return_str = ''
        for water_id in self._waters.keys():
            return_str += '{:8d} waters\n'.format(water_id)
        return return_str
