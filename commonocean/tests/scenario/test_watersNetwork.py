import copy
import unittest
import numpy as np


from commonocean.scenario.waters import Waters, WatersNetwork, WatersType
from commonocean.scenario.obstacle import StaticObstacle, ObstacleType
from commonocean.scenario.traffic_sign import TrafficSignElement, TrafficSign, TrafficLight, TrafficLightCycleElement, TrafficLightState

from commonroad.scenario.trajectory import State
# from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement
from commonroad.geometry.shape import Rectangle

# from commonroad.scenario.lanelet import Lanelet, LineMarking, LaneletNetwork, StopLine, LaneletType
# from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
# from commonroad.scenario.traffic_sign import TrafficSignElement, TrafficSign, TrafficSignIDGermany, \
#     TrafficLight, TrafficLightCycleElement, TrafficLightState
# from commonroad.scenario.trajectory import State
# from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement
# from commonroad.geometry.shape import Rectangle

__author__ = "Bruno Maione"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2022.1"
__email__ = "bruno.maione@tum.de"
__status__ = "Development"

class TestLaneletNetwork(unittest.TestCase):

    def setUp(self):
        self.right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        self.left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        self.center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5],
                                         [8, .5]])
        self.waters_id = 5
        self.predecessor = [1, 2]
        self.successor = [6, 7]
        # self.adjacent_left = 12
        # self.adjacent_right = 4
        # self.adjacent_right_same_dir = True
        # self.adjacent_left_same_dir = False
        # self.line_marking_right = LineMarking.SOLID
        # self.line_marking_left = LineMarking.DASHED
        # traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED.value, ["15"])
        # self.traffic_sign = TrafficSign(1, [traffic_sign_max_speed], {5}, np.array([0.0, 0.0]))
        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2),
                 TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
                 TrafficLightCycleElement(TrafficLightState.RED, 2)]
        self.traffic_light = TrafficLight(567, cycle, position=np.array([10., 10.]))
        # self.stop_line = StopLine(self.left_vertices[-1], self.right_vertices[-1], LineMarking.SOLID,
        #                           {self.traffic_sign.traffic_sign_id}, {self.traffic_light.traffic_light_id})

        # incoming_1 = IntersectionIncomingElement(2, {self.waters_id, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        # incoming_2 = IntersectionIncomingElement(3, {20, 21}, {22, 23}, {24, 25}, {26, 27}, 28)
        # self.intersection = Intersection(1, [incoming_1, incoming_2], {30, 31})

        self.waters = Waters(self.left_vertices, self.center_vertices, self.right_vertices, self.waters_id,
                               self.predecessor, self.successor, traffic_lights={self.traffic_light.traffic_light_id})

        self.waters_2 = Waters(np.array([[8, 1], [9, 1]]), np.array([[8, .5], [9, .5]]), np.array([[8, 0], [9, 0]]),
                                 6, [self.waters.waters_id], [678])

        self.waters_network = WatersNetwork()
        self.waters_network.add_waters(self.waters)
        self.waters_network.add_waters(self.waters_2)
        # self.waters_network.add_traffic_sign(self.traffic_sign, set())
        self.waters_network.add_traffic_light(self.traffic_light, set())
        # self.waters_network.add_intersection(self.intersection)

        self.diagonal_waters_network = WatersNetwork()
        waters_width = np.array([0.0, 3.0])
        right_vertices = np.array([[0., 0.], [6., 0.1], [12., 0.5]])
        left_vertices = copy.copy(right_vertices) + waters_width
        center_vertices = (right_vertices + left_vertices) * 0.5
        waters_id = 0
        self.diagonal_waters_network.add_waters(Waters(left_vertices, center_vertices, right_vertices, waters_id))

        left_vertices = copy.copy(right_vertices)
        right_vertices = copy.copy(left_vertices) - waters_width
        center_vertices = (right_vertices + left_vertices) * 0.5
        waters_id = 1
        self.diagonal_waters_network.add_waters(Waters(left_vertices, center_vertices, right_vertices, waters_id))

    def test_initialize_waters(self):
        s1 = np.sqrt(1.25)
        s2 = np.sqrt(2.0)
        desired_dist = [0.0, 1.0, 2.0, 2.0 + s1, 2.0 + 2 * s1, 3.0 + 2 * s1, 4.0 + 2 * s1, 4.0 + 2 * s1 + s2,
                        5.0 + 2 * s1 + s2]
        for i, dist in enumerate(self.waters.distance):
            self.assertAlmostEqual(dist, desired_dist[i])

        self.assertEqual(self.waters.waters_id, self.waters_id)
        np.testing.assert_array_almost_equal(self.waters.right_vertices, self.right_vertices)
        np.testing.assert_array_almost_equal(self.waters.left_vertices, self.left_vertices)
        np.testing.assert_array_almost_equal(self.waters.center_vertices, self.center_vertices)
        np.testing.assert_array_almost_equal(self.waters.predecessor, self.predecessor)
        np.testing.assert_array_almost_equal(self.waters.successor, self.successor)
        # self.assertEqual(self.waters.adj_left, self.adjacent_left)
        # self.assertEqual(self.waters.adj_right, self.adjacent_right)
        # self.assertEqual(self.waters.adj_left_same_direction, self.adjacent_left_same_dir)
        # self.assertEqual(self.waters.adj_right_same_direction, self.adjacent_right_same_dir)
        # self.assertEqual(self.waters.line_marking_left_vertices, self.line_marking_left)
        # self.assertEqual(self.waters.line_marking_right_vertices, self.line_marking_right)

        self.assertEqual(self.waters_network.waters[0].waters_id, self.waters.waters_id)

    def test_create_from_waters_network(self):
        waters_network = WatersNetwork()

        right_vertices = np.array([[0, 0], [1, 0], [1.1, 0.1]])
        left_vertices = np.array([[0, 1], [1, 1], [1.1, 1.1]])
        center_vertices = np.array([[0, .5], [1, .5], [1.1, .6]])
        waters_id = 5
        waters1 = Waters(left_vertices, center_vertices, right_vertices, waters_id)
        waters_network.add_waters(waters1)
        # waters_network.add_traffic_sign(self.traffic_sign, {waters1.waters_id})
        waters_network.add_traffic_light(self.traffic_light, {waters1.waters_id})

        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5]])
        waters_id = 6
        waters_type = {WatersType.WATER}
        waters2 = Waters(left_vertices, right_vertices, center_vertices, waters_id, None, None, waters_type, None, {self.traffic_light.traffic_light_id})
        waters_network.add_waters(waters2)

        right_vertices = np.array([[5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        waters_id = 7
        waters3 = Waters(left_vertices, center_vertices, right_vertices, waters_id)
        waters_network.add_waters(waters3)

        new_network = waters_network.create_from_waters_network(waters_network)

        a = False
        for i in range(0, len(new_network.waters)):
            if waters1.waters_id == new_network.waters[i].waters_id:
                a = True
        self.assertTrue(a)

        a = False
        for i in range(0, len(new_network.waters)):
            if waters2.waters_id == new_network.waters[i].waters_id:
                a = True
        self.assertTrue(a)

        a = False
        for i in range(0, len(new_network.waters)):
            if waters3.waters_id == new_network.waters[i].waters_id:
                a = True
        self.assertFalse(a)

        # self.assertEqual(waters2.traffic_signs, {1})
        self.assertEqual(waters2.traffic_lights, {567})
        # self.assertEqual(waters1.traffic_signs, {1})
        self.assertEqual(waters1.traffic_lights, {567})

        waters_in_network = []
        for i in range(0, len(new_network.waters)):
            waters_in_network.append(new_network.waters[i])
        self.assertNotIn(waters3.waters_id, waters_in_network)

        new_network_waters_types = waters_network.create_from_waters_network(waters_network, Rectangle(2, 2),
                                                                                {WatersType.WATER})
        waters_in_network = []
        for i in range(0, len(new_network_waters_types.waters)):
            waters_in_network.append(new_network_waters_types.waters[i])

        self.assertNotIn(waters2.waters_id, waters_in_network)
        # self.assertEquals(waters1.traffic_signs, new_network_waters_types.waters[0].traffic_signs)
        self.assertEquals(waters1.traffic_lights, new_network_waters_types.waters[0].traffic_lights)

    def create_from_waters_list(self):
        new_network = WatersNetwork.create_from_waters_list([self.waters])

        for waters_act, waters_des in zip(new_network.waters, self.waters_network.waters):
            np.testing.assert_array_almost_equal(waters_act.right_vertices, waters_des.right_vertices)
            np.testing.assert_array_almost_equal(waters_act.center_vertices, waters_des.center_vertices)
            np.testing.assert_array_almost_equal(waters_act.left_vertices, waters_des.left_vertices)
            self.assertEqual(waters_act.waters_id, waters_des.waters_id)

    def test_find_waters_by_id(self):
        actual_waters_found = self.waters_network.find_waters_by_id(5)

        np.testing.assert_array_almost_equal(actual_waters_found.right_vertices, self.waters.right_vertices)
        np.testing.assert_array_almost_equal(actual_waters_found.center_vertices, self.waters.center_vertices)
        np.testing.assert_array_almost_equal(actual_waters_found.left_vertices, self.waters.left_vertices)
        self.assertEqual(actual_waters_found.waters_id, self.waters.waters_id)
        self.assertEqual(self.waters_network.find_waters_by_id(2), None)

    def test_add_waters(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        waters = Waters(left_vertices, center_vertices, right_vertices, 2)

        self.assertTrue(self.waters_network.add_waters(waters))

        np.testing.assert_array_almost_equal(self.waters_network.waters[0].right_vertices,
                                             self.waters.right_vertices)
        np.testing.assert_array_almost_equal(self.waters_network.waters[0].center_vertices,
                                             self.waters.center_vertices)
        np.testing.assert_array_almost_equal(self.waters_network.waters[0].left_vertices,
                                             self.waters.left_vertices)
        self.assertEqual(self.waters_network.waters[0].waters_id, self.waters.waters_id)

    # Not possible in CommonOcean at the moment

    # def test_add_traffic_sign(self):
    #     traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED.value, ["15"])
    #     traffic_sign = TrafficSign(123, [traffic_sign_max_speed], {5}, np.array([0.0, 0.0]))

    #     self.assertTrue(self.lanelet_network.add_traffic_sign(traffic_sign, {5}))

    #     self.assertEqual(self.lanelet_network.traffic_signs[1].traffic_sign_id, traffic_sign.traffic_sign_id)
    #     self.assertSetEqual(self.lanelet_network.lanelets[0].traffic_signs, {123, 1})

    def test_add_traffic_light(self):
        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2),
                 TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
                 TrafficLightCycleElement(TrafficLightState.RED, 2)]
        traffic_light = TrafficLight(234, cycle, time_offset=5, position=np.array([10., 10.]))

        self.assertTrue(self.waters_network.add_traffic_light(traffic_light, {5}))

        self.assertEqual(self.waters_network.traffic_lights[1].traffic_light_id, traffic_light.traffic_light_id)
        self.assertSetEqual(self.waters_network.waters[0].traffic_lights, {234, 567})

    def test_add_waters_from_network(self):

        actual_network = WatersNetwork()
        actual_network.add_waters_from_network(self.waters_network)

        for waters_act, waters_des in zip(actual_network.waters, self.waters_network.waters):
            np.testing.assert_array_almost_equal(waters_act.right_vertices, waters_des.right_vertices)
            np.testing.assert_array_almost_equal(waters_act.center_vertices, waters_des.center_vertices)
            np.testing.assert_array_almost_equal(waters_act.left_vertices, waters_des.left_vertices)
            self.assertEqual(waters_act.waters_id, waters_des.waters_id)

    def test_translate_rotate(self):

        self.waters_network.translate_rotate(np.array([2, -4]), np.pi / 2)

        desired_waters_center = np.array([[3.5, 2], [3.5, 3], [3.5, 4], [3, 5], [2.5, 6], [2.5, 7], [2.5,  8],
                                           [3.5, 9], [3.5, 10]])
        # desired_traffic_sign_position = np.array([4, 2])

        np.testing.assert_array_almost_equal(self.waters_network.waters[0].center_vertices, desired_waters_center)
        # np.testing.assert_array_almost_equal(self.waters_network.traffic_signs[0].position,
                                            #  desired_traffic_sign_position)

    def test_translate_invalid(self):

        with self.assertRaises(AssertionError):
            self.waters_network.translate_rotate(np.array([2, -4]), 320)
        with self.assertRaises(AssertionError):
            self.waters_network.translate_rotate(np.array([3, 5, -7]), np.pi / 2)
        with self.assertRaises(AssertionError):
            self.waters_network.translate_rotate(np.array([3]), np.pi / 2)
        with self.assertRaises(AssertionError):
            self.waters_network.translate_rotate(0.0, np.pi / 2)

    def test_find_waters_by_position(self):
        additional_waters_network = WatersNetwork.create_from_waters_network(self.waters_network)

        observed_waters = self.waters_network.find_waters_by_position([np.array([1, 1])])
        self.assertEqual(observed_waters[0][0], self.waters.waters_id)
        self.assertEqual(len(self.waters_network.find_waters_by_position([np.array([-5, -5])])[0]), 0)

        observed_waters = additional_waters_network.find_waters_by_position([np.array([1, 1])])
        self.assertEqual(observed_waters[0][0], self.waters.waters_id)
        self.assertEqual(len(additional_waters_network.find_waters_by_position([np.array([-5, -5])])[0]), 0)

        tolerance = 1e-14

        def assert_pos(vertex, waters_id_list):
            [ret_list] = self.diagonal_waters_network.find_waters_by_position([vertex])
            ret_list.sort()
            waters_id_list.sort()
            self.assertEqual(ret_list, waters_id_list)

        waters_0 = self.diagonal_waters_network.find_waters_by_id(0)
        waters_1 = self.diagonal_waters_network.find_waters_by_id(1)
        for dist_i in list(np.linspace(0.0, waters_0.distance[2], 1000)):
            center_vertex, right_vertex, left_vertex, _ = waters_0.interpolate_position(dist_i)
            assert_pos(left_vertex, [0])
            assert_pos(left_vertex + np.array([0.0, 0.5 * tolerance]), [])

            assert_pos(center_vertex, [0])

            assert_pos(right_vertex, [0, 1])
            assert_pos(right_vertex + np.array([0.0, 0.5 * tolerance]), [0])
            assert_pos(right_vertex - np.array([0.0, 0.5 * tolerance]), [1])

            center_vertex, right_vertex, left_vertex, _ = waters_1.interpolate_position(dist_i)
            assert_pos(left_vertex, [0, 1])
            assert_pos(left_vertex + np.array([0.0, 0.5 * tolerance]), [0])
            assert_pos(left_vertex - np.array([0.0, 0.5 * tolerance]), [1])

            assert_pos(center_vertex, [1])

            assert_pos(right_vertex, [1])
            assert_pos(right_vertex - np.array([0.0, 0.5 * tolerance]), [])

        assert_pos(np.array([-tolerance, 0.]), [])
        assert_pos(np.array([waters_0.center_vertices[-1][0] + tolerance, 0.]), [])

    def test_find_waterr_by_shape(self):
        rectangle1 = Rectangle(2, 2)
        rectangle2 = Rectangle(2, 2, np.array([100.0, 100.0]))
        rectangle3 = Rectangle(2, 2, np.array([9.0, 0.0]))

        observed_waters = self.waters_network.find_water_by_shape(rectangle1)
        self.assertEqual(observed_waters[0], self.waters.waters_id)
        observed_waters = self.waters_network.find_water_by_shape(rectangle2)
        self.assertEqual(observed_waters, [])
        observed_waters_pl = self.waters_network.find_water_by_shape(rectangle3)
        self.assertEqual([self.waters_2.waters_id, self.waters.waters_id], observed_waters_pl)

    def test_filter_obstacles_in_network_positive_map_obstacles_to_waters_postive(self):
        initial_state = State(**{'position': np.array([0, 0]), 'orientation': 0.0})
        rect_shape = Rectangle(2, 2)
        expected_obstacle = StaticObstacle(obstacle_id=1, obstacle_type=ObstacleType.MOTORVESSEL, obstacle_shape=rect_shape,
                                           initial_state=initial_state)

        self.waters_network.map_obstacles_to_waters([expected_obstacle])
        actual_obstacles = self.waters_network.filter_obstacles_in_network([expected_obstacle])

        self.assertEqual(len(actual_obstacles), 1)
        self.assertEqual(actual_obstacles[0].obstacle_id, 1)
        self.assertEqual(actual_obstacles[0].obstacle_type, ObstacleType.MOTORVESSEL)

    def test_filter_obstacles_in_network_positive_map_obstacles_to_waters_negative(self):
        initial_state = State(**{'position': np.array([-50, -50]), 'orientation': 0.0})
        rect_shape = Rectangle(2, 2)
        expected_obstacle = StaticObstacle(obstacle_id=1, obstacle_type=ObstacleType.MOTORVESSEL, obstacle_shape=rect_shape,
                                           initial_state=initial_state)

        self.waters_network.map_obstacles_to_waters([expected_obstacle])
        actual_obstacles = self.waters_network.filter_obstacles_in_network([expected_obstacle])

        self.assertEqual(len(actual_obstacles), 0)

    def test_waters_in_proximity(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        waters = Waters(left_vertices, center_vertices, right_vertices, 1)
        waters_network = WatersNetwork()
        waters_network.add_waters(waters)

        radius = 5.0
        point = np.array([0, 0])

        actual_in_proximity = waters_network.waters_in_proximity(point, radius)

        self.assertTrue(len(actual_in_proximity), 1)


    # Not present in CommonOcean at the moment

    # def test_remove_waters(self):
    #     self.waters_network.remove_waters(123456789)  # delete non-existing waters
    #     self.assertEqual(len(self.waters_network.waters), 2)

    #     self.waters_network.remove_waters(self.waters.waters_id)   # delete existing waters
    #     self.assertEqual(len(self.waters_network.waters), 1)

    # def test_remove_traffic_sign(self):
    #     self.waters_network.remove_traffic_sign(123456789)  # delete non-existing traffic sign
    #     self.assertEqual(len(self.waters_network.traffic_signs), 1)

    #     self.waters_network.remove_traffic_sign(self.traffic_sign.traffic_sign_id)  # delete existing traffic sign
    #     self.assertEqual(len(self.waters_network.traffic_signs), 0)

    # def test_remove_traffic_light(self):
    #     self.waters_network.remove_traffic_light(123456789)  # delete non-existing traffic light
    #     self.assertEqual(len(self.waters_network.traffic_lights), 1)

    #     self.waters_network.remove_traffic_light(self.traffic_light.traffic_light_id)  # delete existing traffic light
    #     self.assertEqual(len(self.waters_network.traffic_lights), 0)

    # def test_remove_intersection(self):
    #     self.waters_network.remove_intersection(123456789)  # delete non-existing intersection
    #     self.assertEqual(len(self.waters_network.intersections), 1)

    #     self.waters_network.remove_intersection(self.intersection.intersection_id)  # delete existing traffic light
    #     self.assertEqual(len(self.waters_network.intersections), 0)

    # def test_cleanup_waters_references(self):
    #     # intersection contains dummy references which will be deleted during cleanup
    #     self.assertEqual(len(self.intersection.incomings[0].incoming_lanelets), 2)
    #     self.waters_network.remove_waters(self.waters.waters_id)  # delete existing traffic sign
    #     self.assertEqual(len(self.waters_2.predecessor), 0)
    #     self.assertEqual(len(self.intersection.incomings[0].incoming_lanelets), 0)

    # def test_cleanup_traffic_light_references(self):
    #     self.waters_network.remove_traffic_light(self.traffic_light.traffic_light_id)  # delete existing traffic light
    #     self.assertEqual(len(self.waters.traffic_lights), 0)
    #     self.assertEqual(len(self.waters.stop_line.traffic_light_ref), 0)

    # def test_cleanup_traffic_sign_references(self):
    #     self.waters_network.remove_traffic_sign(self.traffic_sign.traffic_sign_id)  # delete existing traffic sign
    #     self.assertEqual(len(self.waters.traffic_signs), 0)
    #     self.assertEqual(len(self.waters.stop_line.traffic_sign_ref), 0)

    def test_equality_hash(self):
        left_vertices = np.array([[0, 1], [1, 1], [2, 1]])
        center_vertices = np.array([[0, 0], [1, 0], [2, 0]])
        right_vertices = np.array([[0, -1], [1, -1], [2, -1]])
        waters_id = 3
        waters = Waters(left_vertices, center_vertices, right_vertices, waters_id)

        # incoming = IntersectionIncomingElement(2, {9, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        # intersection = Intersection(1, [incoming], {30, 31})

        # traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15"])
        # traffic_sign = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0]))

        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2)]
        traffic_light = TrafficLight(234, cycle, np.array([10., 10.]), 5)

        waters_network_1 = WatersNetwork()
        waters_network_2 = WatersNetwork()
        for waters_network in [waters_network_1, waters_network_2]:
            waters_network.add_waters(waters)
            # waters_network.add_intersection(intersection)
            # waters_network.add_traffic_sign(traffic_sign, {3})
            waters_network.add_traffic_light(traffic_light, {3})
        self.assertTrue(waters_network_1 == waters_network_2)
        self.assertTrue(hash(waters_network_1), hash(waters_network_2))

        # waters_network_2.remove_lanelet(3)
        # self.assertFalse(waters_network_1 == waters_network_2)
        # self.assertNotEqual(hash(waters_network_1), hash(waters_network_2))

        waters_network_2.add_waters(waters)
        # waters_network_2.add_intersection(Intersection(2, [incoming], {30, 31}))
        self.assertFalse(waters_network_1 == waters_network_2)
        self.assertNotEqual(hash(waters_network_1), hash(waters_network_2))

        # waters_network_2.remove_intersection(2)
        # waters_network_2.add_traffic_sign(TrafficSign(4, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0])), {3})
        # self.assertFalse(waters_network_1 == waters_network_2)
        # self.assertNotEqual(hash(waters_network_1), hash(waters_network_2))

        # waters_network_2.remove_traffic_sign(4)
        # waters_network_2.remove_traffic_light(234)
        # self.assertFalse(waters_network_1 == waters_network_2)
        # self.assertNotEqual(hash(waters_network_1), hash(waters_network_2))


if __name__ == '__main__':
    unittest.main()
