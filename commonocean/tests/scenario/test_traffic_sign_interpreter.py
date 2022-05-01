import unittest
import numpy as np

from commonocean.scenario.traffic_sign import TrafficSign, TrafficSignElement

from commonocean.scenario.waters import Waters, WatersNetwork, WatersType, WatersUser

from commonroad.scenario.traffic_sign_interpreter import TrafficSigInterpreter

# Test file not necessary for commonocean (perhaps delete ?) - It doesn't use the trafficsign


class TestTrafficSigInterpreter(unittest.TestCase):
    def setUp(self):
        waters_one = Waters(left_vertices=np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]),
                              center_vertices=np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                              right_vertices=np.array([[0.0, 2], [1.0, 2], [2, 2]]), waters_id=100,
                              waters_type={WatersType.WATER, WatersType.OPENSEA}) # ,
                              # user_one_way={WatersUser.VESSEL})
        waters_two = Waters(left_vertices=np.array([[0.0, 2.0], [1.0, 2.0], [2, 3]]),
                              center_vertices=np.array([[0.0, 3], [1.0, 3], [2, 3]]),
                              right_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]), waters_id=101,
                              waters_type={WatersType.WATER, WatersType.OPENSEA}) # ,
                              # user_one_way={WatersUser.VESSEL})
        waters_three = Waters(left_vertices=np.array([[0.0, 3.0], [1.0, 3.0], [2, 3]]),
                                center_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]),
                                right_vertices=np.array([[0.0, 5], [1.0, 5], [2, 5]]), waters_id=102,
                                waters_type={WatersType.WATER, WatersType.OPENSEA}) # ,
                                # user_one_way={WatersUser.VESSEL})
        waters_four = Waters(left_vertices=np.array([[0.0, 3.0], [1.0, 3.0], [2, 3]]),
                               center_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]),
                               right_vertices=np.array([[0.0, 5], [1.0, 5], [2, 5]]), waters_id=103,
                               waters_type={WatersType.WATER, WatersType.OPENSEA}) # ,
                               # user_one_way={WatersUser.VESSEL})
        waters_five = Waters(left_vertices=np.array([[0.0, 3.0], [1.0, 3.0], [2, 3]]),
                               center_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]),
                               right_vertices=np.array([[0.0, 5], [1.0, 5], [2, 5]]), waters_id=104,
                               waters_type={WatersType.WATER, WatersType.OPENSEA}) # ,
                               # user_one_way={WatersUser.VESSEL})

        # This part is working, except the fact that a TrafficSignID for CommonOcean is absent

        # traffic_sign_one = TrafficSign(201, [TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED, ["20"])], {100},
        #                                np.array([0.0, 2]))
        # traffic_sign_two = TrafficSign(202, [TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED, ["30"])], {101},
        #                                np.array([0.0, 4]))
        # traffic_sign_three = TrafficSign(203, [TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED, ["40"])], {102},
        #                                  np.array([0.0, 5]))
        # traffic_sign_four = TrafficSign(204, [TrafficSignElement(TrafficSignIDZamunda.MIN_SPEED, ["50"])], {103},
        #                                 np.array([0.0, 5]))

        waters_network = WatersNetwork().create_from_waters_list([waters_one, waters_two, waters_three,
                                                                     waters_four, waters_five])
        # waters_network.add_traffic_sign(traffic_sign_one, {100})
        # waters_network.add_traffic_sign(traffic_sign_two, {101})
        # waters_network.add_traffic_sign(traffic_sign_three, {102})
        # waters_network.add_traffic_sign(traffic_sign_four, {103})

        # self.interpreter = TrafficSigInterpreter(SupportedTrafficSignCountry.ZAMUNDA, lanelet_network)


    # How to approach this part? Necessary a SupportedTrafficSignCountry

    # def test_speed_limit(self):
    #     self.assertEqual(20, self.interpreter.speed_limit(frozenset({100, 101, 102})))
    #     self.assertEqual(20, self.interpreter.speed_limit(frozenset({101, 100, 102})))
    #     self.assertEqual(20, self.interpreter.speed_limit(frozenset({102, 100, 101})))
    #     self.assertEqual(30, self.interpreter.speed_limit(frozenset({101, 102})))
    #     self.assertEqual(None, self.interpreter.speed_limit(frozenset({103, 104})))
    #     self.assertEqual(40, self.interpreter.speed_limit(frozenset({102, 104})))
    #     self.assertEqual(None, self.interpreter.required_speed(frozenset({102, 104})))
    #     self.assertEqual(50, self.interpreter.required_speed(frozenset({103, 104})))


if __name__ == '__main__':
    unittest.main()
