import os
import warnings

import matplotlib as mpl
import pytest
# from commonocean.scenario.traffic_sign import TrafficSign, \
#     TrafficSignElement, \
#     TrafficSignElementID

#     # TrafficSignIDUsa, \
#     # TrafficSignIDZamunda, \
#     # TrafficSignIDRussia, \
#     # TrafficSignIDSpain, \
#     # TrafficSignIDChina

from commonocean.common.file_reader import CommonOceanFileReader
import numpy as np

# mpl.use('TkAgg')
import matplotlib.pyplot as plt
import unittest
from commonroad.visualization.param_server import ParamServer
from commonroad.visualization.mp_renderer import MPRenderer


###### Test file not necessary for commonocean (perhaps delete ?) - It doesn't use the intersection
######  To take this test, we need the xml files associated with the paths used

class TestVisualizationIntersection(unittest.TestCase):
    def setUp(self):
        self.full_path = os.path.dirname(os.path.abspath(__file__))
        self.filename_urban = os.path.join(self.full_path,
                                           '../test_scenarios/test_reading_intersection_traffic_sign.xml')
        self.filename_complex_tl = os.path.join(self.full_path,
                                                '../test_scenarios/test_reading_complex_tl.xml')
        self.filename_lanelet = os.path.join(self.full_path,
                                             '../test_scenarios/test_reading_lanelets.xml')
        self.filename_test_all = os.path.join(self.full_path,
                                              '../test_scenarios/test_reading_all.xml')
        self.rnd = MPRenderer()
        self.legend = {
                ('lanelet_network', 'intersection',
                 'incoming_lanelets_color'):   'Incoming lanelets',
                ('lanelet_network', 'intersection',
                 'successors_left_color'):     'Successors left',
                ('lanelet_network', 'intersection',
                 'successors_straight_color'): 'Successors straight',
                ('lanelet_network', 'intersection',
                 'successors_right_color'):    'Successors right',
                ('lanelet_network', 'traffic_light',
                 'green_color'):               'Traffic light green',
                ('lanelet_network', 'traffic_light',
                 'yellow_color'):              'Traffic light yellow',
                ('lanelet_network', 'traffic_light',
                 'red_color'):                 'Traffic light red'
        }

## Not relevant for CommonOcean until now

#     def test_intersection_plot(self):
#         """Uses all options for plotting objects related to intersections or
#         traffic sign/lights."""
#         scenario, pp = CommonOceanFileReader(self.filename_urban).open()
#         plt.close('all')
#         mpl.rcParams['lines.scale_dashes'] = False
#         scenario.lanelet_network.draw(self.rnd, draw_params={
#                 'time_begin': 20, 'lanelet_network': {
#                         'draw_intersections': True,
#                         'draw_traffic_signs': True,
#                         'intersection':       {'show_label': True}
#                 }, 'lanelet': {
#                         'draw_lane_marking': True, 'show_label': True
#                 }
#         })
#         self.rnd.add_legend(self.legend)
#         self.rnd.render(show=True)

#     def test_traffic_signs(self):
#         """Uses all options for plotting objects related to intersections or
#         traffic sign/lights."""
#         scenario, pp = CommonOceanFileReader(self.filename_urban).open()
#         plt.close('all')
#         draw_params = {
#                 'time_begin': 20, 'lanelet_network': {
#                         'intersection': {'draw_intersections': False},
#                         'traffic_sign': {
#                                 'draw_traffic_signs': True,
#                                 'show_label':         False,
#                                 'show_traffic_signs': 'all',
#                                 'scale_factor':       1.0
#                         }
#                 }, 'lanelet': {
#                         'draw_lane_marking': False, 'show_label': False
#                 }
#         }
#         scenario.lanelet_network.draw(self.rnd, draw_params=draw_params)
#         ts = TrafficSign(traffic_sign_id=100000, traffic_sign_elements=[
#             TrafficSignElement(TrafficSignElementID.MAX_WIDTH,
#                                additional_values=[str(3)]),
#             TrafficSignElement(TrafficSignElementID.MAX_SPEED_ZONE_START,
#                                additional_values=[str(30/3.6)]),
#             TrafficSignElement(TrafficSignElementID.ADDITION_VALID_IN_X_KILOMETERS,
#                                additional_values=[str(3)]),
#             TrafficSignElement(TrafficSignElementID.ADDITION_VALID_FOR_X_METERS,
#                                additional_values=[str(3)]),
#             TrafficSignElement(TrafficSignElementID.ADDITION_TIME_PERIOD_PERMITTED,
#                                additional_values=[str(3)]),
#             # TrafficSignElement(TrafficSignElementID.MAX_LENGTH,
#             #                    additional_values=[str(3)]),
#             #     TrafficSignElement(TrafficSignIDUsa.MAX_SPEED,
#             #                        additional_values=[str(50 / 2.23694)]),
#             #     TrafficSignElement(TrafficSignElementID.MIN_SPEED,
#             #                        additional_values=[str(30 / 3.6)]),
#             #     TrafficSignElement(TrafficSignElementID.MAX_SPEED,
#             #                        additional_values=[str(80 / 3.6)]),
#             #     TrafficSignElement(TrafficSignElementID.MAX_SPEED,
#             #                        additional_values=[str(130 / 3.6)]),
#             #     TrafficSignElement(TrafficSignElementID.NO_OVERTAKING_START,
#             #                        additional_values=['test']),
#             #     TrafficSignElement(TrafficSignElementID.MAX_HEIGHT,
#             #                        additional_values=['80']),
#             TrafficSignElement(TrafficSignElementID.MAX_WEIGHT,
#                             additional_values=['80'])],
#                             position=np.array([159., -88.]), virtual=False,
#                             first_occurrence=set())
#         ts.draw(self.rnd, draw_params={
#                 'traffic_sign': {
#                         'speed_limit_unit': 'auto',
#                         'scale_factor':     1.0,
#                         'kwargs':           {
#                                 'arrowprops': {'arrowstyle': "simple"}
#                         }
#                 }
#         })

#         self.rnd.render(show=True)



    # def test_all_signs(self):
    #     """Check if all traffic signs can be plotted."""
    #     kwargs = {
    #             'traffic_sign_id':  1,
    #             'first_occurrence': 1,
    #             'position':         np.array([0., 0.])
    #     }
    #     plt.close('all')
    #     plt.figure(figsize=[10, 10])
    #     self.rnd = MPRenderer(ax=plt.gca())
    #     with pytest.warns(None) as record:
    #         for value in TrafficSignIDGermany:
    #             kwargs['position'] = kwargs['position'] + np.array([0., 5.])
    #             TrafficSign(traffic_sign_elements=[
    #                     TrafficSignElement(value, ['foo'])], **kwargs).draw(
    #                 self.rnd)

    #         for value in TrafficSignIDUsa:
    #             kwargs['position'] = kwargs['position'] + np.array([0., 5.])
    #             TrafficSign(traffic_sign_elements=[
    #                     TrafficSignElement(value, ['foo'])], **kwargs).draw(
    #                 self.rnd)

    #         for value in TrafficSignIDRussia:
    #             kwargs['position'] = kwargs['position'] + np.array([0., 5.])
    #             TrafficSign(traffic_sign_elements=[
    #                     TrafficSignElement(value, ['foo'])], **kwargs).draw(
    #                 self.rnd)

    #         for value in TrafficSignIDSpain:
    #             kwargs['position'] = kwargs['position'] + np.array([0., 5.])
    #             TrafficSign(traffic_sign_elements=[
    #                     TrafficSignElement(value, ['foo'])], **kwargs).draw(
    #                 self.rnd)

    #         for value in TrafficSignIDChina:
    #             kwargs['position'] = kwargs['position'] + np.array([0., 5.])
    #             TrafficSign(traffic_sign_elements=[
    #                     TrafficSignElement(value, ['foo'])], **kwargs).draw(
    #                 self.rnd)

    #         for value in TrafficSignIDZamunda:
    #             kwargs['position'] = kwargs['position'] + np.array([0., 5.])
    #             TrafficSign(traffic_sign_elements=[
    #                     TrafficSignElement(value, ['foo'])], **kwargs).draw(
    #                 self.rnd)

    #     # uncomment to check plots
    #     self.rnd.render(show=True)

    #     # check for warnings
    #     for r in record:
    #         print('Found warning:', r.message)
    #     assert len(record) == 0, record

    def test_signal_states(self):
        """Uses all options for plotting objects related to intersections or
        traffic sign/lights."""
        scenario, pp = CommonOceanFileReader(self.filename_test_all).open()
        plt.close('all')
        draw_params = ParamServer()
        for t in range(2):
            self.rnd.clear()
            draw_params['time_begin'] = t
            self.rnd.draw_list(scenario.obstacles, draw_params=draw_params)
            self.rnd.render(show=True)

    def test_complex_intersection_tl(self):
        scenario, pp = CommonOceanFileReader(self.filename_complex_tl).open()
        plt.close('all')
        mpl.rcParams['lines.scale_dashes'] = False
        scenario.lanelet_network.draw(self.rnd, draw_params={
                'time_begin': 30, 'lanelet_network': {
                        'draw_intersections': True,
                        'draw_traffic_signs': True,
                        'intersection':       {'show_label': True}
                }, 'lanelet': {
                        'draw_lane_marking': True, 'show_label': True
                }
        })
        self.rnd.add_legend(self.legend)
        self.rnd.render(show=True)


if __name__ == '__main__':
    unittest.main()
