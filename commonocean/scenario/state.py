import copy
from typing import List, Union, Tuple, Optional
import numpy as np

import commonroad.geometry.transform
from commonroad.common.util import AngleInterval
from commonroad.common.validity import (
    is_valid_orientation,
    is_real_number_vector,
    is_real_number,
    ValidTypes,
    is_natural_number,
    is_positive,
)
from commonroad.geometry.shape import Shape
from commonroad.common.util import make_valid_orientation
from commonroad.visualization.renderer import IRenderer

__author__ = "Hanna Krasowski, Benedikt Pfleiderer, Fabian Thomas-Barein"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = ["ConVeY"]
__version__ = "2022a"
__maintainer__ = "Hanna Krasowski"
__email__ = "commonocean@lists.lrz.de"
__status__ = "released"


class State:
    """ A state can be either exact or uncertain. Uncertain state elements
    can be either of type
        :class:`commonroad.common.util.Interval` or of type
        :class:`commonroad.geometry.shape.Shape`. A
        state is composed of several elements which are determined during
        runtime. The possible state elements
        are defined as slots, which comprise the necessary state elements to
        describe the states of all CommonRoad
        vehicle models:

        :ivar position: :math:`s_x`- and :math:`s_y`-position in a global
        coordinate system. Exact positions
            are given as numpy array [x, y], uncertain positions are given as :class:`commonroad.geometry.shape.Shape`
        :ivar orientation: yaw angle :math:`\Psi`. Exact values are given as real number, uncertain values are given as
            :class:`commonroad.common.util.AngleInterval`
        :ivar velocity: velocity :math:`v_x` in longitudinal direction in the vehicle-fixed coordinate system. Exact
            values are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar rudder_angle: rudder angle :math:`\beta`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar rudder_angle_speed: rudder angle speed :math:`\dot{\beta}` Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar yaw_rate: yaw rate :math:`\dot{\Psi}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_angle: roll angle :math:`\Phi_S`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_rate: roll rate :math:`\dot{\Phi}_S`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar pitch_angle: pitch angle :math:`\Theta_S`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar pitch_rate: pitch rate :math:`\dot{\Theta}_S`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_y: velocity :math:`v_y` in lateral direction in the vehicle-fixed coordinate system. Exact
            values are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar position_z: position :math:`s_z` (height) from ground. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_z: velocity :math:`v_z` in vertical direction perpendicular to road plane. Exact values are
            given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_angle_front: roll angle front :math:`\Phi_{UF}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_rate_front: roll rate front :math:`\dot{\Phi}_{UF}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_y_front: velocity :math:`v_{y,UF}` in y-direction front. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar position_z_front: position :math:`s_{z,UF}` in z-direction front. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_z_front: velocity :math:`v_{z,UF}` in z-direction front. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_angle_rear: roll angle rear :math:`\Phi_{UR}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_rate_rear: roll rate rear :math:`\dot{\Phi}_{UR}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_y_rear: velocity :math:`v_{y,UR}` in y-direction rear. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar position_z_rear: position :math:`s_{z,UR}` in z-direction rear. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_z_rear: velocity :math:`v_{z,UR}` in z-direction rear. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar acceleration: acceleration :math:`a_x`. We optionally include acceleration as a state variable for
            obstacles to provide additional information, e.g., for motion prediction, even though acceleration is often
            used as an input for vehicle models. Exact values are given as real number, uncertain values are given as
            :class:`commonroad.common.util.Interval`
        :ivar acceleration_y: velocity :math:`a_y`.
            We optionally include acceleration as a state variable for obstacles to provide additional information,
            e.g., for motion prediction, even though acceleration is often used as an input for vehicle models. Exact
            values are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar jerk: acceleration :math:`j`. We optionally include jerk as a state variable for
            obstacles to provide additional information, e.g., for motion prediction, even though jerk is often
            used as an input for vehicle models. Exact values are given as real number, uncertain values are given as
            :class:`commonroad.common.util.Interval`
        :ivar time_step: the discrete time step. Exact values are given as integers, uncertain values are given as
            :class:`commonroad.common.util.Interval`

        :Example:

        >>> import numpy as np
        >>> from commonocean.scenario.state import State
        >>> from commonroad.common.util import Interval
        >>> # a state with position [2.0, 3.0] m and uncertain velocity from 5.4 to 7.0 m/s
        >>> # can be created as follows:
        >>> state = State(position=np.array([2.0, 3.0]), velocity=Interval(5.4, 7.0))
    """

    __slots__ = [
        'position',
        'orientation',
        'velocity',
        'rudder_angle',
        'rudder_angle_speed',
        'yaw_rate',
        'roll_angle',
        'roll_rate',
        'pitch_angle',
        'pitch_rate',
        'velocity_y',
        'position_z',
        'velocity_z',
        'roll_angle_front',
        'roll_rate_front',
        'velocity_y_front',
        'position_z_front',
        'velocity_z_front',
        'roll_angle_rear',
        'roll_rate_rear',
        'velocity_y_rear',
        'position_z_rear',
        'velocity_z_rear',
        'acceleration',
        'acceleration_y',
        'jerk',
        'time_step',
    ]

    def __init__(self, **kwargs):
        """ Elements of state vector are determined during runtime."""
        for (field, value) in kwargs.items():
            setattr(self, field, value)

    def translate_rotate(self, translation: np.ndarray, angle: float) -> 'State':
        """ First translates the state, and then rotates the state around the origin.

            :param translation: translation vector [x_off, y_off] in x- and y-direction
            :param angle: rotation angle in radian (counter-clockwise)
            :return: transformed state
        """
        assert is_real_number_vector(translation, 2), (
            '<State/translate_rotate>: argument translation is not '
            'a vector of real numbers of length 2.'
        )
        assert is_real_number(angle), (
            '<State/translate_rotate>: argument angle must be a scalar. '
            'angle = %s' % angle
        )
        assert is_valid_orientation(angle), (
            '<State/translate_rotate>: argument angle must be within the '
            'interval [-2pi,2pi]. angle = %s.' % angle
        )
        transformed_state = copy.copy(self)
        if hasattr(self, 'position'):
            # exact position:
            if isinstance(self.position, ValidTypes.ARRAY):
                transformed_state.position = commonroad.geometry.transform.translate_rotate(
                    np.array([self.position]), translation, angle
                )[
                    0
                ]
            # uncertain position:
            elif isinstance(self.position, Shape):
                transformed_state.position = self.position.translate_rotate(
                    translation, angle
                )
            else:
                raise TypeError(
                    '<State/translate_rotate> Expected instance of %s or %s. Got %s instead.'
                    % (ValidTypes.ARRAY, Shape, self.position.__class__)
                )
        if hasattr(self, 'orientation'):
            # orientation can be either an interval or an exact value
            if isinstance(self.orientation, ValidTypes.NUMBERS):
                transformed_state.orientation = make_valid_orientation(
                    self.orientation + angle
                )
            elif isinstance(self.orientation, AngleInterval):
                transformed_state.orientation = transformed_state.orientation + angle
            else:
                raise TypeError(
                    '<State/translate_rotate> Expected instance of %s or %s. Got %s instead.'
                    % (ValidTypes.NUMBERS, AngleInterval, self.orientation.__class__)
                )
        return transformed_state

    @property
    def attributes(self) -> List[str]:
        """ Returns all dynamically set attributes of an instance of State.

        :Example:

        >>> import numpy as np
        >>> from commonocean.scenario.state import State
        >>> state = State(position=np.array([0.0, 0.0]), orientation=0.1, velocity=3.4)
        >>> print(state.attributes)
        ['position', 'orientation', 'velocity']

        :return: subset of slots which are dynamically assigned to the object.
        """
        attributes = list()
        for slot in self.__slots__:
            if hasattr(self, slot):
                attributes.append(slot)
        return attributes

    @property
    def is_uncertain_position(self):
        return isinstance(self.position, Shape)

    @property
    def is_uncertain_orientation(self):
        return isinstance(self.orientation, AngleInterval)

    def __str__(self):
        traffic_str = '\n'
        for attr in self.attributes:
            traffic_str += attr
            traffic_str += '= {}\n'.format(self.__getattribute__(attr))
        return traffic_str

    def draw(self, renderer: IRenderer,
             draw_params: Union[ParamServer, dict, None] = None,
             call_stack: Optional[Tuple[str, ...]] = tuple()):
        renderer.draw_state(self, draw_params, call_stack)
