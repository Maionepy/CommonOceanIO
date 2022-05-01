import unittest

from commonocean.planning.goal import GoalRegion
from commonocean.planning.planning_problem import PlanningProblem

import numpy as np
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle, Circle
from commonroad.scenario.trajectory import State, Trajectory

__author__ = "Bruno Maione"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2022.1"
__email__ = "bruno.maione@tum.de"
__status__ = "Development"

class TestTranslateRotate(unittest.TestCase):
    def test_translate(self):
        translation = np.array((10.0, 1.0))
        angle = 0.0
        pos = np.array((1.0, 1.0))
        initial_state = State(position=pos, velocity=10.0, orientation=0.0, yaw_rate=0, slip_angle=0, time_step=1)

        shape1 = Rectangle(2.0, 4.0, np.array((2.0, 2.0)))
        shape2 = Circle(2.5, np.array((-1.0, 1.0)))

        goal_state_1 = State(position=shape1, time_step=Interval(0, 5))
        goal_state_2 = State(position=shape2, time_step=Interval(0, 2))
        goal_region = GoalRegion([goal_state_1, goal_state_2])
        planning_problem = PlanningProblem(1, initial_state, goal_region)

        planning_problem.translate_rotate(translation, angle)

        self.assertAlmostEqual(planning_problem.initial_state.position[0], pos[0] + translation[0])
        self.assertAlmostEqual(planning_problem.initial_state.position[1], pos[1] + translation[1])
        self.assertAlmostEqual(planning_problem.initial_state.orientation, 0.0)

        self.assertAlmostEqual(planning_problem.goal.state_list[0].position.center[0],
                               shape1.center[0] + translation[0])
        self.assertAlmostEqual(planning_problem.goal.state_list[0].position.center[1],
                               shape1.center[1] + translation[1])
        self.assertAlmostEqual(planning_problem.goal.state_list[1].position.center[0],
                               shape2.center[0] + translation[0])
        self.assertAlmostEqual(planning_problem.goal.state_list[1].position.center[1],
                               shape2.center[1] + translation[1])

    def test_rotate(self):
        translation = np.array((0.0, 0.0))
        angle = np.pi/4
        pos = np.array((1.0, 1.0))
        initial_state = State(position=pos, velocity=10.0, orientation=np.pi/2, yaw_rate=0, slip_angle=0, time_step=2)

        shape1 = Rectangle(2.0, 4.0, np.array((2.0, 2.0)))
        shape2 = Circle(2.5, np.array((-1.0, 1.0)))

        goal_state_1 = State(position=shape1, time_step=Interval(0, 5), orientation=AngleInterval(np.pi/8, 3*np.pi/8))
        goal_state_2 = State(position=shape2, time_step=Interval(0, 2), orientation=AngleInterval(3*np.pi/4, np.pi))
        goal_region = GoalRegion([goal_state_1, goal_state_2])
        planning_problem = PlanningProblem(1, initial_state, goal_region)

        planning_problem.translate_rotate(translation, angle)

        self.assertAlmostEqual(planning_problem.initial_state.orientation, np.pi/2 + angle)

        self.assertAlmostEqual(planning_problem.goal.state_list[0].orientation.start, angle + np.pi/8)
        self.assertAlmostEqual(planning_problem.goal.state_list[0].orientation.end, angle + 3 * np.pi/8)
        self.assertAlmostEqual(planning_problem.goal.state_list[1].orientation.start, angle + 3 * np.pi/4)
        self.assertAlmostEqual(planning_problem.goal.state_list[1].orientation.end, angle + np.pi)

    def test_goal_reached(self):
        pos = np.array((0.0, -3.0))
        initial_state = State(position=pos, velocity=10.0, orientation=0.0, yaw_rate=0, slip_angle=0, time_step=1)

        shape1 = Rectangle(2.0, 4.0, np.array((2.0, 2.0)))
        shape2 = Circle(2.5, np.array((-1.0, 1.0)))

        goal_state_1 = State(position=shape1, time_step=Interval(0, 5))
        goal_state_2 = State(position=shape2, time_step=Interval(0, 2))
        goal_region = GoalRegion([goal_state_1, goal_state_2])
        planning_problem = PlanningProblem(1, initial_state, goal_region)

        state_reached = State(position=np.array([2, 2]), velocity=10, orientation=0.0, yaw_rate =0, slip_angle=0,
                              time_step=1)
        state_not_reached = State(position=np.array([0, -6]), velocity=10, orientation=(3/2)*np.pi, yaw_rate=0,
                                  slip_angle=0, time_step=1)
        trajectory_reached = Trajectory(1, [initial_state, state_reached])
        trajectory_not_reached = Trajectory(1, [initial_state, state_not_reached])

        self.assertTrue(planning_problem.goal_reached(trajectory_reached)[0])
        self.assertFalse(planning_problem.goal_reached(trajectory_not_reached)[0])


if __name__ == '__main__':
    unittest.main()