from commonroad.common.file_reader import CommonRoadFileReader

from commonocean.common.file_writer import CommonOceanFileWriter, OverwriteExistingFile
from commonocean.scenario.scenario import Scenario, Tag
from commonocean.planning.planning_problem import PlanningProblemSet, PlanningProblem, GoalRegion
from commonocean.scenario.obstacle import DynamicObstacle, ObstacleType

name = "OCEAN_test_right_before_left_3.xml"
TYPE = ObstacleType.MOTORVESSEL
planning_problem_id = 1006

scenario_car, pp_cr = CommonRoadFileReader(name).open()

scenario = Scenario(scenario_car.dt, scenario_car.scenario_id)
for obstacle in scenario_car.dynamic_obstacles:
    new_obstacle = DynamicObstacle(obstacle.obstacle_id, TYPE, obstacle.obstacle_shape, obstacle.initial_state, obstacle.prediction)
    scenario.add_objects(new_obstacle)

planning_problem_car = pp_cr.find_planning_problem_by_id(planning_problem_id)
goal = GoalRegion(planning_problem_car.goal.state_list)
planning_problem = PlanningProblem(planning_problem_id,planning_problem_car.initial_state,goal)
planning_problem_set = PlanningProblemSet([planning_problem])


cof = CommonOceanFileWriter(scenario, planning_problem_set, author="Hanna Krasowski", affiliation="Technical University of Munich, Germany", source="handcrafted", tags=[Tag.OPENSEA])
cof.write_to_file(name, overwrite_existing_file=OverwriteExistingFile.ALWAYS)
