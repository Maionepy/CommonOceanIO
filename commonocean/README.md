# CommonOcean-io

## Introduction

This folder contains a subpackage of commonroad-io: commonocean-io

Commonocean-io defines the differing classes necessary to represent benchmark scenarios for vessels. 
Thus, in the following we shortly describe the re-definitions in order to make the usage of this subpackage more clear.

## Brief documentation of changes compared to CommonRoad

We now explain the new definitions according to the structure of this repository.

1. common
    - file_reader: new commonocean classes integrated to be read
    - file_writer: new commonocean classes integrated to be written
    - solution_writer: adapted imports
    
2. planning
    - goal: Changed possible goal areas from lanelets to waters 
    - planning_problem: adapted imports for new GoalRegions

3. scenario
    - draft: defines new class draft which models shallow areas as static obstacle
    - obstacle: refactored Enum for obstacle types
    - scenario: underground now sea state, scenario tags adapted
    - traffic_sign: country distinction removes, new traffic signs and ids defined, currently traffic sign types are used to determine the diameter
    - waters: similar to lanelet class, different water types and users are defined, water and water network adapted
    
4. visualization
    - draw_dispatch: refactored imports
    - planning: refactored imports, adapted plotting parameters
    - scenario: adapted imports, adapted default parameters and plotting of waters
    - traffic_sign: adapted imports, added new images for signs
    - util: adapted imports
    - draw_ego_vessel: Drawing function especially for ego vessel s.t. it can be drawn directly.
    - velocity/orientation arrows: Dynamic obstacles and ego vessel can now have an arrow in the render showing their velocity and orientation.
    - draw_params: Enable/disable arrows and change their size, change properties of plotted ego vessel.
    - sector based surrounding observation: (located in surrounding_observation.py) Render the sector based observation using create_wedge_for_sector. Occupied sectors will turn red. This can be turned on and off using render_sector_based_surrounding in configs_ships.yaml. 

