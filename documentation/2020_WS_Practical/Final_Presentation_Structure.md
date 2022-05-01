# Structure

1. Motivation & Problem statement (super short)
    1. short and similar to concept presentation
    2. Scope project as overview
    
3. Implementation (Overview)
    1. Implementing subpackage of commonroad-io
        1. What is different to streets? Rules, dimensions, etc
        2. Abstraction waters
    2. What else had to be adapted (detailes description later in presentation)
        1. drivability checker
        2. commonocean-search
    
4. Implementation
    1. waters and rules
    
5. Implementation
    1. abstraction of waters layers
    
6. Implementation
    1. class / object overview (scenario structure)
    
7. Implementation
    1. changes drivability checker
    
8. Implementation
    1. changes commonocean search
    2. comparison different algorithms (e.g A* much better performance on open sea than Depth search)
    2. mass planner
    
9. Advantages and Disadvantages
    1. (+) Due to the similar structure, we were able to import a lot fromcommonroad.  Updates from commonroad, 
    also updates commonocean.
    2. (-) Further features for commonroad still have to be tested with commonocean and possibly adapted.
       
9. Results
    1. Successful development of subpackage with read, write, plot and search functionality
    2. Obstacles are successfully avoided and accidents are prevented
    
10. Future Work
    1. Reading and plotting of real sea maps
    2. Plotting real ship courses using AIS data
    3. Draft logic (current standard 10 meter)
    4. Performance improvements
        1. e.g. search boundaries to current waters to minimize calculation process

11. References

