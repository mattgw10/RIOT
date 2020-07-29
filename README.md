# RIOT
### Region Informed Optimal Trees (RIOT) code
RIOT is a sampling based motion planner intended for kinodynamic planning which uses an abtsraction (or simplification) of the full state space in order to guide the search. RIOT retains properties of probabilistic completeness and asymptotic optimality while taking advantage of heuristic graph search methods for performance increases.

This planner does not use tha ANN nearest neighbor library but was orignally programmed to compare against others planners which do use this library, thus it is required for compiling and included in the zip file ann.zip and can also be found at: http://www.cs.umd.edu/~mount/ANN/#:~:text=ANN%20%2D%20Approximate%20Nearest%20Neighbor%20Library&text=What%20is%20ANN%3F,d%2Ddimensional%20space%20is%20given.

The include folder has the required header files to read the map, create an abstraction, and search the abstraction as well as several vehicles, a dynamic car, hovercraft, and generic 3D dynamic point robot to run the code with.

#### To compile the code
        make.sh
        
#### To run the code
        RIOT -m [map] -v [vehicle] -t [time] -s [start grid cell] -g [goal grid cell]
        
You can also use `experiment.sh` which has examples with `run.py` and `moving_ai_bench.py` which run for a set number of times or on a batch file respectively for benchmarking. The moving ai maps and batch files are from: https://movingai.com/benchmarks/grids.html

The map files have the top two lines designating the dimensions of the maps occupancy grid. The next two lines are the dimensions of the abstraction, which can be the same or a smaller multiple of each. For example the map orz100d.map could be:

    395
    412
    395
    412
    ....
 for a full resolution abstraction or:
 
    395
    412
    79
    103
    ......
 for a course abstraction. The rest of the map file contains which cells are blocked.
