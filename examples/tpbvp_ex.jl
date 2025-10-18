"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("../src/workspace.jl")
include("../src/plotting.jl")
include("../src/robot.jl")
include("../src/utils.jl")
include("../src/tasks/navigation.jl")
include("../src/sensors/gps.jl")
include("../src/sensors/touch.jl")
include("../models.jl")
include("../src/solve.jl")
#include("../src/problems/planning_problem.jl")

# Usings 


# Define the workspace
𝒲 = workspace([2, 2], 3.0)

# Define and add obstacles 
𝒪 = [
    Ellipsoid([1.0, 1.0], [0.25 0; 0 0.5]),
    Ellipsoid([3.0, 2.0], [0.25 0; 0 0.25])
] 
add_obstacles!(𝒲, 𝒪)

# Define the robot with a navigation task
x0 = [0.0, 0.0, 0.0]
xg1 = [4.0, 3.0, 0.0]

# Define robot and it's constraints
R = robot(x0, 3, 2, circle([0.0, 0.0], 0.1))
add_dynamics!(R, unicycle!)

# Define robot tasks
nav_task_1 = navigation_task(xg1)
add_tasks!(R, [nav_task_1])

# Configure the planning algorithm
# rrt = RRT()
# add_planner!(R, rrt)

# Add robot(s) to the simulation
#add_robots!(𝒲, [R])

# Define the planning problem 
# prob = PlanningProblem(𝒲, (0.0, 1.0), 0.01)

# # Solve the problem
# sol = solve(prob)






# Plot 
# plot(𝒲)