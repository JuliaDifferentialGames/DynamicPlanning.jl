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
include("../src/models.jl")
include("../src/solve.jl")
#include("../src/problems/planning_problem.jl")
include("../src/planning_algorithms/PassThrough.jl")

# Usings 


# Define the workspace
𝒲 = workspace(-2, 7, -2, 7)

# Define and add obstacles 
𝒪 = [
    Ellipsoid([1.0, 1.0], [0.25 0; 0 0.5]),
    Ellipsoid([3.0, 2.0], [0.25 0; 0 0.25])
] 
add_obstacles!(𝒲, 𝒪)

# Define the robot with a navigation task
x0 = [0.0, 0.0, 0.0]
xg1 = [4.0, 3.0, 0.0]
xg2 = [2.0, 3.0, 0.0]

# Define robot and it's constraints
R = robot(x0, 3, 2, circle([0.0, 0.0], 0.1))
add_dynamics!(R, unicycle!)

# Define robot tasks
nav_task_1 = navigation_task(xg1)
add_tasks!(R, [nav_task_1])

# Configure the planning algorithm
pt = PassThrough()
add_planner!(R, pt)

# Add robot(s) to the simulation
add_robots!(𝒲, [R])

# Define the planning problem 
prob = PlanningProblem(𝒲, (0.0, 1.0), 0.01)

# Solve the problem
sol = solve!(prob)

# Add a new task to the existing problem 
nav_task_2 = navigation_task(xg2)
update_tasks!(prob, 1, [nav_task_2])

new_sol = solve!(prob)

# Plot 
plot(prob, new_sol)