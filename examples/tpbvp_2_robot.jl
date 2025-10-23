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
xg2 = [2.0, 3.0, 0.0]

# Define robot and it's constraints
R1 = robot(x0, 3, 2, circle([0.0, 0.0], 0.1))
add_dynamics!(R1, unicycle!)

R2 = robot(x0, 3, 2, circle([0.0, 0.0], 0.1))
add_dynamics!(R2, unicycle!)


# Define robot tasks
nav_task_1 = navigation_task(xg1)
add_tasks!(R1, [nav_task_1])

nav_task_2 = navigation_task(xg2)
add_tasks!(R2, [nav_task_2])

# Configure the planning algorithm
pt = PassThrough()
add_planner!(R1, pt)
add_planner!(R2, pt)

# Add robot(s) to the simulation
add_robots!(𝒲, [R1, R2])

# Define the planning problem 
prob = PlanningProblem(𝒲, (0.0, 1.0), 0.01)

# Solve the problem
sol = solve!(prob)

# Plot 
# plot(𝒲)
plot(prob, sol)