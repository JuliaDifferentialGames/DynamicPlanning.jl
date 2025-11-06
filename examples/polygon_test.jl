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
using LaTeXStrings


# Define the workspace
# 𝒲 = workspace(-2, 7, -2, 7)
𝒲 = workspace([0.0, 0.0], 10.0)


# Define and add obstacles 
𝒪 = create_pursuit_evasion_obstacles(𝒲)

add_obstacles!(𝒲, 𝒪)

plot(𝒲)
xlabel!(L"x")
ylabel!(L"y")