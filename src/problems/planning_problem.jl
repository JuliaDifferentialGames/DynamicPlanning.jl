"""
Author: Bennet Outland
Affiliation: CU Boulder
License: MIT

Resources:
-
"""

# Includes 
include("../workspace.jl")

# Usings


"""
Base Simulation type
"""
mutable struct PlanningProblem
    workspace::Workspace
    tspan::Tuple
    dt::Float64
end

"""
Constructor for a planning problem
"""
function PlanningProblem(workspace::Workspace, tspan::Tuple, dt::Float64)
    return PlanningProblem(workspace, tspan, dt)
end

"""
Solution object for a planning problem. Vectors correspond to the different tasks.
"""
struct Solution
    complete::Vector{Bool}   # Whether all tasks were completed
    X::Vector{Vector}        # Collection of robot states
    Y::Vector{Vector}        # Collection of robot control
    metrics::Vector{Vector}  # Custom metrics
end
