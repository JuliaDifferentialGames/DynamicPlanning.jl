"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("../tasks/navigation.jl")
include("planning.jl")
include("../problems/planning_problem.jl")

# Usings 

"""
A very dumb algorithm that goes to the goal and ignores obstacles
"""
mutable struct PassThrough <: PlanningAlgorithm
    base::BasePlanner
    dx::Float64
    type::String
end

"""
PassThrough Constructor
"""
function PassThrough(; dx=0.05)
    # Make the base 
    base = base_planner(:PassThrough, pass_through_planner, [])

    return PassThrough(base, dx, "TPBVP")
end


function pass_through_planner(problem::TPBVP)
    sol = Solution([true], [problem.x0, problem.xT], [zeros(2), zeros(2)], [[]])

    return sol
end