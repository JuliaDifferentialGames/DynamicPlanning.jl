"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
- https://github.com/schmrlng/MotionPlanning.jl
- https://arxiv.org/pdf/1403.2483
"""

# Includes 
include("../tasks/navigation.jl")
include("planning.jl")
include("../problems/planning_problem.jl")

# Usings 

# Usings 
using DifferentialEquations
using Interpolations 
using LinearAlgebra 
using Optimization
using NearestNeighbors


"""
A very dumb algorithm that goes to the goal and ignores obstacles
"""
mutable struct KinoFMTStar <: PlanningAlgorithm 
    # Base Planner
    base::BasePlanner
    type::String

    # Sampling Params 
    n_samples::Int64 
    goal_bias::Float64

    # Connection 
    r::Float64     
    γ::Float64 
    distance::Function
    goal_tolerance::Float64 

    # Cost and Heuristic 
    cost_function::Function 
    heuristic::Function
end

"""
PassThrough Constructor
"""
function KinoFMTStar(; n_samples=5000, goal_bias=0.05, r=1.0, γ=1.0, distance=x->norm(x), goal_tolerance=0.1, cost_function=x->norm(x), heuristic=x->0)
    # Make the base 
    base = base_planner(:KinoFMTStar, KinoFMTStarPlanner, [])

    return KinoFMTStar(base, "TPBVP", n_samples, goal_bias, r, γ, distance, goal_tolerance, cost_function, heuristic)
end





function KinoFMTStarPlanner(problem::TPBVP)



    sol = Solution([true], [problem.x0, problem.xT], [zeros(2), zeros(2)], [[]])

    return sol
end