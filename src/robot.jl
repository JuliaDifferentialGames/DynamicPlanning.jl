"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes
include("planning_algorithms/planning.jl")
include("tasks/task.jl")
include("constraints.jl")

# Usings 
using LazySets
using Polyhedra

"""
Base robot type 
"""
mutable struct Robot
    x0::AbstractVector              # Initial state
    n::Int                          # State dimension
    m::Int                          # Control dimension
    shape::Any                      # Robot set shape
    observation::Function           # Observation function
    dynamics::Function              # Dynamics function
    p::Any                          # Parameters
    constraints::Vector             # Constraints   
    X::Vector                       # Trajectory
    U::Vector                       # Control along trajectory
    tasks::Vector{AbstractTask}     # Task set
    sensors::Vector                 # Sensors
    planners::Dict                  # Planers
    Id::Int8                        # Robot ID Number
end


# mutable struct Robot{
#     T<:AbstractVector,   # state vector type (e.g. SVector)
#     D<:Function,         # dynamics function type
#     G<:Function,         # observation function type
#     S<:LazySet,          # shape type
#     P,                   # parameter type
#     C<:AbstractVector,   # constraint container type
#     TaskT<:AbstractTask, # task type (or union)
#     PlanT<:AbstractPlanner
# }
#     x0::T
#     n::Int
#     m::Int
#     shape::S
#     observation::G
#     dynamics::D
#     p::P
#     constraints::C
#     X::Vector{T}
#     U::Vector
#     tasks::Vector{TaskT}
#     sensors::Vector  # You may also parametrize this
#     planners::Dict{DataType, PlanT}
#     Id::Int8
# end


"""
Robot constructor
"""
function robot(x0::Vector, n::Int, m::Int, shape::Any; observation=(x, u, p, t)->x, p = nothing) 
    # Checking 
    @assert isa(observation, Function) "observation must be a function of form: g(x, u, p, t)"
    @assert n == length(x0) "initial condition is not the correct size"
    @assert isa(shape, LazySet) "robot shape must be a LazySet"

    # Create the robot
    Robot(x0, n, m, shape, observation, (x, u, p, t)->u,  p, [], [x0], [], [], [], Dict(), 0)
end 

# ======================================================================================= #
#                                    Task Addition
# ======================================================================================= #

"""
Method for adding tasks to the robot
"""
function add_tasks!(robot::Robot, tasks::AbstractVector{T}) where {T <: AbstractTask}
    append!(robot.tasks, tasks)
    return robot
end


"""
Method to add a planner for a specific task
"""
function add_planner!(robot::Robot, planner)
    robot.planners[planner.type] = planner #planner.base.planner
    return robot
end


"""
Adds new tasks to an existing problem
"""
function update_tasks!(problem::PlanningProblem, id::Int64, tasks::AbstractVector{T}) where {T <: AbstractTask}
    # Access the robot and update the tasks
    return add_tasks!(problem.workspace.robots[id], tasks)
end


# ======================================================================================= #
#                                    Sensor Addition
# ======================================================================================= #

"""
Method for adding sensors to the robot
"""
function add_sensors!(robot::Robot, sensor_vec)
    push!(robot.sensors, sensor_vec...)
    return robot
end


# ======================================================================================= #
#                                    Robot Constraints
# ======================================================================================= #

"""
Adds a general constraint to a robot
"""
function add_constraint!(robot::Robot, C::Function, type::Symbol)
    if type == :equality
        robot = equality_constraint(robot, C, type)
    elseif type ==:inequality
        println(">=")
    elseif type == :soc
        println("soc")
    else
        ArgumentError("constraint type not defined")
    end

    return robot
end


"""
Adds dynamics constraints to a robot
"""
function add_dynamics!(robot::Robot, f::Function)
    robot.dynamics = f;
    return robot
end


# ======================================================================================= #
#                          Constraint Assignment Functions
# ======================================================================================= #

function equality_constraint(robot::Robot, C::Function, type::Symbol)
    # Define the Euclidean projection TODO?

    # Create the constraint
    c = Constraint(C, type)

    # Mutate and return the player
    push!(robot.constraints, c)
    return robot
end

# """
# Adds box constraints 
# """
# function add_box_constraint!(robot::Robot, lower::Vector, upper::Vector)
#     # Form each constraint
#     constraints = []
#     for i ∈ eachindex(lower) 
#         push!(constraints, Constraint((x, u) -> lower[i] - u[i], :ineq))
#         push!(constraints, Constraint((x, u) -> u[i] - upper[i], :ineq))
#     end

#     # Mutate and return the player
#     push!(robot.constraints, constraints...)
#     return robot
# end

"""
Adds box constraints 
"""
function add_box_constraint!(robot::Robot, lower::Vector, upper::Vector, type::Symbol)
    # Mutate and return the player
    push!(robot.constraints, BoxConstraint(upper, lower, type))
    return robot
end