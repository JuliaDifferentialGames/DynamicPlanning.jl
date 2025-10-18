"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("utils.jl")

# Usings 
using LazySets
using Polyhedra


"""
Base workspace type. 
"""
mutable struct Workspace
    bounds::Any
    obstacles::Vector{Any}
    robots::Vector{Any}
end


"""
Workspace constructor
"""
function workspace(bounds)
    # Check if it is a viable set 
    if bounds isa LazySet
        return Workspace(bounds, [], [])
    else 
        throw("bounds must be a concrete LazySet")
    end
end

"""
Overloaded workspace constructor for a circlular workspace
"""
function workspace(center, radius)
    # Check if it is a viable set 
    if radius > 0
        c = circle(center, radius)
        return Workspace(c, [], [])
    else 
        throw("radius must be positive")
    end
end

"""
Add obstacles to the workspace
"""
function add_obstacles!(workspace, obstacle_vector)
    # Check if it is a viable set 
    if all([obstacle isa LazySet for obstacle ∈ obstacle_vector])
        push!(workspace.obstacles, obstacle_vector...) 
        return workspace
    else 
        throw("obstacle must be a concrete LazySet")
    end
end


"""
Add robots to the workspace
"""
function add_robots!(workspace, robot_vector)
    # Check if it is a viable set 
    if true
        push!(workspace.robots, robot_vector...) 
        return workspace
    else 
        throw("TODO check")
    end
end