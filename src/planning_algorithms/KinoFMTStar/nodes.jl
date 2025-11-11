

# ============================================================================ #
# Kinodynamic Node Structure
# ============================================================================ #

"""
    KinoNode{T, U}

Node for kinodynamic planning containing state and control trajectory.

# Fields
- `state::T`: State at this node (position, velocity, etc.)
- `control::Union{U, Nothing}`: Control input that reached this state
- `duration::Float64`: Duration of control application
- `cost::Float64`: Cost-to-come from start
- `parent::Union{KinoNode{T,U}, Nothing}`: Parent node
- `trajectory::Vector{T}`: State trajectory from parent (for collision checking)
- `in_open::Bool`: Node in OPEN set flag
- `in_closed::Bool`: Node in CLOSED set flag
"""
mutable struct KinoNode{T, U}
    state::T
    control::Union{U, Nothing}
    duration::Float64
    cost::Float64
    parent::Union{KinoNode{T, U}, Nothing}
    trajectory::Vector{T}
    in_open::Bool
    in_closed::Bool
end

function KinoNode(state::T, control::Union{U, Nothing}=nothing, 
                  duration::Float64=0.0) where {T, U}
    KinoNode{T, U}(state, control, duration, Inf, nothing, T[], false, false)
end
