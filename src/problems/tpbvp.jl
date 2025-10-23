"""
Author: Bennet Outland
Affiliation: CU Boulder
License: MIT

Resources:
- https://docs.sciml.ai/SciMLExpectations/stable/tutorials/optimization_under_uncertainty/
- https://docs.sciml.ai/Optimization/stable/
"""

# Includes 


# Usings 
using DifferentialEquations
using Interpolations 
using LinearAlgebra 
using Optimization
#using OptimizationIpopt


"""
Structure for defining the parameters of a TPBVP
"""
mutable struct TPBVP
    n::Int64               # State dimension
    m::Int64               # Control dimension
    f::Function          # Dynamics
    x0::Vector{Float64}  # Initial state
    xT::Vector{Float64}  # Final state
    tspan::Tuple         # Time span
    N::Int64             # Number of discretization points
    X::Matrix            # State trajectory matrix
    Y::Matrix            # Control trajectory matrix
    J::Float64           # Total cost 
end

"""
Constructor for the TPBVP
"""
function TPBVP(n::Int64, m::Int64, f::Function, x0::Vector{Float64}, xT::Vector{Float64}, tspan::Tuple, N::Int64)
    return TPBVP(n, m, f, x0, xT, tspan, N, zeros(n, N), zeros(m, N), 0)
end

