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
include("../tasks/task.jl")
include("../problems/tpbvp.jl")
include("planning.jl")

# Usings 
using DifferentialEquations
using Interpolations 
using LinearAlgebra 
using Optimization


"""
Base planning algorithm type
"""
mutable struct InteriorPointPlanner
    name::Symbol
    type::AbstractTask
    problem::TPBVP
    sensors::Vector
    verbose::Bool 
end


"""
Solve the TPBVP using Ipopt
"""
function InteriorPointPlanner(problem::TPBVP)

end