"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Imports 

# Usings 
using LazySets
using Polyhedra
using LinearAlgebra


"""
Defines an n dimensional ball/circle
"""
function circle(center, radius)
    n = length(center)
    Ellipsoid(Vector{Float64}(center), Matrix((radius^2) * I, n, n))
end