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

# Usings 

# Usings 
using DifferentialEquations
using Interpolations 
using LinearAlgebra 
using Optimization
using NearestNeighbors