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


"""
Base planning algorithm type
"""
abstract type PlanningAlgorithm end


"""
Base planning algorithm type
"""
mutable struct BasePlanner
    name::Symbol
    planner::Function
    sensors::Vector
    frequency::Int
    verbose::Bool 
end

"""
Base planner constructor
"""
function base_planner(name, planner, sensors; frequency=1, verbose=false)
    return BasePlanner(name, planner, sensors, frequency, verbose)
end


