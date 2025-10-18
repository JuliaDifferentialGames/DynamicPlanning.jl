"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("sensor.jl")

# Usings 


"""
GPS type
"""
mutable struct GPS <: AbstractSensor
    base::BaseSensor          # Base sensor information
    n_measure::Vector{Int64}  # The measureable states 
    data::Vector              # Collection of measurements
end


"""
GPS constructor
"""
function create_gps(n_measure; frequency=1, delay=0, enabled=true)
    # Define info for the base type 
    base = BaseSensor(:GPS, :global, frequency, delay, enabled)

    # Some handling for UnitRanges
    n = n_measure
    if typeof(n_measure) == UnitRange
        n = Vector(n)
    end

    # Create the gps sensor 
    return GPS(base, n, [])
end
