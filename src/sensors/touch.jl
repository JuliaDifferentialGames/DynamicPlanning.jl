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
Touch sensor type
"""
mutable struct TouchSensor <: AbstractSensor
    base::BaseSensor          # Base sensor information
    data::Bool                # Whether or not there has been contact
end


"""
Touch sensor constructor
"""
function create_touch_sensor(; frequency=1, delay=0, enabled=true)
    # Define info for the base type 
    base = BaseSensor(:TOUCH, :local, frequency, delay, enabled)

    # Create the gps sensor 
    return TouchSensor(base, false)
end
