"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 


# Usings 


"""
Abstract sensor type
"""
abstract type AbstractSensor end

"""
Base sensor type
"""
mutable struct BaseSensor 
    name::Symbol          # Sensor name
    frame::Symbol         # Either :body or :world
    frequency::Int        # Frequency per timestep
    delay::Int            # Delay in timesteps 
    enabled::Bool         # Is the sensor active
end

"""
Base sensor constructor
"""
function base_sensor(name, frame; frequency=1, delay=0, enabled=true)
    return BaseSensor(name, frame, frequency, delay, enabled)
end

