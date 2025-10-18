"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("../src/workspace.jl")
include("../src/plotting.jl")
include("../src/robot.jl")
include("../src/utils.jl")
include("../src/tasks/navigation.jl")
include("../src/sensors/gps.jl")
include("../src/sensors/touch.jl")

# Usings 


# Define the workspace
𝒲 = workspace([2, 2], 3.0)

# Define and add obstacles 
𝒪 = [
    Ellipsoid([1.0, 1.0], [0.25 0; 0 0.5]),
    Ellipsoid([3.0, 2.0], [0.25 0; 0 0.25])
] 
add_obstacles!(𝒲, 𝒪)

# Define the robot with a navigation task
x0 = [0.0, 0.0]
xg1 = [4.0, 3.0]
xg2 = [2.0, 4.0]

R = robot(x0, 2, 2, circle([0.0, 0.0], 0.1))

nav_task_1 = navigation_task(xg1)
nav_task_2 = navigation_task(xg2)
add_tasks!(R, [nav_task_1, nav_task_2])

gps = create_gps(1:2)
touch_sensor = create_touch_sensor()
add_sensors!(R, [gps, touch_sensor])

add_robots!(𝒲, [R])


# Configure the planning algorithm



# Collect information for the simulation






# Plot 
plot(𝒲)