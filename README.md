# DynamicPlanning.jl


This is a work in progress library for warm-starting differential games via kinodynamic motion planning.


## Immediate Development Plan

1. Develop base types
    - Obstacle
    - Robot
    - Workspace
    - TPBVP
    - Navigation Task
    - Planner
2. Implement Kinodynamic RRT*
3. Define the sample game/games
4. Implement IBR
5. Create the potential function and type
6. Determine a the planning/games loop


## Example

This example will be used for a final project in ASEN 5254: Algorithmic Motion Planning at CU Boulder. 


I propose a hybrid motion planning framework for solving non-convex pursuit-evasion games, inspired by Isaacs' homicidal chauffeur problem, in which the players operate in a workspace containing obstacles. This method alternates between global motion planning using Kinodynamic RRT* and local game-theoretic trajectory refinement via state-based potential games which cycles after a given time horizon. To determine a goal state for motion planning, the evader will sample a free location from its reachable set, over a time horizon, that is furthest from the pursuer. The pursuer will also follow the same sampling strategy from the reachable set of the evader. Through this formulation, Kinodynamic RRT* with reachable set sampling is utilized to determine a feasible trajectory to a new desired state. Iterative best response applied to a state-based potential game is then utilized to explore localized coupling in the strategies between the agents and deviate from the global strategy while still maintaining obstacle constraints. This framework grants the ability to determine game theoretic solutions in real-time for non-convex pursuit-evasion games by leveraging the strengths of sampling-based planning and differential game theory. 
