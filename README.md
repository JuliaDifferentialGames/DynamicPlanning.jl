# DynamicPlanning.jl


This is a work in progress library for warm-starting differential games via kinodynamic motion planning.


## Immediate Development Plan

1. Develop base types [Done]
    - Obstacle [Done]
    - Robot [Done]
    - Workspace [Done]
    - TPBVP [Done]
    - Navigation Task[Done]
    - Planner [Done]
2. Implement Kinodynamic FMT*
3. Define the sample game/games
4. Implement iLQGames
5. Create the potential function and type
6. Determine the planning/games loop


## Testing Plan

1. Solve a TPBVP [Done]
2. Solve for two robots [Done]
3. Update the task and solve two TPBVPs [Done]
4. Solve a TPBVP via KinoFMTStar
5. Determine the reachable sets of the players
6. Verify the sampling strategy
7. Run the potential game in an open map
8. Create a custom task
9. Test the loop with just navigation
10. Add in potential game


## Example

This example will be used for a final project in ASEN 5254: Algorithmic Motion Planning at CU Boulder. 


I propose a hybrid motion planning framework for solving non-convex pursuit-evasion games, inspired by Isaacs' homicidal chauffeur problem, in which the players operate in a workspace containing obstacles. This method alternates between global motion planning using kinodynamic FMT* and local game-theoretic trajectory refinement via iterative Linear-Quadratic games which cycles after a given time horizon. To determine a goal state for motion planning, the evader will sample a free location from its reachable set, over a time horizon, that is furthest from the pursuer. The pursuer will also follow the same sampling strategy from the reachable set of the evader. Through this formulation, kinodynamic FMT* with reachable set sampling is utilized to determine a feasible trajectory to a new desired state. Iterative Linear-Quadratic games applied to a navigation function is then utilized to explore localized coupling in the strategies between the agents and deviate from the global strategy while still maintaining obstacle constraints \cite{rimon1990exact}. This framework grants the ability to determine game theoretic solutions in real-time for non-convex pursuit-evasion games by leveraging the strengths of sampling-based planning and differential game theory.