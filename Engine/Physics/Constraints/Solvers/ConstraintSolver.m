classdef (Abstract) ConstraintSolver < Module
% The root class for all constraint solvers.

    methods (Abstract)
        % Resolve the collision manifolds
        [this] = Solve(this,dt,constraints);
    end
end