classdef (Abstract) NarrowPhaseSolver < CollisionSolver
    % NARROWPHASESOLVER is a class to resolve collision occurance with
    % higher accuracy that the broad-phase solvers. The usage of
    % narrow-phase collision solves typically follows a broad-phase solver.

    methods
        function [this] = NarrowPhaseSolver()
            % CONSTRUCTOR - Create an instance of a narrow-phase solver.
        end
    end
    methods (Abstract)
        % Solve the collisions
        [this] = Solve(collisions,dt);
    end
end