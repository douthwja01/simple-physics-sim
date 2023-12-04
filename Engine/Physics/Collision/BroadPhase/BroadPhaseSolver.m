classdef (Abstract) BroadPhaseSolver < CollisionSolver
    %BROADPHASESOLVER is class of solver designed to resolve (cheaply) a
    %complete set of collider pairs where collision is possible. These
    %collision pairs are then passed on to a narrow-phase solver to resolve
    %the actual collision points.
    
    methods
        function [this] = BroadPhaseSolver()
            % CONSTRUCTOR - Create an instance of a broad-phase solver.
        end
    end

    methods (Abstract)
        % Evaluate the complete set of collision pairs.
        [manifolds] = ResolveManifolds(this,colliders);
    end
end

