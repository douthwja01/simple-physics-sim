classdef (Abstract) CollisionResolver < matlab.mixin.Heterogeneous & handle
    % NARROWPHASESOLVER is a class to resolve collision occurance with
    % higher accuracy that the broad-phase solvers. The usage of
    % narrow-phase collision solves typically follows a broad-phase solver.

    methods
        function [this] = CollisionResolver()
            % CONSTRUCTOR - Create an instance of a narrow-phase solver.
        end
    end
    methods (Abstract)
        % Resolve the collision manifolds
        [this] = Resolve(collisions,dt);
    end
end