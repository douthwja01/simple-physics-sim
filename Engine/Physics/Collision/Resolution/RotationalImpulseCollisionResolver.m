classdef RotationalImpulseCollisionResolver < CollisionResolver
    %ROTATIONALIMPULSESOLVER A collision solver that uses the rotational 
    % impulse solution method.

    methods        
        function [this] = Resolve(this,manifolds,dt)
            % SOLVE - Solve the set of the collision manifolds using the
            % rotational impulse solver.

            % Move through the collsion instances
            for i = 1:numel(manifolds)
                % The current manifold
                manifold = manifolds(i);

            end
        end
    end
end

