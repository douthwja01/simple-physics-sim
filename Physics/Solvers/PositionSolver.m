classdef PositionSolver < NarrowPhaseSolver
    % This basic collision solver resolves collisions simply by resolving
    % the minimum seperation between the two colliders by directly setting
    % the position of the two objects via their transforms.
    
    methods
        function [this] = Solve(this,collisions,dt)
            % Sanity check
            assert(isa(collisions,"Manifold"),"Expecting an array of collisions objects.");
            assert(isnumeric(dt),"Expecting an array of collisions objects.");

            for i = 1:numel(collisions) %Manifold& manifold : manifolds)
                % For each collision
                collision = collisions(i);
                manifold = collision.Points;       

                % Party one
                transformA = collision.ColliderA.Transform;
                isStaticA = transformA.IsStatic;

                % Party two
                transformB = collision.ColliderB.Transform;
                isStaticB = transformB.IsStatic;

                % Calculate the resolution
                resolution = manifold.Normal * manifold.Depth / max(1, isStaticA + isStaticB);
                % Ensure 
                resolution = resolution/2;
                % Modify the positions
                delta_a = resolution * (1 - isStaticA);
                delta_b = resolution * (1 - isStaticB);
                % Modify the positions
                transformA.position =  transformA.position + delta_a;
                transformB.position =  transformB.position - delta_b;
            end
        end
    end
end