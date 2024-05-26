classdef PositionCR < CollisionResolver
    % This basic collision solver resolves collisions simply by resolving
    % the minimum seperation between the two colliders by directly setting
    % the position of the two objects via their transforms.
    
    properties (Constant)
        Name = "A simple position-based collision resolution implementation.";
    end

    methods
        function [this] = Resolve(this,manifolds,dt)
            % Sanity check
            assert(isa(manifolds,"Manifold"),"Expecting an array of collisions objects.");
            assert(isnumeric(dt),"Expecting an array of collisions objects.");

            for i = 1:numel(manifolds)
                % For each collision
                collision = manifolds(i);
                points = collision.Points;       

                % Party one
                entityA = collision.ColliderA.Entity;
                transformA = entityA.Transform;
                isStaticA = transformA.IsStatic;

                % Party two
                entityB = collision.ColliderB.Entity;
                transformB = entityB.Transform;
                isStaticB = transformB.IsStatic;

%                 fprintf("Resolving collision between %s and %s.\n",entityA.Name,entityB.Name);

                % Calculate the resolution
                resolution = points.Normal * points.Depth / max(1, isStaticA + isStaticB);
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