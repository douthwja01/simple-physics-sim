classdef PositionSolver < NumericConstraintSolver
    % This basic collision solver resolves collisions simply by resolving
    % the minimum seperation between the two colliders by directly setting
    % the position of the two objects via their transforms.
    
    properties (Constant)
        Name = "A simple position-based collision resolution implementation.";
    end

    methods
        function [this] = Solve(this,dt,manifolds)
            % Sanity check
            assert(isa(manifolds,"Manifold"),"Expecting an array of collisions objects.");
            assert(isnumeric(dt),"Expecting an array of collisions objects.");

            if isempty(manifolds)
                return
            end

            for i = 1:numel(manifolds)
                % For each collision
                manifold = manifolds(i);
                points = manifold.Points;       

                % Party one
                bodyA = manifold.BodyA;
                transformA  = bodyA.Transform;
                isStaticA   = bodyA.IsStatic;                
                
                % Party two
                bodyB = manifold.BodyB;
                transformB  = bodyB.Transform;
                isStaticB   = bodyB.IsStatic;

%                 fprintf("Resolving collision between %s and %s.\n",entityA.Name,entityB.Name);

                assert(points.Depth > 0,"Depth value not defined correctly (it is negative).");

                %----- [TO FIX] Depth value is incorrect

                % Calculate the resolution
                resolution = points.Normal * points.Depth / max(1, isStaticA + isStaticB);
                % Modify the positions
                delta_a = resolution * (1 - isStaticA);
                delta_b = resolution * (1 - isStaticB);

                % Assign the positions
                transformA.SetWorldPosition(transformA.GetWorldPosition() + delta_a);
                transformB.SetWorldPosition(transformB.GetWorldPosition() - delta_b);
            end
        end
    end
end