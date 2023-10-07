
classdef PositionSolver < Solver

    methods
        function [this] = Solve(this,collisions,dt)
            % Sanity check
            assert(isa(collisions,"Collision"),"Expecting an array of collisions objects.");
            assert(isnumeric(dt),"Expecting an array of collisions objects.");

            for i = 1:numel(collisions) %Manifold& manifold : manifolds) 
                % For each collision
                collision = collisions(i);
                manifold = collision.Points;

                entityA = collision.ColliderA.Entity;
                transformA = entityA.GetElement("Transform");
                isStaticA = transformA.IsStatic;
			    
                entityB = collision.ColliderB.Entity;
                transformB = entityB.GetElement("Transform");
			    isStaticB = transformB.IsStatic;

                % Calculate the resolution
			    resolution = manifold.Normal * manifold.Depth / max(1, isStaticA + isStaticB);
                resolution = resolution/2;
                % Modify the positions
                delta_a = resolution * (1 - isStaticA);
			    delta_b = resolution * (1 - isStaticB);
                transformA.position =  transformA.position - delta_a;
			    transformB.position =  transformB.position + delta_b;
    		end
        end
    end    
end