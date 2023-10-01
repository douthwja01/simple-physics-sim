
classdef PositionSolver < Solver

    methods
        function [this] = Solve(this,collisions,dt)
            % Sanity check
            assert(isa(collisions,"Collision"),"Expecting an array of collisions objects.");
            assert(isnumeric(dt),"Expecting an array of collisions objects.");

            for i = 1:numel(collisions) %Manifold& manifold : manifolds) 

                transform_a = collisions(i).A
                transform_b = collisions(i).B

                manifold = collisions(i);

			    aBody = manifold.A;
			    bBody = manifold.B;
    
% 			    aStatic = aBody.IsStatic;
% 			    bStatic = bBody.IsStatic;
%     
% 			    resolution = manifold.Normal * manifold.Depth / iw::max<scalar>(1, aStatic + bStatic);
    
% 			    aBody->Transform.Position -= resolution * scalar(1 - aStatic);
% 			    bBody->Transform.Position += resolution * scalar(1 - bStatic);
    		end
        end
    end    
end