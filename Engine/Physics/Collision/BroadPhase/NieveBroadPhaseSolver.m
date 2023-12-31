classdef NieveBroadPhaseSolver < BroadPhaseSolver
    %NIEVE BROAD PHASE SOLVER is class of solver designed to resolve the
    % complete set of collider pairs where collision is possible using a
    % nieve approach. These collision pairs are then passed on to a
    % narrow-phase solver to resolve the actual collision points.

    methods
        function [this] = NieveBroadPhaseSolver(varargin)
            % CONSTRUCTOR - Create an instance of a broad-phase solver.
        
            % Call the parent
            [this] = this@BroadPhaseSolver(varargin{:})
        end
    end

    methods
        % Evaluate the complete set of collision pairs.
        function [manifolds] = ResolveManifolds(this,colliders)
            % This approach uses a nieve approach to find all the possible
            % collision manifolds where collision could be occurring.

            manifolds = Manifold.empty;
            % This function solves the inter-particle collisions (brute force)
            for i = 1:numel(colliders)
                collider_i = colliders(i);
                for j = i+1:numel(colliders)
                    collider_j = colliders(j);

                    % Check if assessing self-collision
                    if (collider_i.Cid == collider_j.Cid)
                        continue;
                    end

                    newManifold = Manifold(collider_i,collider_j);

                    manifolds = vertcat(manifolds,newManifold); 
                end
            end
        end
    end
end