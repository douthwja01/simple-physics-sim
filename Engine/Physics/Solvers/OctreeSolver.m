classdef OctreeSolver < BroadPhaseSolver
    % An Octree algorithm instance, used to resolve the set of collision
    % 
    
    properties
        maxDepth = 8;
%         AABBs = AABB.empty;
    end
    methods 
        function [this] = OctreeSolver()
            % CONSTRUCTOR - 

            % Call the parent
            [this] = this@BroadPhaseSolver();
        end
        function [manifolds] = ResolveManifolds(this,colliders)
            % 

            % Sanity check
            assert(isa(colliders,"Collider"),"Expecting a list of 'collider' objects.");
        
            scale = 100;
            minSize = 1;

            [node] = OctreeSolver.CreateNode(scale,depth);
      
        end
    end

    methods (Static) %, Access = private)
        function [nodes] = RecursiveNodeCreation(center,maxSize,depth,maxDepth)
        
            if depth > maxDepth
                nodes = [];
                return
            end

            voxelScale = maxSize/depth;
            unitBounds = [0,0,1;0,1,1;1,0,1;1,1,1;0,0,0;0,1,0;1,1,0;1,0,0];
            zeroCenteredBounds = unitBounds - 0.5*ones(1,3);

            
            childcentroids = center' + zeroCenteredBounds*voxelScale;
            childDepth = depth + 1;
            nodes = [];
            for i = 1:8

                
                childCenter = childcentroids(i,:);

                [childNodes] = OctreeSolver.RecursiveNodeCreation(childCenter,maxSize,childDepth,maxDepth);

                nodes = vertcat(nodes,childNodes);
            end
        end
    end
end