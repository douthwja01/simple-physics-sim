classdef OctreeSolver < BroadPhaseSolver
    % An Octree algorithm instance, used to resolve the set of collision
    % 
    
    properties
        maxDepth = 8;
        Root = OctreeNode.empty;
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
        
            this.Root = OctreeNode(zeros(3,1),30);
            
            for i = 1:numel(colliders)
                % Insert all colliders
                this.Root.InsertCollider(colliders(i));
            end
        end
    end

    methods (Static) %, Access = private)
        
    end
end