classdef OctreeSolver < BroadPhaseSolver
    % An Octree algorithm instance, used to resolve the set of collision
    % 
    
    properties (Access = private)
        Tree = Octree.empty;
        MaxSize = 100;
    end
    methods 
        function [this] = OctreeSolver(maxSize)
            % CONSTRUCTOR - 

            % Call the parent
            [this] = this@BroadPhaseSolver();

            if nargin > 0
                this.MaxSize = maxSize/2;
            end
        end
        function [manifolds] = ResolveManifolds(this,colliders)
            % Compute all the possible collision-pairs.

            % Sanity check
            assert(isa(colliders,"Collider"),"Expecting a list of 'collider' objects.");
        
            % 1. Build the Octree (by inserting all points)
            worldExtents = this.MaxSize*[-1;1];
            worldBoundary = AABB([0;0;0.5]*this.MaxSize,worldExtents,worldExtents,worldExtents/2);
            % Create the Octree
            this.Tree = Octree(worldBoundary,1);

            for i = 1:numel(colliders)
                % Insert all colliders
                %this.Root.InsertCollider(colliders(i));

                % Simple case (use their centers)
                if ~this.Tree.InsertPoint(colliders(i).Transform.position)
                    warning("\n Something went wrong, didn't insert Collider %d",colliders(i).Cid);
                end
            end

            fprintf("\n Total nodes '%d'.",this.Tree.GetNumberOfNodes());
            this.Tree.Draw();

            % 2. Query the Octree against each collider
            manifolds = Manifold.empty;
            for i = 1:numel(colliders)
                % We need to decide (based on the objects geometries, what
                % the minimal viable query range can be). Ideally, this
                % would only seach where d = 2*r; 

                % Build the query box/bounds
                queryPosition = colliders(i).Transform.position;
                extents = 2*[-1;1];
                queryBounds = AABB(queryPosition,extents,extents,extents);

                mesh = AABB.CreateMesh(queryBounds);
                
                h = mesh.Draw(gca,"r");
                set(h,"FaceAlpha",0.2);
                
                % Query the octree with a range
                results = this.Tree.Query(queryBounds);

                % Append
                manifolds = vertcat(manifolds,results);
            end

            fprintf("\nTotal Manifolds '%d'.",length(manifolds));

            % Reset the tree
            this.Tree = [];
        end
    end

    methods (Static) %, Access = private)
        
    end
end