classdef OctreeSolver < BroadPhaseSolver
    % An Octree algorithm instance, used to resolve the set of collision
    % 
    
    properties (Access = private)
        Tree = Octree.empty;
        MaxSize = 10;
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
        
            [~,ah] = Graphics.FigureTemplate("Octree View");
            view(ah,[45,45]);
            axis vis3d;

            % 1. Build the Octree (by inserting all points)
            worldExtents = this.MaxSize*[-1;1] + [0.5;0.5];
            worldBoundary = AABB(worldExtents,worldExtents,worldExtents);
            % Create the Octree
            this.Tree = Octree(worldBoundary,1);

            for i = 1:numel(colliders)
                % Insert the collider into the tree
                if ~this.Tree.InsertCollider(colliders(i))
                    warning("\n Something went wrong, didn't insert Collider %d",colliders(i).Cid);
                end
            end

            fprintf("\n Total nodes '%d'.",this.Tree.GetNumberOfNodes());
            this.Tree.DrawNodes(ah,"c",true);
            this.Tree.DrawPoints(ah);

            % 2. Query the Octree against each collider
            octreePoints = OctreePoint.empty;
            for i = 1:numel(colliders)
                % We need to decide (based on the objects geometries, what
                % the minimal viable query range can be). Ideally, this
                % would only seach where d = 2*r; 

                % Build the query box/bounds
                p = colliders(i).Transform.position;
                extents = 2*[-1;1];
                
                % Construct the boundary to query
                queryBounds = AABB(...
                    extents + p(1), ...
                    extents + p(2), ...
                    extents + p(3));

                % Create a representative query mesh
                mesh = AABB.CreateMesh(queryBounds);
                h = mesh.Draw(ah,"r");
                set(h,"FaceAlpha",0.2);
                
                % Query the octree with a range
                results = this.Tree.Query(queryBounds);

                % Append
                octreePoints = vertcat(octreePoints,results);
            end

            fprintf("\nTotal Octree-points '%d'.",length(octreePoints));

            manifolds = Manifold.empty;
            for i = 1:length(octreePoints)


                % Append
%                 manifolds = vertcat(manifolds,results);
            end

            fprintf("\nTotal Manifolds '%d'.",length(manifolds));


            % Reset the tree
            this.Tree = [];
        end
    end

    methods (Static) %, Access = private)
        
    end
end