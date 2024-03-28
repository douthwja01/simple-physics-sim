classdef OctreeSolver < BroadPhaseSolver
    % An Octree algorithm instance, used to resolve the set of collision
    % 
    
    properties (Access = private)
        Tree = Octree.empty;
        MaxSize = 10;
        NodeCapacity = 1;
        PointsInserted = 0;
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
            this.Tree = Octree(worldBoundary,this.NodeCapacity);

            % Go through all the colliders and insert them into the octree
            for i = 1:numel(colliders)
                % Insert the collider into the tree
                if ~this.InsertCollider(colliders(i))
                    warning("\n Something went wrong, didn't insert Collider %d",colliders(i).Cid);
                end
            end

            fprintf("\n Total nodes '%d'.",this.Tree.GetNumberOfNodes());
            this.Tree.DrawNodes(ah,"c",true);
            %this.Tree.DrawPoints(ah,"g");

            % 2. Query the Octree against each collider
            octreePoints = OctreePoint.empty;
            for i = 1:numel(colliders)
                % We need to decide (based on the objects geometries, what
                % the minimal viable query range can be). Ideally, this
                % would only seach where d = 2*r; 

                % Build the query box/bounds
                p = colliders(i).Transform.Position;
                extents = 10*[-1;1];
                
%                 plot3(ah,p(1),p(2),p(3),"ro","MarkerFaceColor","r");

                % Construct the boundary to query
                queryBounds = AABB(extents,extents,extents);
                queryBounds = queryBounds + p;

                % Create a representative query mesh
                mesh = AABB.CreateMesh(queryBounds);
                h = mesh.Draw(ah,"r");
                set(h,"FaceAlpha",0.1);
                
                % Get all OctreePoints within the queried boundary
                results = this.Tree.Query(queryBounds);

                fprintf("\nPoints inserted '%d'",this.PointsInserted);

                % Draw all the query response points.
                for j = 1:numel(results)
                    p = results(j).Position;
                    plot3(ah,p(1),p(2),p(3),"ro","MarkerFaceColor","k");
                end

                % Append
%                 octreePoints = vertcat(octreePoints,results);
            end
            
%             fprintf("\nTotal Octree-points '%d'.",length(octreePoints));
% 
%             manifolds = Manifold.empty;
%             for i = 1:length(octreePoints)
% 
% 
%                 % Append
% %                 manifolds = vertcat(manifolds,results);
%             end

            fprintf("\nTotal Manifolds '%d'.",length(manifolds));


            % Reset the tree
            this.Tree = [];
        end
    
        function [flag] = InsertCollider(this,collider)
            % Insert a complete collider into the root node.
            
            % We need to insert a mesh
            if isa(collider,"MeshCollider")

                worldMesh = collider.GetWorldMesh();
                v = worldMesh.Vertices;
                %v = collider.Mesh.Vertices;
                for i = 1:worldMesh.NumberOfVertices
                    % Insert a point with reference to the collider
                    octPoint = OctreePoint(v(i,:)',collider);
                    % Insert the vertex
                    if ~this.Tree.InsertPoint(octPoint)
                        flag = false;
                        return;
                    end
                    this.PointsInserted = this.PointsInserted + 1;
                end
                flag = true;
                return
            end

            if isa(collider,"SphereCollider")
                position = collider.Center;
                octPoint = OctreePoint(position,collider);
                % [Test] Insert the center as a vertex
                if ~this.Tree.InsertPoint(octPoint)
                    flag = false;
                    return;
                end
                flag = true;
                this.PointsInserted = this.PointsInserted + 1;
                return 
            end

            flag = true;
        end
    end
end