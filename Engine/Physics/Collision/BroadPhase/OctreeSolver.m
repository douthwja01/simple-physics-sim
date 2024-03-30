classdef OctreeSolver < BroadPhaseSolver
    % An Octree algorithm instance, used to resolve the set of collision
    % 
    
    properties (Access = private)
        Tree = Octree.empty;
        MaxSize = 10;
        NodeCapacity = 1;
        PointsInserted = 0;
        DrawComponents = false;
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

            % Default value
            manifolds = Manifold.empty;

            % Sanity check
            assert(isa(colliders,"Collider"),"Expecting a list of 'collider' objects.");
        
            if this.DrawComponents
                [fh,ah] = Graphics.FigureTemplate("Octree View");
                view(ah,[45,45]);
                axis vis3d;
            end

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

            if this.DrawComponents
                fprintf("\n Points inserted '%d', Total nodes '%d'.",this.PointsInserted,this.Tree.GetNumberOfNodes());
                this.Tree.DrawNodes(ah,"c",true);
                %this.Tree.DrawPoints(ah,"g");
            end
            % Containers
            colliderCids = [colliders.Cid]';

            % 2. Query the Octree against each collider
            for i = 1:numel(colliders)
                % We need to decide (based on the objects geometries, what
                % the minimal viable query range can be). Ideally, this
                % would only seach where d = 2*r; 

                % Get the query collider
                collider_i = colliders(i);
                % Construct the boundary to query
                queryBounds = collider_i.GetWorldAABB();
                
                % Draw output
                if this.DrawComponents
                    h = queryBounds.Draw(ah,"r");
                    set(h,"FaceAlpha",0.1);
                end

                % Get all OctreePoints within the queried boundary
                results = this.Tree.Query(queryBounds);

                % No points within the collider (AABB) query
                if isempty(results)
                    continue;
                end

                % Draw all the query response points.
                if this.DrawComponents
                    for j = 1:numel(results)
                        p = results(j).Position;
                        plot3(ah,p(1),p(2),p(3),"ro","MarkerFaceColor","k");
                    end
                end

                % We need to extract a collider reference for each unique
                % Cid point.

                % Get the unique collider identities
                uniqueCids = unique([results.Cid]);
         
                for j = 1:numel(uniqueCids)
                    % Selection logicals
                    logicals = colliderCids == uniqueCids(j);
                    % Isolate unique collider reference
                    collider_j = colliders(logicals);
                    % Add a manifold between this 'query collider and and the other
                    manifolds = vertcat(manifolds,Manifold(collider_i,collider_j));
                end
            end

            fprintf("\nTotal Manifolds '%d'.",length(manifolds));

            if this.DrawComponents
                delete(fh);
            end

            % Reset the tree
            this.Tree = Octree.empty;
        end
    
        function [flag] = InsertCollider(this,collider)
            % Insert a complete collider into the root node.
            
            aabb = collider.GetWorldAABB();
            meshAABB = aabb.ToMesh();
          
            for i = 1:meshAABB.NumberOfVertices
                % Insert a point and collider 'cid'
                octPoint = OctreePoint(meshAABB.Vertices(i,:)',collider.Cid);
                % Insert the vertex
                if ~this.Tree.Insert(octPoint)
                    flag = false;
                    return;
                end
                this.PointsInserted = this.PointsInserted + 1;
            end

%             % We need to insert a mesh
%             if isa(collider,"MeshCollider")
%                 worldMesh = collider.GetWorldMesh();
%                 v = worldMesh.Vertices;
%                 %v = collider.Mesh.Vertices;
%                 for i = 1:worldMesh.NumberOfVertices
%                     % Insert a point and collider 'cid'
%                     octPoint = OctreePoint(v(i,:)',collider.Cid);
%                     % Insert the vertex
%                     if ~this.Tree.Insert(octPoint)
%                         flag = false;
%                         return;
%                     end
%                     this.PointsInserted = this.PointsInserted + 1;
%                 end
%                 flag = true;
%                 return
%             end

%             if isa(collider,"SphereCollider")
%                 position = collider.Center;
%                 octPoint = OctreePoint(position,collider.Cid);
%                 % [Test] Insert the center as a vertex
%                 if ~this.Tree.Insert(octPoint)
%                     flag = false;
%                     return;
%                 end
%                 flag = true;
%                 this.PointsInserted = this.PointsInserted + 1;
%                 return 
%             end

            flag = true;
        end
    end
end