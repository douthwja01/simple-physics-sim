classdef (Abstract) MeshCollider < Collider
    %MESHCOLLIDER defines a collider that uses a mesh as a reference for
    %its collision properties.
    
    properties
        Mesh = Mesh.empty;  % Mesh defining the collider
    end
    
    methods
        function [this] = MeshCollider()
            % CONSTRUCTOR - Construct an instance of a mesh-collider.
            
            % Call the parent
            [this] = this@Collider();
        end
        % Get/sets
        function set.Mesh(this,mesh)
            assert(isa(mesh,"Mesh"),"Expecting a valid mesh.");
            this.Mesh = mesh;
        end
    
        function [mesh] = GetWorldMesh(this)
            % Get the mesh tranformed into world space.
            T = this.Transformation.GetWorldMatrix();
            % Transform the collision mesh
            mesh = this.Mesh.TransformBy(T);
        end
    end
    % Collision Utilities
    methods
        function [aabb] = GetWorldAABB(this)
            % This function recalculates the bounding box from the collider
            % properties.

            % Sanity check
            if isempty(this.Mesh) || this.Mesh.NumberOfVertices == 0
                return
            end

            % Get the mesh transformed in world coordinates
            mesh = this.GetWorldMesh();
            % Recompute AABB
            aabb = AABB.FromMesh(mesh);
            % Assign the owner's id
            aabb.Cid = this.Cid;
        end
        function [h] = Draw(this,container)
            % Draw the mesh collider

            % Input check
            if nargin < 2
                container = gca;
            end

            % get the transformed mesh
            mesh = this.GetWorldMesh();
            % Draw the collider to the container
            h = mesh.Draw(container,"r");
        end
    end
end

