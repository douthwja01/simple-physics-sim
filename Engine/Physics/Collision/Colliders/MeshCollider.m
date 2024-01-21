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
    end
    % Utilities
    methods
        function [mesh] = GetTransformedMesh(this)
            % Return the mesh transformed to world coordinate frame.
            mesh = this.Mesh.TransformBy(this.Transformation.GetWorldMatrix());
        end
        function [aabb] = GetTransformedAABB(this)
            % This function recalculates the bounding box from the collider
            % properties.

            % Sanity check
            if isempty(this.Mesh) || this.Mesh.NumberOfVertices == 0
                return
            end

            % Get the mesh transformed in world coordinates
            mesh = this.GetTransformedMesh();
            % Recompute AABB
            aabb = AABB.EncloseMesh(mesh);
        end
    
        function [h] = Draw(this,container)
            % Draw the mesh collider

            % Input check
            if nargin < 2
                container = gca;
            end

            % get the transformed mesh
            mesh = this.GetTransformedMesh();
            % Draw the collider to the container
            h = mesh.Draw(container,"r");
        end
    end
end

