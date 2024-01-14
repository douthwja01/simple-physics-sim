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
            % Update the AABB
            this.RecalculateAABB();
        end

        function [mesh] = GetTransformedMesh(this)
            % Return the mesh transformed to world coordinate frame.
            mesh = this.Mesh.TransformBy(this.Pose.GetWorldMatrix());
        end
    end
    % Internals
    methods (Access = protected)
        function [this] = RecalculateAABB(this)
            % This function recalculates the bounding box from the collider
            % properties.

            % Sanity check
            if isempty(this.Mesh) || this.Mesh.NumberOfVertices == 0
                return
            end
            % Recompute AABB
            this.AABB = AABB.EncloseMesh(this.Mesh);
            this.AABB.Parent = this;
        end
    end
end

