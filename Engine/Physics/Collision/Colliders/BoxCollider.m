
classdef BoxCollider < MeshCollider
    % An Object-Aligned Bounding Box (OBB) collider primitive

    properties (Constant)
        Code = ColliderCode.OBB;
    end
    properties
        Width = 1;
        Depth = 1;
        Height = 1;
    end

    methods
        function [this] = BoxCollider()
            % CONSTRUCTOR - Generate a box-collider with a unitary cube mesh.

            % Call the parent
            [this] = this@MeshCollider();
            % Generate a cuboid-mesh
            this.Mesh = MeshExtensions.UnitCube();
        end
    end
    % Utilities
    methods
        function [points] = TestCollision(this,colliderB)
            % Test for collision between this collider and a variable
            % second collider.

            switch colliderB.Code
                case ColliderCode.Sphere
                    % The second collider is a sphere
                    points = CollisionExtensions.FindSphereOBBContactPoints(colliderB,this);
                case ColliderCode.Plane
                    % The second collider is a plane
                    points = CollisionExtensions.FindPlaneOBBContactPoints(colliderB,this);
                case ColliderCode.OBB
                    % The second collider is a plane
                    points = CollisionExtensions.FindOBBOBBContactPoints(this,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end
        end
        function [mesh] = GetTransformedMesh(this)
            % Convert the default mesh to the rendered mesh.
            % (Overridden in parent classes).

            % Get the base mesh
            mesh = GetTransformedMesh@MeshCollider(this);
            % Get the transformed mesh
            mesh = mesh.ScaleBy(this.Width,this.Height,this.Depth);
        end
        function [aabb] = GetTransformedAABB(this)
            % Overrided needed, sphere colliders do not need a mesh
            % representation as a unitary radius is valid. However, for
            % AABB comparison, a primitive must be constructed any way to
            % represent this radius.

            % Sanity check
            if isempty(this.Mesh) || this.Mesh.NumberOfVertices == 0
                return
            end

            % Get the mesh transformed in world coordinates
            mesh = this.GetTransformedMesh();
            % Compute the aabb for the mesh
            aabb = AABB.EncloseMesh(mesh);
        end
    end
end