
classdef BoxCollider < Collider
    % An Object-Aligned Bounding Box (OBB) collider primitive.
    % Distinct from mesh-colliders as its collider representation is
    % simpler than a standard mesh.

    properties (Constant)
        Code = ColliderCode.OBB;
    end
    properties
        Width = 1;
        Depth = 1;
        Height = 1;
    end

    % Main methods
    methods
        function [this] = BoxCollider()
            % CONSTRUCTOR - Generate a box-collider with a unitary cube mesh.

            % Call the parent
            [this] = this@Collider();
        end
    end
    % Collision Utilities
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

        function [n] = GetWorldFaceNormals(this)
            % The standard normals
            n_unit = [
                 1,0,0;
                0, 1,0;
                0,0, 1;
                0,0,-1;
                0,-1,0;
                -1,0,0];
            % Return the rotated normals
            n = this.Transform.GetWorldRotationMatrix()*n_unit;
        end

        function [mesh] = GetWorldMesh(this)
            % Calculate the boxes mesh in the world frame.

            mesh = MeshExtensions.UnitCube();
            % Transform the mesh by the box dimensions
            mesh = mesh.ScaleBy(this.Width,this.Height,this.Depth);
            % Transform the base mesh (by scale and position)
            mesh = mesh.TransformBy(this.Transform.GetWorldMatrix());
        end

        function [aabb] = GetWorldAABB(this)
            % This function recalculates the bounding box from the collider
            % properties.

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