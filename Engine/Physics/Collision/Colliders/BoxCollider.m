
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

    % Main methods
    methods
        function [this] = BoxCollider()
            % CONSTRUCTOR - Generate a box-collider with a unitary cube mesh.

            % Call the parent
            [this] = this@MeshCollider();
            % Generate a cuboid-mesh
            this.Mesh = MeshExtensions.UnitCube();
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
        function [mesh] = GetWorldMesh(this)
            % Calculate the boxes mesh in the world frame.

            % Transform the mesh by the box dimensions
            mesh = this.Mesh.ScaleBy(this.Width,this.Height,this.Depth);
            % Transform the base mesh (by scale and position)
            mesh = mesh.TransformBy(this.Transform.Inertial.GetMatrix());
%             mesh = mesh.TransformBy(this.Transform.GetWorldMatrix());
        end
    end
end