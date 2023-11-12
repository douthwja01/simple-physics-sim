
classdef BoxCollider < MeshCollider
    % An Object-Aligned Bounding Box (OBB) collider primitive
    
    properties (Constant)
        Code = ColliderCode.OBB;
    end
    properties
        Center = zeros(3,1);
    end

    methods
        function [this] = BoxCollider()
            % CONSTRUCTOR - Generate a box-collider with a unitary cube mesh.

            % Call the parent
            [this] = this@MeshCollider();
            % Generate a cuboid-mesh
            extents = 0.5*[1;1;1];% ones(3,1);
            this.Mesh = MeshGenerator.CuboidFromExtents(-extents,extents);
        end
        function [points] = TestCollision(this,colliderB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Code
                case ColliderCode.Sphere
                    % The second collider is a sphere
                    points = Collider.FindSphereOBBCollisionPoints(colliderB,this);
                case ColliderCode.Plane
                    % The second collider is a plane
                    points = Collider.FindPlaneOBBCollisionPoints(colliderB,this);
                 case ColliderCode.OBB
                    % The second collider is a plane
                    points = Collider.FindOBBOBBCollisionPoints(this,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end
        end
    end
end