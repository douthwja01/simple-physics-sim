classdef CapsuleCollider < Collider

    properties (Constant)
        Code = ColliderCode.Capsule;
    end
    properties
        Radius = 1;
        Length = 1;
    end

    methods
        function [this] = CapsuleCollider()
            % CONSTRUCTOR - Construct an instance of a capsule-collider.
            
            % Call the parent
            [this] = this@Collider();
        end
        function [aabb] = GetWorldAABB(this)
            % This function recalculates the bounding box from the collider
            % properties.

            % Recompute AABB
            aabb = AABBCollider();
            % Assign the owner's id
            aabb.Cid = this.Cid;
        end
    end
    %% Collision Pairing
    methods
        function [points] = CheckPoint(this,point)
            % Find the collision points between the capsule and a point.

            % [TO FILL]
        end
        function [points] = CheckLine(this,line)
            % Find the collision points between the capsule and a line.

            % [TO FILL]
        end
        function [points] = CheckRay(this,ray)
            % Find the collision points between the capsule and a ray.

            % [TO FILL]
        end
        function [points] = CheckSphere(this,sphere)
            % Find the collision points between the capsule and a sphere.
            
            % [TO FILL]
        end
        function [points] = CheckPlane(this,plane)
            % Find the collision points between the capsule and a plane.

            % [TO FILL]
        end
        function [points] = CheckCapsule(this,capsule)
            % Find the collision points between the capsule and a capsule.

            % [TO FILL]
        end
        function [points] = CheckAABB(this,aabb)
            % Find the collision points between the capsule and an AABB.

            % [TO FILL]
        end
        function [points] = CheckOBB(this,obb)
            % Find the collision points between the capsule and an obb.

            % [TO FILL]
        end
        function [points] = CheckTriangle(this,triangle)
            % Find the collision points between the capsule and a triangle.

            % [TO FILL]
        end
        function [points] = CheckMesh(this,mesh)
            % Find the collision points between the capsule and a mesh.
            
            % [TO FILL]
        end
    end
end