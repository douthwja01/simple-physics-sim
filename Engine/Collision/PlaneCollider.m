
classdef PlaneCollider < Collider
    % A plane collider primitive 

    properties (Constant)
        Code = ColliderCode.Plane;
    end
    properties
        Width = 1;
        Height = 1;
        Normal = [0;0;1];
    end
    methods
        function [this] = PlaneCollider()
            % CONSTRUCTOR - Generate a plane-collider with a normal.

            % Create a collider
            [this] = this@Collider();   
            % Load a plane mesh
            this.RecalculateAABB();
        end
        function set.Normal(this,n)
            this.Normal = n;
            this.RecalculateAABB();
        end
        function [points] = TestCollision(this,colliderB)
            % Test for collision between this collider and a variable
            % second collider.
            
            switch colliderB.Code
                case ColliderCode.Sphere
                    % The second collider is sphere
                    points = Collider.FindSpherePlaneCollisionPoints(colliderB,this);
                case ColliderCode.OBB
                    % The second collider is an OBB box
                    points = Collider.FindPlaneOBBCollisionPoints(this,colliderB);
                otherwise
                    error("Collider type not recognised.");
            end
        end
    end
    methods (Access = protected)
        function [this] = RecalculateAABB(this)
            % This function is called when the normal vector is changed in
            % order to update the mesh that defines the plane to match the
            % new normal.
        
            % Create 
            tempMesh = MeshGenerator.Plane( ...
                zeros(3,1), ...
                this.Normal, ...
                this.Width, ...
                this.Height);
        
            % We need the box enclosing the plane in each dimension.

            % Recompute AABB
            this.AABB = AABB.EncloseMesh(tempMesh);
            this.AABB.Parent = this;
        end
    end
end