classdef MeshCollider < Collider
    %MESHCOLLIDER defines a collider that uses a mesh as a reference for
    %its collision properties.
    
    properties (Constant)
        Code = ColliderCode.Mesh;
    end
    properties
        Mesh = Mesh.empty;
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
    % Legacy
    methods
        function [mesh] = GetWorldMesh(this)
            % Get the mesh tranformed into world space.
            T = this.Transformation.GetWorldMatrix();
            % Transform the collision mesh
            mesh = this.Mesh.TransformBy(T);
        end
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
    %% Collision Pairing
    methods
        function [isColliding,points] = CheckPoint(this,point)
            % Find the collision points between a mesh and a point.

            % [TO FILL]
        end
        function [isColliding,points] = CheckLine(this,line)
            % Find the collision points between a mesh and a line.

            % [TO FILL]
        end
        function [isColliding,points] = CheckRay(this,ray)
            % Find the collision points between a mesh and a ray.

            % [TO FILL]
        end
        function [isColliding,points] = CheckSphere(this,sphere)
            % Find the collision points between this mesh and a sphere.
            
            % [TO FILL]
        end
        function [isColliding,points] = CheckPlane(this,plane)
            % Find the collision points between this mesh and a plane.

            % [TO FILL]
        end
        function [isColliding,points] = CheckCapsule(this,capsule)
            % Find the collision points between a mesh and a capsule.

            % [TO FILL]
        end
        function [isColliding,points] = CheckAABB(this,aabb)
            % Find the collision points between a mesh and an AABB.

            % [TO FILL]
        end
        function [isColliding,points] = CheckOBB(this,obb)
            % Find the collision points between a mesh and an obb.

            % [TO FILL]
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Find the collision points between the mesh and a triangle.

            % [TO FILL]
        end
        function [isColliding,points] = CheckMesh(this,mesh)
            % Find the collision points between a mesh and a mesh.
            
            % [TO FILL]
        end
    end
end

