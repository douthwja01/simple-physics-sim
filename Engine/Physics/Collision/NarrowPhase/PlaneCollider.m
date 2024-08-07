
classdef PlaneCollider < Collider
    % A plane collider primitive

    properties (Constant)
        Code = ColliderCode.Plane;
    end
    properties
        Width = 1;
        Depth = 1;
        Thickness = 0.05;
        Normal = [0;0;1];
        Distance = 0; % The position of the plane along the normal
    end
    methods
        function [this] = PlaneCollider()
            % CONSTRUCTOR - Generate a plane-collider with a normal.

            % Create a collider
            [this] = this@Collider();
        end
        function set.Normal(this,n)
            assert(IsColumn(n,3),"Expecting a normal [3x1].");
            this.Normal = n;
        end
    end
    %% Collision Pairing
    methods
        function [isColliding,points] = CheckPoint(this,point)              % [DONE, TO TEST]
            % Find the collision points between the plane and a point
            
            result = this.PlaneEquation(point);

            isColliding = false;
            if result ~= 0
                points = ContactPoints();
                return;
            end

            so3 = this.Transform.Inertial;
            normal = so3.Rotation.GetMatrix()*this.Normal;

            relativePosition = point - so3.Position;

            distance = dot(relativePosition,normal);
            % The plane surface point (closest)
            closestPlanePoint = point - distance*normal;

            % Create the points structure
            points = ContactPoints( ...
                point, ...
                closestPlanePoint, ...
                normal, ...
                abs(distance), ...
                isColliding);
        end
        function [isColliding,points] = CheckLine(this,line)
            % Find the collision points between the plane and a line.
        end
        function [isColliding,points] = CheckRay(this,ray)
            % Find the collision points between the plane and a ray.
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Find the collision points between the plane and a triangle.

            % [TO FILL]
        end
        function [isColliding,points] = CheckSphere(this,sphere)            % [DONE]
            % Find the collision points between this plane and a sphere.
            [isColliding,points] = sphere.CheckPlane(this);
        end
        function [isColliding,points] = CheckPlane(this,plane)              % [DONE,TO TEST]
            % Find the collision points between this OBB box and a plane.

            % Sanity check
            assert(plane.Code == ColliderCode.Plane,"Second collider must be a plane collider.");
            
            so3PlaneA = this.Transform.Inertial;
            so3PlaneB = plane.Transform.Inertial;
            planeNormalA = so3PlaneA.Rotation.GetMatrix()*this.Normal;
            planeNormalB = so3PlaneB.Rotation.GetMatrix()*plane.Normal;

            % The plane argument
            normalArg = cross(planeNormalA,planeNormalB);
            % If the arguement is null.
            isColliding = norm(normalArg) > 0 || (this.Distance == plane.Distance);
            % No collision, abort
            if ~isColliding
                points = ContactPoints();
                return;
            end

            relativeVector = so3PlaneB.Position - so3PlaneA.Position;
            % Projection of center "A"
            aProjection = dot(planeNormalB,relativeVector);
            % Projection of center "B" 
            bProjection = dot(planeNormalA,relativeVector);
            % The closest points on the respective planes
            planeAPointOnB = so3PlaneA.Position + aProjection*planeNormalB;
            planeBPointOnA = so3PlaneB.Position - bProjection*planeNormalA;

%             % Debugging
%             aLine = [so3PlaneA.Position,planeAPointOnB]';
%             bLine = [so3PlaneB.Position,planeBPointOnA]';
%             plot3(gca,aLine(:,1),aLine(:,2),aLine(:,3),"r","LineWidth",2);
%             plot3(gca,bLine(:,1),bLine(:,2),bLine(:,3),"b","LineWidth",2);
%             plot3(gca,planeAPointOnB(1),planeAPointOnB(2),planeAPointOnB(3),"r^","LineWidth",2);
%             plot3(gca,planeBPointOnA(1),planeBPointOnA(2),planeBPointOnA(3),"b^","LineWidth",2);

            % To check
            axis = relativeVector/norm(relativeVector);
            depth = 0;  
            % Return the points structure
            points = ContactPoints( ...
                planeAPointOnB, ...
                planeBPointOnA, ...
                axis, ...
                depth, ...
                isColliding);
        end
        function [isColliding,points] = CheckCapsule(this,capsule)          % [DONE]
            % Find the collision points between a plane and a capsule.
            [isColliding,points] = capsule.CheckPlane(this);
        end
        function [isColliding,points] = CheckAABB(this,aabb)                % [DONE]
            % Find the collision points between a plane and an AABB.
            [isColliding,points] = aabb.CheckPlane(this);
        end
        function [isColliding,points] = CheckOBB(this,obb)                  % [DONE]   
            % Find the collision points between a plane and an obb.
            [isColliding,points] = obb.CheckPlane(this);
        end
        function [isColliding,points] = CheckMesh(this,mesh)                % [DONE]
            % Find the collision points between a plane and a mesh.
            [isColliding,points] = mesh.CheckPlane(this);
        end
    end
    %% Utilties
    methods 
        function [p] = PlaneEquation(this,point)
            % This function determines whether a given point lies on one
            % side of the plane.
            % +1 - In front of the plane
            %  0 - On the plane
            % -1 - Behind the plane

            d = dot(point,this.Normal);
            p = d - this.Distance;
        end
        function [p] = NearestPoint(this,point)
            % This function retrieves the point on the plant nearest to a
            % given point.

            R = this.Transform.Inertial.Rotation.GetMatrix();

            normal = R*this.Normal;

            dist = dot(normal,point) - this.Distance;
            scaled_dist = normal*dist;
            p = point - scaled_dist;
        end
        function [aabb,cid] = GetWorldAABB(this)
            % This function is called when the normal vector is changed in
            % order to update the mesh that defines the plane to match the
            % new normal.

            % Recompute AABB...

            % Generate patch
            mesh = MeshExtensions.UnitPlane();
            % Scale to configuration dimensions
            toSize = mesh.ScaleBy(this.Depth,this.Width,1);
            % Transform by transformation in space
%             so3 = this.Transform.Inertial;
            T = this.Transform.GetWorldMatrix();

            mesh = toSize.TransformBy(T);
            
            % aabb from extents
            aabb = AABB.FromMesh(mesh);
            % Return the collider ID
            cid = this.Cid;
        end
        function [this] = Normalize(this)
            % Normalize the plane object (this.Normal doesn't have to be
            % unitary).
            
            mag = norm(this.Normal);
            this.Normal = this.Normal/mag;
            this.Distance = this.Distance/mag;
        end
        function [h] = Draw(this,container,colour)
            % Allow the drawing of this collider to a given container.
            if nargin < 3
                colour = 'b';
            end
            if nargin < 2
                container = gca;
            end
            % Generate patch
            mesh = MeshExtensions.UnitPlane();
            % Scale to configuration dimensions
            toSize = mesh.ScaleBy(this.Depth,this.Width,1);
            % Transform by transformation in space
            so3 = this.Transform.Inertial;
            mesh = toSize.TransformBy(so3.GetMatrix());
            % Draw
            h = mesh.Draw(container,colour);
            set(h,"FaceAlpha",0.2);
        end
    end
end