classdef SphereCollider < Collider
    % A sphere collider primitive

    properties (Constant)
        Code = ColliderCode.Sphere;
    end
    properties
        Radius = 1;
    end

    methods
        function [this] = SphereCollider()
            % CONSTRUCTOR - Sphere collider.
        end
        % Get/sets
        function set.Radius(this,r)
            assert(isnumeric(r),"Expecting a numerical radius.");
            this.Radius = r;
        end
    end
    %% Collision Pairing
    methods
        function [isColliding,points] = CheckPoint(this,point)
            % Check the collision points between a sphere and a point.
        end
        function [isColliding,points] = CheckLine(this,line)
            % Check the collision points between a sphere and a line.
        end
        function [isColliding,points] = CheckRay(this,ray)
            % Check the collision points between a sphere and a ray.
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Check the collision points between a sphere and a triangle.
        end
        function [isColliding,points] = CheckSphere(this,sphere)            % [DONE]
            % Check this collider against a second sphere collider

            % Sanity check
            assert(this.Code == ColliderCode.Sphere,"First collider must be a sphere collider.");
            assert(sphere.Code == ColliderCode.Sphere,"Second collider must be a sphere collider.");

            % Pull out the world positions
            positionA = this.Transform.Inertial.Position;
            positionB = sphere.Transform.Inertial.Position;

            % Separation axis
            seperationAxis = positionA - positionB;
            distance = norm(seperationAxis);
            unitSeperationAxis = seperationAxis/distance;
            % Points furthest towards each other
            pointAinB = positionA + unitSeperationAxis*this.Radius;
            pointBinA = positionB - unitSeperationAxis*sphere.Radius;
            % The overlap distance
            depth = distance - (this.Radius + sphere.Radius);
            isColliding = ~(depth > 0);
            % No collision
            if ~isColliding
                points = ContactPoints();
                return;
            end

            % Collision points
            points = ContactPoints( ...
                pointAinB, ...
                pointBinA, ...
                unitSeperationAxis, ...
                depth, ...
                isColliding);
        end
        function [isColliding,points] = CheckPlane(this,plane)
            % Find the collisions points between a sphere and a plane.

            % Sanity check
            assert(plane.Code == ColliderCode.Plane,"Second collider must be a plane collider.");

            % Pull out the transforms
            sWorldPosition = this.Transform.Inertial.Position;
            % Origin positions in the world
            pWorldPosition = plane.Transform.Inertial.Position;

            % Sphere properties
            aCenter = sWorldPosition;
            aRadius = this.Radius; % * sTransform.GetWorldScale();
            planeNormal = plane.Normal/norm(plane.Normal);

            % Planar projection
            distance = dot(sWorldPosition - pWorldPosition,planeNormal);

            % Is it colliding
            isColliding = ~(distance > aRadius);
            if ~isColliding
                points = ContactPoints();
                return;
            end
            % Calculate the scalar distance to be resolved
            toResolve = distance - aRadius;
            sDepth = aCenter - planeNormal * aRadius;
            pDepth = aCenter - planeNormal * toResolve;
            % Return the collision points
            points = ContactPoints(pDepth,sDepth,planeNormal,toResolve,isColliding);
        end
        function [isColliding,points] = CheckCapsule(this,capsule)          % [DONE]
            % Find the collision points between a sphere and a capsule.
            points = capsule.CheckSphere(this);
        end
        function [isColliding,points] = CheckAABB(this,aabb)
            % Find the collision points between a sphere and an AABB.
            
            so3 = this.Transform.Inertial;
            abbSo3 = aabb.Transform.Inertial;

            % Check spherical separation first
            relativePosition = abbSo3.Position - so3.Position;
            distance = norm(relativePosition);
            if distance > aabb.ImaginaryRadius + this.Radius
                isColliding = false;
                points = ContactPoints();
                return
            end
            
            % Point inside or on boundary of AABB
            nearest = aabb.NearestPoint(so3.Position);

            nearestVector = nearest - so3.Position;
            distance = norm(nearestVector);
            unitSeperationAxis = nearestVector/distance;
            % Collision data
            isColliding = distance < this.Radius;
            spherePointInBox = so3.Position + this.Radius*unitSeperationAxis;
            boxPointInSphere = nearest;
            depth = norm(spherePointInBox - boxPointInSphere);

%             line = [so3.Position,nearest]';
%             plot3(gca,line(:,1),line(:,2),line(:,3),"r","LineWidth",2);
%             %aabb.Draw(gca,"g",0.2);         
%             plot3(gca,spherePointInBox(1),spherePointInBox(2),spherePointInBox(3),"r^","LineWidth",2);
%             plot3(gca,boxPointInSphere(1),boxPointInSphere(2),boxPointInSphere(3),"b^","LineWidth",2);

            % Collision points
            points = ContactPoints( ...
                spherePointInBox, ...
                boxPointInSphere, ...
                unitSeperationAxis, ...
                depth, ...
                isColliding);
        end
        function [isColliding,points] = CheckOBB(this,obb)                  % [DONE]
            % Find the collision points between a sphere and an OBB box.
            [isColliding,points] = obb.CheckSphere(this);
            % Invert the result
            points = points.Swap();
        end
        function [isColliding,points] = CheckMesh(this,mesh)                % [DONE]
            % Find the collision points between a sphere and a mesh.
            points = mesh.CheckSphere(this);
        end
    end
    %% Utilities
    methods
        function [h] = Draw(this,container,colour)
            % Allow the visualisation of this collider in a given
            % container.

            % Sanity check
            if nargin < 3
                colour = "c";
            end
            if nargin < 2
                container = gca;
            end
            % Create a visual mesh    
            so3 = this.Transform.Inertial;
            mesh = MeshExtensions.Sphere(so3.Position,this.Radius,10);
            % Draw the mesh
            h = mesh.Draw(container,colour);
            set(h,"FaceAlpha",0.2);
        end
        function [aabb,cid] = GetWorldAABB(this)
            % Overrided needed, sphere colliders do not need a mesh
            % representation as a unitary radius is valid. However, for
            % AABB comparison, a primitive must be constructed any way to
            % represent this radius.

            % Recompute AABB
            r = this.Radius;
            unit_aabb = AABB([-r,r],[-r,r],[-r,r]);
            % Get the world properties
            so3 = this.Transform.Inertial;
            % Offset and scale the unit AABB
            aabb = so3.Position + so3.Scale*unit_aabb;
            % Assign the owner's id
            cid = this.Cid;
        end
    end
end