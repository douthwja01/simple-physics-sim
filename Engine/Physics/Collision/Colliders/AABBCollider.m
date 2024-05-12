classdef AABBCollider < Collider
    % An Axis-Aligned Bounding Box (AABB) collider primitive

    properties (Constant)
        Code = ColliderCode.AABB;
    end
    properties
        Size = ones(3,1);
    end
    properties (SetAccess = private, Hidden)
        ImaginaryRadius = 1;
    end
    % Intervals in each dimensions
    properties (Access = private)
        AABB = AABB();
    end
    methods
        % Constructor
        function [this] = AABBCollider(size)
            % CONSTRUCTOR - Generate a box- collider with aligned axial
            % limits.

            % Input checks
            if nargin < 1
                size = ones(3,1);
            end
            this.Size = size;
        end
    end

    methods
        function [aabb,cid] = GetWorldAABB(this)
            % Simply return the world AABB

            % Get the world properties
            so3 = this.Transform.Inertial;
            % Offset and scale the unit AABB
            aabb = so3.Position + so3.Scale*this.AABB;
            % Return the collider ID
            cid = this.Cid;
        end
        % Get/Sets
        function set.Size(this,s)
            assert(IsColumn(s,3),"Expecting a valid size vector [3x1].");
            this.Size = s;
            this.RecalculateProperties();
        end
    end
    %% Collision Pairing
    methods
        function [isColliding,points] = CheckPoint(this,point)
            % Find the collision points between an aabb and a point.

            % [TO FILL]
        end
        function [isColliding,points] = CheckLine(this,line)
            % Find the collision points between an aabb and a line.
        end
        function [isColliding,points] = CheckRay(this,ray)
            % Find the collision points between an aabb and a ray.
        end
        function [isColliding,points] = CheckSphere(this,sphere)            % [DONE]
            % Check this collider against a second sphere collider
            [isColliding,points] = sphere.CheckAABB(this);
        end
        function [isColliding,points] = CheckPlane(this,plane)
            % Find the collisions points between an AABB and a plane.

            % [TO FILL]

            % Return the collision points
            points = ContactPoints(pDepth,sDepth,planeNormal,toResolve,isColliding);
        end
        function [isColliding,points] = CheckCapsule(this,capsule)          % [DONE]
            % Find the collision points between an AABB and a capsule.
            [isColliding,points] = capsule.CheckAABB(this);
        end
        function [isColliding,points] = CheckAABB(this,aabb)
            % Find the collision points between an AABB and an AABB.

            % Sanity check
            assert(this.Code == ColliderCode.AABB,"First collider must be a box collider.");
            assert(aabb.Code == ColliderCode.AABB,"Second collider must be a box collider.");

            boxA = this.GetTransformedAABB();
            boxB = aabb.GetTransformedAABB();
            overlap = boxA.IntersectAABB(boxB);

            % If no overlap exists, no collision
            if isempty(overlap)
                isColliding = false;
                points = ContactPoints();
                return;
            end

            % Variables
            isColliding = false;
            so3A = this.Transform.Inertial;
            so3B = aabb.Transform.Inertial;

            distance = norm(so3B.Position - so3A.Position);
            if distance > (this.ImaginaryRadius + aabb.ImaginaryRadius)
                points = ContactPoints();
                return;
            end

            aVertices = this.GetVertices();
            bVertices = aabb.GetVertices();
            minSeparation = inf;
            minSeparationAxis = zeros(3,1);
            % Array of testable axes
            axes = eye(3);
            for i = 1:size(axes,1)
                queryAxis = [axes(i,1);axes(i,2);axes(i,3)];

                [projectionA] = CollisionExtensions.GetAxisInterval(aVertices,queryAxis);
                [projectionB] = CollisionExtensions.GetAxisInterval(bVertices,queryAxis);

                % Intersection projections along the axis
                overlap = projectionA.Intersect(projectionB);
                % An axis exists with no overlap, no collision
                if isempty(overlap)
                    isColliding = false;
                    points = ContactPoints();
                    return;
                end

                % If the separation is less than recorded
                if overlap.Span < minSeparation
                    minAProjection = projectionA;
                    minBProjection = projectionB;
                    minSeparation = overlap.Span;
                    minSeparationAxis = queryAxis;
                end
            end
            
            % Calculate the collision points based on the minimum separation axis
            pointAinB = so3A.Position + (minAProjection.Span/2) * minSeparationAxis;
            pointBinA = so3B.Position - (minBProjection.Span/2) * minSeparationAxis;

%             aLine = [so3A.Position,pointAinB]';
%             bLine = [so3B.Position,pointBinA]';
%             plot3(gca,aLine(:,1),aLine(:,2),aLine(:,3),"r","LineWidth",2);
%             plot3(gca,bLine(:,1),bLine(:,2),bLine(:,3),"b","LineWidth",2);
%             plot3(gca,pointAinB(1),pointAinB(2),pointAinB(3),"r^");
%             plot3(gca,pointBinA(1),pointBinA(2),pointBinA(3),"b^");
%             this.Draw(gca,"c");

            isColliding = true;
            % Construct the points
            points = ContactPoints( ...
                pointAinB, ...
                pointBinA, ...
                -minSeparationAxis, ...
                minSeparation, ...
                isColliding);
        end
        function [isColliding,points] = CheckOBB(this,obb)                  % [DONE]
            % Find the collision points between an AABB and an OBB box.
            [isColliding,points] = obb.CheckAABB(this);
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Find the collision points between an aabb and a triangle.
        end
        function [isColliding,points] = CheckMesh(this,mesh)                % [DONE]
            % Find the collision points between an AABB and a mesh.
            [isColliding,points] = mesh.CheckAABB(this);
        end
    end
    %% Utilities
    methods
        function [p] = NearestPoint(this,point)
            % This function calculates the nearest point on this collider
            % to a given point.

            worldAABB = this.GetTransformedAABB();
            box_min = worldAABB.Min;
            box_max = worldAABB.Max;
            % Minumum mapping
            xx = point(1);
            if xx < box_min(1)
                xx = box_min(1);
            end
            yy = point(2);
            if yy < box_min(2)
                yy = box_min(2);
            end
            zz = point(3);
            if zz < box_min(3)
                zz = box_min(3);
            end
            % Maximum mapping
            if xx > box_max(1)
                xx = box_max(1);
            end
            if yy > box_max(2)
                yy = box_max(2);
            end
            if zz > box_max(3)
                zz = box_max(3);
            end
            % Return the contrained point
            p =[xx;yy;zz];
        end
        function [mesh] = ToMesh(this)
            % Create a mesh from the current AABB instance.
            aabb = this.AABB;
            mesh = MeshExtensions.CuboidFromExtents(aabb.Min,aabb.Max);
        end
        function [h] = Draw(this,container,colour)
            % Draw this mesh to a given graphical container
            if nargin < 3
                colour = 'b';
            end
            if nargin < 2
                container = gca;
            end
            % Get the aabb in the world space
            aabb = this.GetTransformedAABB();
            % Draw 
            h = aabb.Draw(container,colour);
            set(h,"FaceAlpha",0.2);
        end        
        function [aabb] = GetTransformedAABB(this)
            % Return the AABB transformed in the world-space.
            so3 = this.Transform.Inertial;
            aabb = so3.Position + so3.Scale*this.AABB;
        end
    end
    methods (Static)
        function [aabb] = FromMesh(mesh)
            % This function creates an axis-aligned-bounding box from a
            % given mesh.

            % Sanity check
            assert(isa(mesh,"Mesh"),"Expecting a valid mesh class.");

            % Simply extract the extents
            [xlimits,ylimits,zlimits] = Mesh.Extents(mesh);
            % Create the primitive
            aabb = AABBCollider(xlimits,ylimits,zlimits);
        end
        function [collider] = FromAABB(aabb,cid)

            % Create an equivalent aabb
            collider = AABBCollider(aabb.Range(1),aabb.Range(2),aabb.Range(3));
            collider.Cid = cid;
        end
    end
    methods (Access = protected)
        function [vertices] = GetVertices(this)
            % This function returns the bounds of the AABB as vertices.

            % Transform the aabb
            aabb = this.GetTransformedAABB();
            % Convert the vertices
            vertices = aabb.ToVertices();
        end
        function [this] = RecalculateProperties(this)
            % This function recalculates the un-transformed AABB
            % property.
            dims = this.Size/2;
            this.AABB.Min = -dims;
            this.AABB.Max = dims;
            % Imaginary radius update
            this.ImaginaryRadius = norm(dims);
        end
    end
end