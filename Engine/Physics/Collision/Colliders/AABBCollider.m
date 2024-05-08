classdef AABBCollider < Collider
    % An Axis-Aligned Bounding Box (AABB) collider primitive
    
    properties (Constant)
        Code = ColliderCode.AABB;
    end
    properties (Dependent) 
        Min;
        Max;
    end 
    % Intervals in each dimensions
    properties (Access = private)
        AABB = AABB.empty;
    end
    methods
        % Constructor
        function [this] = AABBCollider(xRange,yRange,zRange)
            % CONSTRUCTOR - Generate a box- collider with unitary axial
            % limits.
            % center - 3D center position.
            % xRange - The x-variation around the center.
            % yRange - The y-variation around the center.
            % zRange - The z-variation around the center.

            % Input check
            if nargin < 1
                xRange = [-1;1];
            end
            if nargin < 2
                yRange = [-1;1];
            end
            if nargin < 3
                zRange = [-1;1];
            end

            % Parameterise
            this.AABB = AABB(xRange,yRange,zRange);
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
        % General
        function [mesh] = ToMesh(this)
            % Create a mesh from the current AABB instance.
            mesh = MeshExtensions.CuboidFromExtents(this.Min,this.Max);
        end
        function [h] = Draw(this,container,colour)
            % Draw this mesh to a given graphical container
            if nargin < 3
                colour = 'b';
            end
            if nargin < 2
                container = gca;
            end
            mesh = this.ToMesh();
            h = mesh.Draw(container,colour);
        end
    end
    % AABB Tools
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
        function [isColliding,points] = CheckSphere(this,sphere)
            % Check this collider against a second sphere collider

            % [TO FILL]

            % Collision points
            points = ContactPoints(a,b,unitSeperationAxis,depth,isColliding);
        end
        function [isColliding,points] = CheckPlane(this,plane)
            % Find the collisions points between an AABB and a plane.
            
            % [TO FILL]

            % Return the collision points
            points = ContactPoints(pDepth,sDepth,planeNormal,toResolve,isColliding);
        end
        function [isColliding,points] = CheckCapsule(this,capsule)
            % Find the collision points between an AABB and a capsule.
            points = capsule.CheckAABB(this);
        end
        function [isColliding,points] = CheckAABB(this,aabb)
            % Find the collision points between an AABB and an AABB.
            
            % [TO FILL]

        end
        function [isColliding,points] = CheckOBB(this,obb)
            % Find the collision points between an AABB and an OBB box.
            points = obb.CheckAABB(this);
        end
        function [isColliding,points] = CheckTriangle(this,triangle)
            % Find the collision points between an aabb and a triangle.
        end
        function [isColliding,points] = CheckMesh(this,mesh)
            % Find the collision points between an AABB and a mesh.
            points = mesh.CheckAABB(this);
        end
    end
    methods (Access = protected)
         function [axisInterval] = GetAxisInterval(this,axis)
            % This function creates an interval defining the projection of
            % this shape onto a given axis.
            
            % Ensure same size
            axis = axis'; 
            % Define vertex data from limits
            vertices = this.GetVertices();
            % We need to loop through these points and get the minimum and
            % maximums on the provided axis (axis is assumed to be
            % arbitrary)
            imin = inf;
            imax = -inf;

            for i = 1:size(vertices,1)
                projection = dot(axis,vertices);
                % If the projection is greater record it.
                if projection > imax
                    imax= projection;
                end
                % If the projection is smaller record it.
                if projection < imin
                    imin = projection;
                end
            end
            % Return an interval capturing it.
            axisInterval = Interval(imin,imax);
         end
         function [vertices] = GetVertices(this)
            % This function returns the bounds of the AABB as vertices.

            % Express the bounds as dimension ranges
            xRange = this.Bounds(1).Range;
            yRange = this.Bounds(2).Range;
            zRange = this.Bounds(2).Range;
            % Define vertex data from limits
            vertices = zeros(8,3);
            vertices(1,:) = [xRange(2),yRange(2),zRange(2)];
            vertices(2,:) = [xRange(2),yRange(2),zRange(1)];
            vertices(3,:) = [xRange(2),yRange(1),zRange(1)];
            vertices(4,:) = [xRange(2),yRange(1),zRange(2)];
            vertices(5,:) = [xRange(1),yRange(1),zRange(1)];
            vertices(6,:) = [xRange(1),yRange(1),zRange(2)];
            vertices(7,:) = [xRange(1),yRange(2),zRange(2)];
            vertices(8,:) = [xRange(1),yRange(2),zRange(1)];
        end
    end
end