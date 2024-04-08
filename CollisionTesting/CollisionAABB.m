classdef CollisionAABB < CollisionPrimitive

    % https://www.youtube.com/watch?v=tu2FViMrwLc

    properties
        Bounds = Interval.empty(3,0);
    end
    % Main
    methods 
        function [this] = CollisionAABB()
            % COLLISIONAABB Create an instance of the class.

            % Populate bounds
            this.Bounds(1) = Interval();
            this.Bounds(2) = Interval();
            this.Bounds(3) = Interval();
        end
    end

    % Core methods
    methods   
        function [flag] = CheckPoint(this,point)
            % Check if a given point lies within the AABB.
            flag = point.CheckAABB(this);
        end
        function [flag] = CheckSphere(this,sphere)
            % Check if this aabb is colliding with a given sphere
            % primitive.
            flag = sphere.CheckAABB(this);
        end
        function [flag] = CheckAABB(this,aabb)
            % Check if there is an overlap between this and another aabb.

            flag = false;
            for i = 1:3
                % Is there is axial intersection
                if ~this.Bounds(i).IsIntersecting(aabb.Bounds(i))
                    % Non-intersection found, abort.
                    return;
                end
            end            
            flag = true;
        end
        function [flag] = CheckPlane(this,plane)
            % Check if this aabb is colliding with a given plane
            % primitive.
            
            he = this.half_extents;
            
            anorm = norm(plane.Normal);
            plength = dot(he,anorm);
            ndot = dot(plane.Normal,this.Position);
            dist = ndot - plane.Distance;
            flag = abs(dist) <= plength;
        end
        function [flag] = CheckOBB(this,obb)
            % Check if this aabb is colliding with a given obb
            % primitive.
            flag = obb.CheckAABB(this);
        end
        function [flag] = CheckCapsule(this,capsule)
            % Check if this aabb is colliding with a given capsule
            % primitive.
            flag = capsule.CheckAABB(this);
        end
        function [flag] = CheckTriangle(this,triangle)
            % Check if this aabb is colliding with a given triangle
            % primitive.

            % Get the vertex normals
            ab = triangle.B - triangle.A;
            bc = triangle.C - triangle.B;
            ca = triangle.A - triangle.C;
        
            nx = [1, 0, 0];
            ny = [0, 1, 0];
            nz = [0, 0, 1];
        
            axes = zeros(13,3);
            axes(1,:) = nx;
            axes(2,:) = ny;
            axes(3,:) = nz;
            axes(4,:) = triangle.GetNormal();
            axes(5,:) = cross(nx,ab);
            axes(6,:) = cross(nx,bc);
            axes(7,:) = cross(nx,ca);
            axes(8,:) = cross(ny,ab);
            axes(9,:) = cross(ny,bc);
            axes(10,:) = cross(ny,ca);
            axes(11,:) = cross(nz,ab);
            axes(12,:) = cross(nz,bc);
            axes(13,:) = cross(nz,ca);
    
            % Evaluate overlap on each axis
            for i = 1:13
                if ~this.CollisionOverlapAxis(self, triangle, axes(i,:))
                    flag = false;
                    return
                end
            end
            flag = true;
        end
        function [flag] = CheckRay(this,ray)
            % Check if this aabb is colliding with a given ray
            % primitive.

            flag = false;

            % The aabb extents
            box_min = [this.Bounds.Min];
            box_max = [this.Bounds.Max];
            
            ray_x = (ray.direction.x == 0) ? 0.0001 : ray.direction.x;
            ray_y = (ray.direction.y == 0) ? 0.0001 : ray.direction.y;
            ray_z = (ray.direction.z == 0) ? 0.0001 : ray.direction.z;
            
            t1 = (box_min.x - ray.origin.x) / ray_x;
            t2 = (box_max.x - ray.origin.x) / ray_x;
            t3 = (box_min.y - ray.origin.y) / ray_y;
            t4 = (box_max.y - ray.origin.y) / ray_y;
            t5 = (box_min.z - ray.origin.z) / ray_z;
            t6 = (box_max.z - ray.origin.z) / ray_z;
            
            tmin = max(
                min(t1, t2),
                min(t3, t4),
                min(t5, t6)
            );
            var tmax = min(
                max(t1, t2),
                max(t3, t4),
                max(t5, t6)
            );
            
            if tmax < 0
                return
            end
            if tmin > tmax
                return
            end
            
%             t = tmax;
%             if tmin > 0
%                 t = tmin;
%             end
            
%             % Project the ray to the de
%             contact_point = ray.Origin + ray.Direction*t;
%             
%             tnormal = zero(3,1);
%             if t == t1
%                 tnormal = [-1; 0; 0];
%             end
%             if t == t2
%                 tnormal = [1; 0; 0];
%             end
%             if t == t3
%                 tnormal = [0; -1; 0];
%             end
%             if t == t4 
%                 tnormal = [0; 1; 0];
%             end
%             if t == t5 
%                 tnormal = [0; 0; -1];
%             end
%             if t == t6 
%                 tnormal = [0; 0; 1];
%             end
            %hit_info.Update(t, self, contact_point, tnormal);
            
            flag = true;
        end
        function [flag] = CheckLine(this,line)
            % Check if this aabb is colliding with a given line
            % primitive.

            dir = line.finish - line.start;
            dir = dir/norm(dir);

            ray = Ray(line.start, dir);
            % hit_info = new RaycastHitInformation();
            if this.CheckRay(ray, hit_info) 
                flag = hit_info.distance <= line.Distance; %Length()
                return;
            end
            flag = false;
        end
        function [flag] = CheckMesh(this,mesh)
            % Check if this aabb is colliding with a given mesh
            % primitive.
            flag = mesh.CheckAABB(this);
        end
    end
    % Support methods
    methods
        function [flag] = CollisionOverlapAxis(this,triangle,axis)


        end
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